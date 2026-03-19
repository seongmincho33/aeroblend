//! `AeroBlend` glTF/glb importer.
//!
//! Loads aircraft models from Blender-exported glTF files,
//! extracts meshes, classifies parts, and derives physics metadata.

// JSF-Rust SR-801: Enable clippy lints for code quality
#![warn(clippy::all, clippy::pedantic)]
#![allow(clippy::similar_names)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::wildcard_imports)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_lossless)] // i32 to f64 casts are clear in context
#![allow(clippy::missing_errors_doc)] // internal functions
#![allow(clippy::missing_panics_doc)] // unwrap on sort is safe with floats
#![allow(clippy::if_not_else)] // clearer logic with !is_empty()
#![allow(clippy::double_must_use)] // Result already implies must_use
#![allow(clippy::cast_possible_truncation)] // f64->f32, usize->u32 acceptable for mesh data
#![allow(clippy::uninlined_format_args)] // clearer with positional args in some cases
#![allow(clippy::suboptimal_flops)] // clearer midpoint calculation
#![allow(clippy::redundant_closure)] // clearer with explicit closures
#![allow(clippy::redundant_closure_for_method_calls)] // clearer with explicit closures
#![allow(clippy::map_unwrap_or)] // clearer pattern
#![allow(clippy::manual_midpoint)] // overflow not a concern for airfoil coordinates

// JSF-Rust SR-803: Forbid unsafe code in non-FFI crates
#![forbid(unsafe_code)]

pub mod airfoil;
pub mod loader;
pub mod parts;

use aeroblend_physics::types::*;
use glam::DVec3;

/// High-level model loader: loads a .glb/.gltf file and returns a complete `AircraftModel`.
pub fn load_aircraft_model(path: &str) -> Result<AircraftModel, String> {
    let loaded = loader::load_gltf(path)?;
    let classified = parts::classify_meshes(&loaded.meshes);
    let mut aero_surfaces = parts::extract_aero_surfaces_with_meshes(&classified, &loaded.meshes);
    let mut engines = parts::extract_engines_with_meshes(&classified, &loaded.meshes);

    // Fix coordinate system: Blender exports glTF with forward=-Z,
    // but the physics engine expects forward=+Z. Negate Z on all
    // physics-related positions and directions to align conventions.
    // (Mesh vertices stay in glTF coords — the C++ renderer already
    // applies a 180° Y rotation for visual rendering.)
    for surf in &mut aero_surfaces {
        surf.position_local.z = -surf.position_local.z;
    }
    for eng in &mut engines {
        eng.position_local.z = -eng.position_local.z;
        eng.direction_local.z = -eng.direction_local.z;
    }

    // Compute bounding box
    let (bbox_min, bbox_max) = compute_bounding_box(&loaded.meshes);

    // Estimate mass from wing loading (primary) or bounding box (fallback).
    // Typical wing loading: 60-80 kg/m² for general aviation, ~63 for An-2.
    let extent = bbox_max - bbox_min;
    let volume = extent.x.abs() * extent.y.abs() * extent.z.abs();
    let total_wing_area: f64 = aero_surfaces.iter().map(|s| s.area_m2).sum();
    let mass_kg = if total_wing_area > 1.0 {
        // Wing-loading based estimate: 70 kg/m² is a reasonable middle ground
        (total_wing_area * 70.0).clamp(1000.0, 30_000.0)
    } else if volume > 0.1 {
        // Fallback: conservative bounding-box density (~8 kg/m³ accounts for
        // the fact that bounding box is mostly empty space)
        (volume * 8.0).clamp(1000.0, 20_000.0)
    } else {
        5000.0
    };

    // Estimate inertia tensor from bounding box with hollow-body correction.
    // A solid uniform-density box gives ~10x too much inertia for aircraft,
    // because most mass is concentrated near the center (engine, fuel, cockpit)
    // while the bounding box is mostly empty space (wings, tail boom).
    // Factor 0.08 empirically matches general aviation aircraft:
    //   An-2 real:     Ix=12000, Iy=18000, Iz=8000
    //   Solid box:     Ix~90000, Iy~230000, Iz~163000  (7-20x too large)
    //   0.08 × solid:  Ix~7200,  Iy~18400,  Iz~13000   (correct ballpark)
    let hollow_factor = 0.08;
    let (dx, dy, dz) = (extent.x.abs(), extent.y.abs(), extent.z.abs());
    let ix = mass_kg / 12.0 * (dy * dy + dz * dz) * hollow_factor;
    let iy = mass_kg / 12.0 * (dx * dx + dz * dz) * hollow_factor;
    let iz = mass_kg / 12.0 * (dx * dx + dy * dy) * hollow_factor;
    let inertia = glam::DMat3::from_diagonal(DVec3::new(ix.max(1000.0), iy.max(1000.0), iz.max(1000.0)));

    // Scale engine thrust based on aircraft mass for realistic flight.
    // Hardcoded per-engine thrust (50kN jet, 20kN prop) often gives absurd
    // thrust/weight ratios. Target T/W ≈ 0.35 for general aviation.
    if !engines.is_empty() {
        let desired_tw = 0.35;
        let total_needed = desired_tw * mass_kg * 9.81;
        let per_engine = total_needed / engines.len() as f64;
        for eng in &mut engines {
            eng.max_thrust_n = per_engine;
        }
    }

    Ok(AircraftModel {
        meshes: loaded.meshes,
        aero_surfaces,
        engines,
        mass_kg,
        inertia_tensor: inertia,
        bounding_box_min: bbox_min,
        bounding_box_max: bbox_max,
    })
}

fn compute_bounding_box(meshes: &[MeshData]) -> (DVec3, DVec3) {
    let mut min = DVec3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = DVec3::new(f64::MIN, f64::MIN, f64::MIN);

    for mesh in meshes {
        for v in &mesh.vertices {
            // JSF-Rust SR-501: Use From/Into for type conversion instead of `as` cast
            min.x = min.x.min(f64::from(v[0]));
            min.y = min.y.min(f64::from(v[1]));
            min.z = min.z.min(f64::from(v[2]));
            max.x = max.x.max(f64::from(v[0]));
            max.y = max.y.max(f64::from(v[1]));
            max.z = max.z.max(f64::from(v[2]));
        }
    }

    if min.x > max.x {
        min = DVec3::new(-5.0, -2.0, -8.0);
        max = DVec3::new(5.0, 2.0, 8.0);
    }

    (min, max)
}
