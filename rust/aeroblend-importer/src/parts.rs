//! Aircraft part classification from mesh names.
//!
//! Maps object names to aircraft component categories using regex patterns,
//! then derives `AeroSurface` and `EngineSpec` physics metadata from geometry.

use glam::DVec3;
use regex::Regex;
use std::sync::LazyLock;

use aeroblend_physics::types::*;

/// Part category enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartCategory {
    Wing,
    WingLeft,
    WingRight,
    Fuselage,
    TailHorizontal,
    TailVertical,
    Tail,
    Engine,
    Propeller,
    Rotor,
    LandingGear,
    Canopy,
    ControlSurface,
    Unknown,
}

/// Classification pattern entry.
struct ClassPattern {
    regex: Regex,
    kind: &'static str,
}

static PATTERNS: LazyLock<Vec<ClassPattern>> = LazyLock::new(|| {
    vec![
        ClassPattern { regex: Regex::new(r"(?i)wing[_.\s-]?l(?:eft)?").unwrap(), kind: "wing_left" },
        ClassPattern { regex: Regex::new(r"(?i)wing[_.\s-]?r(?:ight)?").unwrap(), kind: "wing_right" },
        ClassPattern { regex: Regex::new(r"(?i)v(?:ertical)?[_.\s-]?(?:tail|stab)|rudder|(?:v|vert)[_.\s-]?fin").unwrap(), kind: "v_tail" },
        ClassPattern { regex: Regex::new(r"(?i)h(?:orizontal)?[_.\s-]?(?:tail|stab)|stabiliz").unwrap(), kind: "h_tail" },
        ClassPattern { regex: Regex::new(r"(?i)aileron|elevator|flap|slat|spoiler").unwrap(), kind: "control_surface" },
        ClassPattern { regex: Regex::new(r"(?i)wing").unwrap(), kind: "wing" },
        ClassPattern { regex: Regex::new(r"(?i)rotor|main_rotor|tail_rotor").unwrap(), kind: "rotor" },
        ClassPattern { regex: Regex::new(r"(?i)tail|empennage").unwrap(), kind: "tail" },
        ClassPattern { regex: Regex::new(r"(?i)prop(?:eller)?|blade").unwrap(), kind: "propeller" },
        ClassPattern { regex: Regex::new(r"(?i)engine|motor|nacelle|intake|exhaust|jet|turbine").unwrap(), kind: "engine" },
        ClassPattern { regex: Regex::new(r"(?i)fuselage|body|hull|fuse").unwrap(), kind: "fuselage" },
        ClassPattern { regex: Regex::new(r"(?i)gear|wheel|strut|landing").unwrap(), kind: "gear" },
        ClassPattern { regex: Regex::new(r"(?i)canopy|cockpit|glass|windshield").unwrap(), kind: "canopy" },
    ]
});

const AERO_SURFACE_KINDS: &[&str] = &[
    "wing", "wing_left", "wing_right", "h_tail", "v_tail", "tail", "control_surface",
];

const ENGINE_KINDS: &[&str] = &["engine", "propeller", "rotor"];

/// Classify a mesh name into a part kind string.
pub fn classify(name: &str) -> &'static str {
    for pat in PATTERNS.iter() {
        if pat.regex.is_match(name) {
            return pat.kind;
        }
    }
    "unknown"
}

/// A classified aircraft part with spatial metadata.
#[derive(Debug, Clone)]
pub struct PartInfo {
    pub name: String,
    pub kind: &'static str,
    pub mesh_index: usize,
    pub center: DVec3,
    pub extent: DVec3,
}

/// Compute bounding box center and half-extent for a mesh.
pub fn center_and_extent(mesh: &MeshData) -> (DVec3, DVec3) {
    if mesh.vertices.is_empty() {
        return (DVec3::ZERO, DVec3::ZERO);
    }

    let mut min = DVec3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = DVec3::new(f64::MIN, f64::MIN, f64::MIN);

    for v in &mesh.vertices {
        // JSF-Rust SR-501: Use From/Into for type conversion instead of `as` cast
        min.x = min.x.min(f64::from(v[0]));
        min.y = min.y.min(f64::from(v[1]));
        min.z = min.z.min(f64::from(v[2]));
        max.x = max.x.max(f64::from(v[0]));
        max.y = max.y.max(f64::from(v[1]));
        max.z = max.z.max(f64::from(v[2]));
    }

    ((min + max) / 2.0, max - min)
}

/// Classify all meshes and attach bounding box metadata.
pub fn classify_meshes(meshes: &[MeshData]) -> Vec<PartInfo> {
    meshes
        .iter()
        .enumerate()
        .map(|(i, mesh)| {
            let kind = classify(&mesh.name);
            let (center, extent) = center_and_extent(mesh);
            PartInfo {
                name: mesh.name.clone(),
                kind,
                mesh_index: i,
                center,
                extent,
            }
        })
        .collect()
}

/// Create an `AeroSurface` from mesh geometry and part kind.
///
/// Attempts to extract the airfoil cross-section from the mesh to derive
/// geometry-based aerodynamic coefficients. Falls back to hardcoded defaults
/// if the mesh is too coarse or non-airfoil-shaped.
pub fn make_aero_surface(mesh: &MeshData, kind: &str) -> AeroSurface {
    let (center, extent) = center_and_extent(mesh);
    // For vertical surfaces (v_tail), span is in Y (height) and chord is in Z.
    // For horizontal surfaces (wings, h_tail), span is in X and chord is in Z.
    let (span, chord) = if kind == "v_tail" {
        (extent.y.max(0.1), extent.z.max(0.1))
    } else {
        (extent.x.max(0.1), extent.z.max(0.1))
    };
    let area = span * chord;

    let is_tail = kind == "h_tail" || kind == "v_tail" || kind == "tail";
    let normal = if kind == "v_tail" {
        DVec3::new(1.0, 0.0, 0.0)
    } else {
        DVec3::new(0.0, 1.0, 0.0)
    };

    // Try airfoil cross-section extraction
    let (cl_alpha, cd0, cl_max, alpha_stall_rad, alpha_zero_lift_rad) =
        if let Some(profile) = crate::airfoil::extract_airfoil_profile(mesh) {
            let coeffs = crate::airfoil::derive_aero_coefficients(&profile);
            let tail_factor = if is_tail { 0.6 } else { 1.0 };
            (
                coeffs.cl_alpha * tail_factor,
                coeffs.cd0,
                coeffs.cl_max,
                coeffs.alpha_stall_rad,
                coeffs.alpha_zero_lift_rad,
            )
        } else {
            // Fallback: hardcoded defaults.
            // Non-tail surfaces get alpha_zero_lift = -0.035 (~-2°) to model
            // cambered airfoils, ensuring positive lift at zero AoA.
            // Tail surfaces use symmetric airfoil (zero-lift at 0°).
            (
                2.0 * std::f64::consts::PI * if is_tail { 0.6 } else { 1.0 },
                if is_tail { 0.015 } else { 0.02 },
                if is_tail { 1.2 } else { 1.5 },
                if is_tail { 0.25 } else { 0.27 },
                if is_tail { 0.0 } else { -0.035 },
            )
        };

    AeroSurface {
        name: mesh.name.clone(),
        area_m2: area,
        span_m: span,
        chord_m: chord,
        position_local: center,
        normal_local: normal,
        cl_alpha,
        cd0,
        cl_max,
        alpha_stall_rad,
        alpha_zero_lift_rad,
    }
}

/// Create an `EngineSpec` from mesh geometry and part kind.
pub fn make_engine_spec(mesh: &MeshData, kind: &str) -> EngineSpec {
    let (center, _) = center_and_extent(mesh);

    let (engine_type, thrust) = match kind {
        "rotor" => (EngineType::Rotor, 30000.0),
        "propeller" => (EngineType::Propeller, 20000.0),
        _ => (EngineType::Jet, 50000.0),
    };

    // Engine thrust always points forward. In Blender coordinates, forward is -Z.
    // The lib.rs coordinate conversion negates Z, so (0,0,-1) → (0,0,1) = forward
    // in physics space.
    // NOTE: Average mesh normals on cylindrical nacelles give radial outward
    // direction (sideways), which is WRONG for thrust direction.
    let direction = DVec3::new(0.0, 0.0, -1.0);

    EngineSpec {
        engine_type,
        max_thrust_n: thrust,
        position_local: center,
        direction_local: direction,
    }
}

/// Extract `AeroSurface` objects from classified parts.
pub fn extract_aero_surfaces(_parts: &[PartInfo]) -> Vec<AeroSurface> {
    // Note: we need the meshes to create surfaces. This is a simplified version
    // that creates default surfaces from part info.
    // In the full pipeline, caller provides meshes.
    Vec::new()
}

/// Extract `AeroSurface` objects from classified parts with mesh data.
pub fn extract_aero_surfaces_with_meshes(parts: &[PartInfo], meshes: &[MeshData]) -> Vec<AeroSurface> {
    parts
        .iter()
        .filter(|p| AERO_SURFACE_KINDS.contains(&p.kind))
        .map(|p| make_aero_surface(&meshes[p.mesh_index], p.kind))
        .collect()
}

/// Extract `EngineSpec` objects from classified parts.
pub fn extract_engines(_parts: &[PartInfo]) -> Vec<EngineSpec> {
    Vec::new()
}

/// Extract `EngineSpec` objects from classified parts with mesh data.
pub fn extract_engines_with_meshes(parts: &[PartInfo], meshes: &[MeshData]) -> Vec<EngineSpec> {
    parts
        .iter()
        .filter(|p| ENGINE_KINDS.contains(&p.kind))
        .map(|p| make_engine_spec(&meshes[p.mesh_index], p.kind))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_classify_wing() {
        assert_eq!(classify("Wing_Left"), "wing_left");
        assert_eq!(classify("wing_right"), "wing_right");
        assert_eq!(classify("MainWing"), "wing");
    }

    #[test]
    fn test_classify_tail() {
        assert_eq!(classify("H_Tail"), "h_tail");
        assert_eq!(classify("Vertical_Stabilizer"), "v_tail");
        assert_eq!(classify("rudder_01"), "v_tail");
    }

    #[test]
    fn test_classify_engine() {
        assert_eq!(classify("Jet_Engine_L"), "engine");
        assert_eq!(classify("Propeller_Main"), "propeller");
    }

    #[test]
    fn test_classify_fuselage() {
        assert_eq!(classify("Fuselage_Main"), "fuselage");
        assert_eq!(classify("Body"), "fuselage");
    }

    #[test]
    fn test_classify_unknown() {
        assert_eq!(classify("random_object"), "unknown");
    }

    #[test]
    fn test_classify_biplane_parts() {
        assert_eq!(classify("Upper_Wing_Left"), "wing_left");
        assert_eq!(classify("Upper_Wing_Right"), "wing_right");
        assert_eq!(classify("Lower_Wing_Left"), "wing_left");
        assert_eq!(classify("Lower_Wing_Right"), "wing_right");
        assert_eq!(classify("Propeller"), "propeller");
        assert_eq!(classify("Strut_Left_0"), "gear");
        assert_eq!(classify("Strut_Right_1"), "gear");
    }

    #[test]
    fn test_classify_control_surfaces() {
        assert_eq!(classify("Aileron_Left"), "control_surface");
        assert_eq!(classify("Elevator"), "control_surface");
        assert_eq!(classify("Flap_Right"), "control_surface");
    }

    #[test]
    fn test_center_and_extent_empty() {
        let mesh = MeshData::default();
        let (c, e) = center_and_extent(&mesh);
        assert_eq!(c, DVec3::ZERO);
        assert_eq!(e, DVec3::ZERO);
    }

    #[test]
    fn test_center_and_extent() {
        let mesh = MeshData {
            vertices: vec![[-1.0, -2.0, -3.0], [1.0, 2.0, 3.0]],
            ..Default::default()
        };
        let (c, e) = center_and_extent(&mesh);
        assert!((c.x).abs() < 1e-10);
        assert!((c.y).abs() < 1e-10);
        assert!((c.z).abs() < 1e-10);
        assert!((e.x - 2.0).abs() < 1e-10);
        assert!((e.y - 4.0).abs() < 1e-10);
        assert!((e.z - 6.0).abs() < 1e-10);
    }
}
