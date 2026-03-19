//! Aerodynamics force models for flight simulation.
//!
//! Airfoil lift/drag, control surface moments, thrust, angular damping,
//! flap effects, wing geometry utilities.
//!
//! Coordinate system: right-handed, Y-up.
//!   +X = right, +Y = up, +Z = forward

use glam::{DMat3, DVec3};
use std::f64::consts::PI;

use crate::atmosphere::{ISAAtmosphere, G};
use crate::types::{AeroSurface, AircraftPhysicsState, ControlState};

// ─────────────────────────────────────────────────────────────
// Wing Geometry Utilities
// ─────────────────────────────────────────────────────────────

/// Compute wing aspect ratio AR = b² / S.
pub fn aspect_ratio(span_m: f64, area_m2: f64) -> f64 {
    if area_m2 <= 0.0 {
        return 0.0;
    }
    span_m * span_m / area_m2
}

/// Compute mean aerodynamic chord c = S / b.
pub fn mean_aerodynamic_chord(area_m2: f64, span_m: f64) -> f64 {
    if span_m <= 0.0 {
        return 0.0;
    }
    area_m2 / span_m
}

/// Estimate Oswald span efficiency factor from aspect ratio.
/// e = 1.78 * (1 - 0.045 * AR^0.68) - 0.64, clamped to [0.5, 0.95].
pub fn estimate_oswald_efficiency(ar: f64) -> f64 {
    if ar <= 0.0 {
        return 0.7;
    }
    let e = 1.78 * (1.0 - 0.045 * ar.powf(0.68)) - 0.64;
    e.clamp(0.5, 0.95)
}

// ─────────────────────────────────────────────────────────────
// Airfoil Model
// ─────────────────────────────────────────────────────────────

/// Airfoil lift/drag coefficient calculator.
pub struct AirfoilModel {
    pub oswald_efficiency: f64,
}

impl Default for AirfoilModel {
    fn default() -> Self {
        Self {
            oswald_efficiency: 0.8,
        }
    }
}

impl AirfoilModel {
    /// Lift coefficient as a function of angle of attack.
    /// Linear pre-stall, smooth sine-based decay post-stall.
    /// Uses `alpha_zero_lift_rad` to shift the lift curve for cambered airfoils.
    pub fn compute_cl(&self, alpha_rad: f64, surface: &AeroSurface) -> f64 {
        let mut alpha = alpha_rad - surface.alpha_zero_lift_rad;
        let mut sign = 1.0;
        if alpha < 0.0 {
            alpha = -alpha;
            sign = -1.0;
        }

        let stall = surface.alpha_stall_rad;
        let cl = if alpha <= stall {
            surface.cl_alpha * alpha
        } else {
            let cl_at_stall = surface.cl_alpha * stall;
            let half_pi = 0.5 * PI;
            let t = ((alpha - stall) / (half_pi - stall)).min(1.0);
            cl_at_stall * (1.0 - 0.8 * (t * half_pi).sin())
        };

        let cl = cl.min(surface.cl_max);
        sign * cl
    }

    /// Drag coefficient using parabolic drag polar: CD = CD0 + CL²/(π·e·AR).
    pub fn compute_cd(&self, cl: f64, surface: &AeroSurface) -> f64 {
        let ar = if surface.area_m2 > 0.0 {
            surface.span_m * surface.span_m / surface.area_m2
        } else {
            10.0
        };
        let induced = (cl * cl) / (PI * self.oswald_efficiency * ar);
        surface.cd0 + induced
    }
}

// ─────────────────────────────────────────────────────────────
// Flap / Airbrake Effects
// ─────────────────────────────────────────────────────────────

/// Additional CL due to flap deployment (0..1).
pub fn flap_cl_bonus(flap_deflection: f64) -> f64 {
    0.4 * flap_deflection.clamp(0.0, 1.0)
}

/// Additional CD due to flap deployment (0..1).
pub fn flap_cd_penalty(flap_deflection: f64) -> f64 {
    0.02 * flap_deflection.clamp(0.0, 1.0)
}

/// Additional CD when airbrake is deployed.
pub fn airbrake_cd_penalty(airbrake_deployed: bool) -> f64 {
    if airbrake_deployed { 0.03 } else { 0.0 }
}

// ─────────────────────────────────────────────────────────────
// Thrust Computation
// ─────────────────────────────────────────────────────────────

/// Compute total thrust force from all engines (world frame).
pub fn compute_thrust(state: &AircraftPhysicsState, throttle: f64) -> DVec3 {
    let throttle = throttle.clamp(0.0, 1.0);
    let mut total = DVec3::ZERO;

    for engine in &state.engines {
        let thrust_mag = engine.max_thrust_n * throttle;
        let thrust_dir_world = state.orientation * engine.direction_local;
        total += thrust_dir_world * thrust_mag;
    }

    // Fallback if no engines defined
    if state.engines.is_empty() {
        let forward_world = state.orientation * DVec3::new(0.0, 0.0, 1.0);
        total = forward_world * 50000.0 * throttle;
    }

    total
}

/// Gravitational force (world frame, Y-down).
pub fn gravity_force(mass_kg: f64) -> DVec3 {
    DVec3::new(0.0, -mass_kg * G, 0.0)
}

// ─────────────────────────────────────────────────────────────
// Control Surface Moments
// ─────────────────────────────────────────────────────────────

/// Compute moments from control surface deflections (body frame).
pub fn compute_control_moments(
    state: &AircraftPhysicsState,
    controls: &ControlState,
    atmosphere: &ISAAtmosphere,
) -> DVec3 {
    let altitude = state.position.y.max(0.0);
    let props = atmosphere.get_properties(altitude);
    let q = 0.5 * props.density_kg_m3 * state.airspeed_ms * state.airspeed_ms;

    // Use largest surface by area for reference geometry (not [0] which may be a small aileron)
    let (total_area, ref_chord, ref_span) = if state.aero_surfaces.is_empty() {
        (30.0, 2.5, 12.0)
    } else {
        let area: f64 = state.aero_surfaces.iter().map(|s| s.area_m2).sum();
        let largest = state.aero_surfaces.iter()
            .max_by(|a, b| a.area_m2.partial_cmp(&b.area_m2).unwrap())
            .unwrap();
        (area, largest.chord_m, largest.span_m)
    };

    // Control moment derivatives (per-radian equivalent)
    // Increased 2.5× from original for responsive feel with hollow-body inertia
    let cm_elevator = -0.025;
    let cl_aileron = 0.020;
    let cn_rudder = -0.012;

    let pitch_moment = q * total_area * ref_chord * cm_elevator * controls.pitch;
    let roll_moment = q * total_area * ref_span * cl_aileron * controls.roll;
    let yaw_moment = q * total_area * ref_span * cn_rudder * controls.yaw;

    // Body-frame torque: X=pitch, Y=yaw, Z=roll
    DVec3::new(pitch_moment, yaw_moment, roll_moment)
}

/// Compute static stability moments (body frame).
///
/// Provides restoring moments that keep the aircraft in stable flight:
/// - Pitch stability around trim AoA (not zero) — allows level flight
/// - Weathervane stability: yaw toward wind when sideslipping
/// - Roll stability: wings-level tendency from dihedral + gravity effect
pub fn compute_stability_moments(
    state: &AircraftPhysicsState,
    atmosphere: &ISAAtmosphere,
) -> DVec3 {
    let altitude = state.position.y.max(0.0);
    let props = atmosphere.get_properties(altitude);
    let q = 0.5 * props.density_kg_m3 * state.airspeed_ms * state.airspeed_ms;

    // Use largest surface by area for reference geometry (not [0] which may be a small aileron)
    let (total_area, ref_chord, ref_span) = if state.aero_surfaces.is_empty() {
        (30.0, 2.5, 12.0)
    } else {
        let area: f64 = state.aero_surfaces.iter().map(|s| s.area_m2).sum();
        let largest = state.aero_surfaces.iter()
            .max_by(|a, b| a.area_m2.partial_cmp(&b.area_m2).unwrap())
            .unwrap();
        (area, largest.chord_m, largest.span_m)
    };

    let alpha = state.angle_of_attack_rad;
    let beta = state.sideslip_rad;

    // Pitch stability around TRIM angle (~3° AoA needed for level flight).
    // In our body frame, negative X torque = nose UP. Standard aero Cm_alpha < 0
    // needs sign flip because our +X rotation = nose DOWN (opposite of aero convention).
    let alpha_trim = 0.05; // ~3° — equilibrium AoA for default wing at cruise
    let cm_alpha = 0.04;   // positive in our convention: nose-up when alpha < trim
    let pitch_stability = q * total_area * ref_chord * cm_alpha * (alpha - alpha_trim);

    // Weathervane yaw stability (nose into wind)
    let cn_beta = 0.05;
    let yaw_stability = q * total_area * ref_span * cn_beta * beta;

    // Roll stability: combined dihedral effect (sideslip-based) + gravity/pendulum
    let cl_beta = -0.05;
    let roll_from_sideslip = q * total_area * ref_span * cl_beta * beta;

    // Direct wings-level tendency (simplified gravity+dihedral coupling).
    let roll_pendulum = -q * total_area * ref_span * 0.01 * state.roll_rad;

    let roll_stability = roll_from_sideslip + roll_pendulum;

    // Body-frame torque: X=pitch, Y=yaw, Z=roll
    DVec3::new(pitch_stability, yaw_stability, roll_stability)
}

// ─────────────────────────────────────────────────────────────
// Angular Damping
// ─────────────────────────────────────────────────────────────

/// Compute angular velocity damping moment (body frame).
pub fn compute_angular_damping(
    angular_velocity: DVec3,
    airspeed_ms: f64,
    altitude_m: f64,
    atmosphere: &ISAAtmosphere,
) -> DVec3 {
    let props = atmosphere.get_properties(altitude_m);
    let q = 0.5 * props.density_kg_m3 * airspeed_ms * airspeed_ms;

    // Increased damping (was 800/600/400 — too weak, caused uncontrolled spin)
    let damping_coeffs = DVec3::new(5000.0, 4000.0, 3000.0);
    -damping_coeffs * q * angular_velocity / (q + 500.0).max(1.0)
}

// ─────────────────────────────────────────────────────────────
// Flight Characteristics Computation
// ─────────────────────────────────────────────────────────────

/// Compute flight characteristics from aircraft geometry and mass properties.
///
/// These parameters characterize the aircraft's flight envelope and are used
/// by the mouse-aim instructor to adapt control gains per airframe.
pub fn compute_flight_characteristics(
    mass_kg: f64,
    aero_surfaces: &[AeroSurface],
    engines: &[crate::types::EngineSpec],
    inertia_tensor: glam::DMat3,
) -> crate::types::FlightCharacteristics {
    let atm = ISAAtmosphere::new();
    let rho_sl = atm.get_properties(0.0).density_kg_m3; // sea-level density

    // Total wing area: sum of all lifting surface areas
    let total_wing_area = if aero_surfaces.is_empty() {
        30.0 // default fallback
    } else {
        aero_surfaces.iter().map(|s| s.area_m2).sum::<f64>()
    };

    // CL_max: area-weighted average across all surfaces
    let cl_max = if aero_surfaces.is_empty() {
        1.5
    } else {
        let weighted_sum: f64 = aero_surfaces.iter()
            .map(|s| s.cl_max * s.area_m2)
            .sum();
        weighted_sum / total_wing_area
    };

    // Max stall AoA: take the maximum alpha_stall_rad from wing surfaces
    let max_aoa_rad = aero_surfaces.iter()
        .map(|s| s.alpha_stall_rad)
        .fold(0.27_f64, f64::max);

    // Stall speed: V_stall = sqrt(2 * W / (rho * S * CL_max))
    let weight = mass_kg * G;
    let stall_speed = if total_wing_area > 0.0 && cl_max > 0.0 && rho_sl > 0.0 {
        (2.0 * weight / (rho_sl * total_wing_area * cl_max)).sqrt()
    } else {
        30.0
    };

    // Reference speed: 1.3 * stall speed (typical cruise margin)
    let ref_speed = 1.3 * stall_speed;

    // Wing loading
    let wing_loading = if total_wing_area > 0.0 {
        mass_kg / total_wing_area
    } else {
        166.0
    };

    // Thrust-to-weight ratio
    let total_thrust: f64 = engines.iter().map(|e| e.max_thrust_n).sum();
    let thrust_to_weight = if weight > 0.0 {
        total_thrust / weight
    } else {
        0.35
    };

    // Control authority: reference control torque / inertia
    // Use sea-level dynamic pressure at ref_speed for the reference moment
    let q_ref = 0.5 * rho_sl * ref_speed * ref_speed;
    let ref_chord = if aero_surfaces.is_empty() {
        2.5
    } else {
        let largest = aero_surfaces.iter()
            .max_by(|a, b| a.area_m2.partial_cmp(&b.area_m2).unwrap())
            .unwrap();
        largest.chord_m
    };
    let ref_span = if aero_surfaces.is_empty() {
        12.0
    } else {
        let largest = aero_surfaces.iter()
            .max_by(|a, b| a.area_m2.partial_cmp(&b.area_m2).unwrap())
            .unwrap();
        largest.span_m
    };

    // Control moment derivative magnitudes
    let cm_elevator = 0.025;
    let cl_aileron = 0.020;
    let cn_rudder = 0.012;

    // Per-axis inertia moments
    let pitch_inertia = inertia_tensor.col(0).x.abs().max(1.0); // Ixx = pitch
    let yaw_inertia = inertia_tensor.col(1).y.abs().max(1.0);   // Iyy = yaw
    let roll_inertia = inertia_tensor.col(2).z.abs().max(1.0);  // Izz = roll

    // Per-axis control authority (P3-1: replaces avg_inertia approach)
    let pitch_torque = q_ref * total_wing_area * ref_chord * cm_elevator;
    let roll_torque = q_ref * total_wing_area * ref_span * cl_aileron;
    let yaw_torque = q_ref * total_wing_area * ref_span * cn_rudder;

    let ca_pitch = pitch_torque / pitch_inertia;
    let ca_roll = roll_torque / roll_inertia;
    let ca_yaw = yaw_torque / yaw_inertia;

    // Average control authority (legacy, kept for backward compatibility)
    let avg_inertia = (pitch_inertia + yaw_inertia + roll_inertia) / 3.0;
    let ref_torque = q_ref * total_wing_area * ref_chord * cm_elevator;
    let control_authority = if avg_inertia > 0.0 {
        ref_torque / avg_inertia
    } else {
        1.0
    };

    let max_pitch_rate = pitch_torque / pitch_inertia;
    let max_roll_rate = roll_torque / roll_inertia;

    crate::types::FlightCharacteristics {
        stall_speed_ms: stall_speed,
        ref_speed_ms: ref_speed,
        max_aoa_rad,
        total_wing_area_m2: total_wing_area,
        control_authority,
        ca_pitch,
        ca_roll,
        ca_yaw,
        wing_loading_kg_m2: wing_loading,
        thrust_to_weight,
        max_pitch_rate_rad_s: max_pitch_rate,
        max_roll_rate_rad_s: max_roll_rate,
    }
}

// ─────────────────────────────────────────────────────────────
// Aerodynamics Calculator
// ─────────────────────────────────────────────────────────────

/// Computes total aerodynamic forces and torques on an aircraft.
#[derive(Default)]
pub struct AerodynamicsCalculator {
    pub atmosphere: ISAAtmosphere,
    pub airfoil: AirfoilModel,
}


impl AerodynamicsCalculator {
    /// Compute total forces (world) and torques (body) on the aircraft.
    ///
    /// When `controls` is provided, includes thrust, control moments, and damping.
    pub fn compute_forces(
        &self,
        state: &mut AircraftPhysicsState,
        surfaces: &[AeroSurface],
        controls: Option<&ControlState>,
        flap_deflection: f64,
        airbrake_deployed: bool,
    ) -> (DVec3, DVec3) {
        let (flap_defl, airbrake_on) = if let Some(ctrl) = controls {
            (ctrl.flaps, ctrl.airbrake)
        } else {
            (flap_deflection, airbrake_deployed)
        };

        let mut total_force_world = DVec3::ZERO;
        let mut total_torque_body = DVec3::ZERO;

        let altitude = state.position.y.max(0.0);
        let props = self.atmosphere.get_properties(altitude);

        let v_world = state.velocity;
        let airspeed = v_world.length();

        state.altitude_m = altitude;
        state.airspeed_ms = airspeed;
        state.mach = if props.speed_of_sound_ms > 0.0 {
            airspeed / props.speed_of_sound_ms
        } else {
            0.0
        };

        let q = 0.5 * props.density_kg_m3 * airspeed * airspeed;

        let r = state.orientation; // body->world
        let r_inv = r.transpose();   // world->body

        if airspeed > 1e-3 {
            let v_body = r_inv * v_world;
            let vbx = v_body.x;
            let vby = v_body.y;
            let vbz = v_body.z;

            let v_xz = (vbx * vbx + vbz * vbz).sqrt();
            state.angle_of_attack_rad = if v_xz > 1e-6 {
                (-vby).atan2(v_xz)
            } else {
                0.0
            };
            state.sideslip_rad = if vbz > 1e-6 {
                vbx.atan2(vbz)
            } else {
                0.0
            };

            for surf in surfaces {
                let (force_w, torque_b) = self.compute_surface(
                    surf, v_body, q, airspeed, r, flap_defl, airbrake_on,
                );
                total_force_world += force_w;
                total_torque_body += torque_b;
            }
        } else {
            state.angle_of_attack_rad = 0.0;
            state.sideslip_rad = 0.0;
        }

        // Gravity
        total_force_world += DVec3::new(0.0, -state.mass_kg * G, 0.0);

        // Controls-integrated forces
        if let Some(ctrl) = controls {
            total_force_world += compute_thrust(state, ctrl.throttle);
            state.throttle = ctrl.throttle;
            total_torque_body +=
                compute_control_moments(state, ctrl, &self.atmosphere);
            total_torque_body += compute_angular_damping(
                state.angular_velocity,
                airspeed,
                altitude,
                &self.atmosphere,
            );
        }

        (total_force_world, total_torque_body)
    }

    /// Compute force (world) and torque (body) for a single surface.
    fn compute_surface(
        &self,
        surf: &AeroSurface,
        v_body: DVec3,
        q: f64,
        airspeed: f64,
        r: DMat3,
        flap_defl: f64,
        airbrake_on: bool,
    ) -> (DVec3, DVec3) {
        let n_len = surf.normal_local.length() + 1e-12;
        let n = surf.normal_local / n_len;

        let v_n = v_body.dot(n);
        let v_tangent = v_body - v_n * n;
        let v_t_mag = v_tangent.length();

        let alpha_local = if v_t_mag > 1e-6 {
            (-v_n).atan2(v_t_mag)
        } else {
            0.0
        };

        let cl = self.airfoil.compute_cl(alpha_local, surf) + flap_cl_bonus(flap_defl);
        let cd = self.airfoil.compute_cd(cl, surf)
            + flap_cd_penalty(flap_defl)
            + airbrake_cd_penalty(airbrake_on);

        let lift_mag = q * surf.area_m2 * cl;
        let drag_mag = q * surf.area_m2 * cd;

        let lift_dir_body = if v_t_mag > 1e-6 {
            let flow_dir_body = v_body / (airspeed + 1e-12);
            let cross = flow_dir_body.cross(n);
            let cross_mag = cross.length();
            if cross_mag > 1e-8 {
                let lift_d = (cross / cross_mag).cross(flow_dir_body);
                let lift_d_len = lift_d.length() + 1e-12;
                lift_d / lift_d_len
            } else {
                n
            }
        } else {
            n
        };

        let drag_dir_body = -v_body / (airspeed + 1e-12);

        let force_body = lift_mag * lift_dir_body + drag_mag * drag_dir_body;
        let force_world = r * force_body;

        // Do NOT use position-based torques: surf.position_local.cross(force_body)
        // The model origin from Blender rarely coincides with the true CG,
        // causing wildly wrong pitching moments. Pitch/roll/yaw stability
        // is handled by coefficient-based control + stability + damping moments.
        let torque_body = DVec3::ZERO;

        (force_world, torque_body)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aspect_ratio() {
        assert!((aspect_ratio(10.0, 25.0) - 4.0).abs() < 1e-10);
        assert!(aspect_ratio(10.0, 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_mean_aerodynamic_chord() {
        assert!((mean_aerodynamic_chord(25.0, 10.0) - 2.5).abs() < 1e-10);
        assert!(mean_aerodynamic_chord(25.0, 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_oswald_efficiency() {
        let e = estimate_oswald_efficiency(8.0);
        assert!((0.5..=0.95).contains(&e));
        assert!((estimate_oswald_efficiency(0.0) - 0.7).abs() < 1e-10);
    }

    #[test]
    fn test_airfoil_cl_linear() {
        let airfoil = AirfoilModel::default();
        let surf = AeroSurface::default();
        let cl = airfoil.compute_cl(0.1, &surf);
        // In linear region: CL = cl_alpha * alpha ≈ 2π * 0.1 ≈ 0.628
        assert!((cl - surf.cl_alpha * 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_airfoil_cl_symmetric() {
        let airfoil = AirfoilModel::default();
        let surf = AeroSurface::default();
        let cl_pos = airfoil.compute_cl(0.1, &surf);
        let cl_neg = airfoil.compute_cl(-0.1, &surf);
        assert!((cl_pos + cl_neg).abs() < 1e-10);
    }

    #[test]
    fn test_airfoil_cd_parabolic() {
        let airfoil = AirfoilModel::default();
        let surf = AeroSurface::default();
        let cd = airfoil.compute_cd(0.0, &surf);
        assert!((cd - surf.cd0).abs() < 1e-10);
    }

    #[test]
    fn test_flap_effects() {
        assert!((flap_cl_bonus(1.0) - 0.4).abs() < 1e-10);
        assert!((flap_cl_bonus(0.0)).abs() < 1e-10);
        assert!((flap_cd_penalty(1.0) - 0.02).abs() < 1e-10);
    }

    #[test]
    fn test_airbrake() {
        assert!((airbrake_cd_penalty(true) - 0.03).abs() < 1e-10);
        assert!(airbrake_cd_penalty(false).abs() < 1e-10);
    }

    #[test]
    fn test_cl_zero_lift_offset() {
        let airfoil = AirfoilModel::default();
        // Cambered airfoil: produces lift at alpha=0
        let surf = AeroSurface {
            alpha_zero_lift_rad: -0.05,
            ..Default::default()
        };
        let cl_at_zero = airfoil.compute_cl(0.0, &surf);
        // Effective alpha = 0 - (-0.05) = 0.05
        assert!((cl_at_zero - surf.cl_alpha * 0.05).abs() < 1e-10);
        // At the zero-lift angle, CL should be zero
        let cl_at_zero_lift = airfoil.compute_cl(-0.05, &surf);
        assert!(cl_at_zero_lift.abs() < 1e-10);
    }

    #[test]
    fn test_cl_default_zero_lift_unchanged() {
        let airfoil = AirfoilModel::default();
        let surf = AeroSurface::default();
        // Default alpha_zero_lift_rad = 0.0 → identical to old behavior
        assert!(surf.alpha_zero_lift_rad.abs() < 1e-10);
        let cl = airfoil.compute_cl(0.1, &surf);
        assert!((cl - surf.cl_alpha * 0.1).abs() < 1e-10);
        let cl_zero = airfoil.compute_cl(0.0, &surf);
        assert!(cl_zero.abs() < 1e-10);
    }

    #[test]
    fn test_gravity_force() {
        let f = gravity_force(5000.0);
        assert!((f.y - (-5000.0 * G)).abs() < 1e-6);
        assert!(f.x.abs() < 1e-10);
        assert!(f.z.abs() < 1e-10);
    }

    #[test]
    fn test_flight_characteristics_biplane() {
        use crate::types::AircraftModel;
        let model = AircraftModel::biplane();
        let fc = compute_flight_characteristics(
            model.mass_kg,
            &model.aero_surfaces,
            &model.engines,
            model.inertia_tensor,
        );
        // An-2 biplane: huge wing area, low stall speed
        assert!(fc.stall_speed_ms > 10.0 && fc.stall_speed_ms < 30.0,
            "Biplane stall speed should be 10-30 m/s, got {:.1}", fc.stall_speed_ms);
        assert!(fc.ref_speed_ms > fc.stall_speed_ms,
            "Ref speed should exceed stall speed");
        assert!(fc.max_aoa_rad > 0.30,
            "Biplane max AoA should be > 0.30 rad, got {:.3}", fc.max_aoa_rad);
        assert!(fc.total_wing_area_m2 > 80.0,
            "Biplane total wing area should be > 80 m2, got {:.1}", fc.total_wing_area_m2);
        assert!(fc.control_authority > 0.0,
            "Control authority should be positive");
        assert!(fc.wing_loading_kg_m2 < 100.0,
            "Biplane wing loading should be low, got {:.1}", fc.wing_loading_kg_m2);
        assert!(fc.thrust_to_weight > 0.1 && fc.thrust_to_weight < 1.0,
            "Biplane T/W should be 0.1-1.0, got {:.3}", fc.thrust_to_weight);
        eprintln!("[test] biplane flight_chars: stall={:.1}m/s ref={:.1}m/s max_aoa={:.1}deg",
            fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad.to_degrees());
        eprintln!("[test]   wing_area={:.1}m2 ctrl_auth={:.3} wing_load={:.1}kg/m2 T/W={:.3}",
            fc.total_wing_area_m2, fc.control_authority, fc.wing_loading_kg_m2, fc.thrust_to_weight);
    }

    #[test]
    fn test_flight_characteristics_defaults() {
        // Test with default/empty surfaces
        let fc = compute_flight_characteristics(
            5000.0,
            &[],
            &[],
            DMat3::from_diagonal(DVec3::new(5000.0, 8000.0, 3000.0)),
        );
        assert!(fc.stall_speed_ms > 0.0, "Stall speed should be positive");
        assert!(fc.ref_speed_ms > fc.stall_speed_ms, "Ref > stall");
        assert!(fc.thrust_to_weight == 0.0, "No engines = 0 T/W");
    }
}
