//! Physics engine core implementing 6DoF flight dynamics.
//!
//! Uses quaternion-based orientation, RK4 integration, force/torque
//! accumulation, and ground collision detection.

use glam::{DMat3, DVec3};

use aeroblend_math::{quat_derivative, quat_to_rotation_matrix};
use crate::aerodynamics::*;
use crate::atmosphere::G;
use crate::integrator::*;
use crate::types::*;

/// Ground level in metres.
const GROUND_LEVEL: f64 = 0.0;
/// Ground collision spring constant (N/m).
const GROUND_SPRING_K: f64 = 5000.0;
/// Ground collision damping (Ns/m).
const GROUND_DAMPING: f64 = 500.0;
/// Brake deceleration coefficient (per second).
const BRAKE_COEFF: f64 = 8.0;

/// Force accumulator for the physics engine.
struct ForceAccumulator {
    total_force: DVec3,
    total_torque: DVec3,
}

impl ForceAccumulator {
    fn new() -> Self {
        Self {
            total_force: DVec3::ZERO,
            total_torque: DVec3::ZERO,
        }
    }

    // Note: reset() method currently unused but may be needed for future force reset logic
    #[allow(dead_code)]
    fn reset(&mut self) {
        self.total_force = DVec3::ZERO;
        self.total_torque = DVec3::ZERO;
    }

    fn add_force_world(&mut self, force: DVec3) {
        self.total_force += force;
    }

    fn add_torque_body(&mut self, torque: DVec3) {
        self.total_torque += torque;
    }
}

/// Main physics engine implementing 6DoF flight dynamics.
pub struct AeroBlendPhysicsEngine {
    state: Option<PhysicsState6DoF>,
    aero_calc: AerodynamicsCalculator,
    sim_time: f64,
    controls: ControlState,
    last_total_force: DVec3,
    flight_chars: FlightCharacteristics,
}

impl AeroBlendPhysicsEngine {
    pub fn new() -> Self {
        Self {
            state: None,
            aero_calc: AerodynamicsCalculator::default(),
            sim_time: 0.0,
            controls: ControlState::default(),
            last_total_force: DVec3::ZERO,
            flight_chars: FlightCharacteristics::default(),
        }
    }

    /// Create initial physics state from an aircraft model.
    pub fn init(&mut self, aircraft: &AircraftModel) -> AircraftPhysicsState {
        let inertia = aircraft.inertia_tensor;

        // Compute flight characteristics from geometry
        let fc = compute_flight_characteristics(
            aircraft.mass_kg,
            &aircraft.aero_surfaces,
            &aircraft.engines,
            inertia,
        );
        self.flight_chars = fc;

        // Initial cruise speed: at least 1.5x stall or old defaults, whichever is higher
        let cruise_speed = if aircraft.engines.iter().any(|e| e.engine_type == EngineType::Propeller) {
            fc.ref_speed_ms.max(50.0) // prop planes: at least 50 m/s
        } else {
            fc.ref_speed_ms.max(80.0) // jets: at least 80 m/s
        };
        let s = PhysicsState6DoF {
            mass_kg: aircraft.mass_kg,
            inertia_tensor: inertia,
            inv_inertia_tensor: inertia.inverse(),
            aero_surfaces: aircraft.aero_surfaces.clone(),
            engines: aircraft.engines.clone(),
            position: DVec3::new(0.0, 300.0, 0.0),
            velocity: DVec3::new(0.0, 0.0, cruise_speed),
            throttle: 0.6,
            ..Default::default()
        };

        self.state = Some(s.clone());
        self.sim_time = 0.0;
        let mut interface_state = s.to_interface_state();
        interface_state.flight_chars = fc;
        interface_state
    }

    /// Get the pre-computed flight characteristics for the current aircraft.
    pub fn get_flight_characteristics(&self) -> FlightCharacteristics {
        self.flight_chars
    }

    /// Advance physics by dt seconds using RK4 integration.
    pub fn step(
        &mut self,
        state: &AircraftPhysicsState,
        controls: &ControlState,
        dt: f64,
    ) -> AircraftPhysicsState {
        let s = {
            let mut tmp = PhysicsState6DoF::from_interface_state(state);
            tmp.throttle = controls.throttle;
            tmp
        };
        self.controls = controls.clone();

        let ctrl = self.controls.clone();
        let sim_time = self.sim_time;

        let new_s = {
            let aero_calc = &self.aero_calc;
            integrate_rk4(&s, sim_time, dt, &|st, _t| {
                compute_derivative(st, &ctrl, aero_calc)
            })
        };

        let new_s = handle_ground_collision(new_s, &self.controls, dt);

        self.sim_time += dt;

        // Compute the final forces for acceleration/g-force output
        let mut interface_state = new_s.to_interface_state();
        let surfaces = if new_s.aero_surfaces.is_empty() {
            vec![AeroSurface {
                name: "default_wing".to_string(),
                area_m2: 30.0,
                span_m: 12.0,
                ..Default::default()
            }]
        } else {
            new_s.aero_surfaces.clone()
        };

        let (force_world, _) = self.aero_calc.compute_forces(
            &mut interface_state,
            &surfaces,
            Some(&ctrl),
            ctrl.flaps,
            ctrl.airbrake,
        );
        self.last_total_force = force_world;

        let accel = self.last_total_force / new_s.mass_kg;
        interface_state.acceleration = accel;
        // P2-5: Signed G-force along body up axis
        // F_aero = total_force - gravity; body_up = orientation column 1
        let f_aero = self.last_total_force - DVec3::new(0.0, -new_s.mass_kg * G, 0.0);
        let body_up = quat_to_rotation_matrix(new_s.orientation_q).col(1);
        interface_state.g_force = f_aero.dot(body_up) / (new_s.mass_kg * G);

        interface_state.flight_chars = self.flight_chars;

        self.state = Some(new_s);
        interface_state
    }

    /// Get the most recent physics state.
    pub fn get_state(&self) -> AircraftPhysicsState {
        if let Some(ref s) = self.state {
            let mut st = s.to_interface_state();
            st.flight_chars = self.flight_chars;
            st
        } else {
            AircraftPhysicsState::default()
        }
    }

    /// Return (position, `rotation_matrix`) for the renderer.
    pub fn get_render_transform(&self) -> (DVec3, DMat3) {
        if let Some(ref s) = self.state {
            (s.position, quat_to_rotation_matrix(s.orientation_q))
        } else {
            (DVec3::ZERO, DMat3::IDENTITY)
        }
    }
}

impl Default for AeroBlendPhysicsEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute time derivative of state.
fn compute_derivative(
    state: &PhysicsState6DoF,
    controls: &ControlState,
    aero_calc: &AerodynamicsCalculator,
) -> StateDerivative {
    let mut accum = ForceAccumulator::new();
    let q = state.orientation_q;

    let mut interface_state = state.to_interface_state();

    let surfaces = if state.aero_surfaces.is_empty() {
        vec![AeroSurface {
            name: "default_wing".to_string(),
            area_m2: 30.0,
            span_m: 12.0,
            ..Default::default()
        }]
    } else {
        state.aero_surfaces.clone()
    };

    // 1. Aerodynamic forces + gravity
    let (aero_force_world, aero_torque_body) = aero_calc.compute_forces(
        &mut interface_state,
        &surfaces,
        None,
        controls.flaps,
        controls.airbrake,
    );
    accum.add_force_world(aero_force_world);
    accum.add_torque_body(aero_torque_body);

    // 2. Thrust
    let thrust_world = compute_thrust(&interface_state, controls.throttle);
    accum.add_force_world(thrust_world);

    // NOTE: Engine position-based torques (position.cross(thrust)) are disabled
    // because the model origin from Blender doesn't coincide with the true CG.
    // Torque effects from asymmetric thrust (engine out) would need explicit CG
    // calculation to work correctly.

    // 3. Control surface torques
    let control_moments =
        compute_control_moments(&interface_state, controls, &aero_calc.atmosphere);
    accum.add_torque_body(control_moments);

    // 4. Static stability (restoring moments from AoA and sideslip)
    let stability_moments =
        compute_stability_moments(&interface_state, &aero_calc.atmosphere);
    accum.add_torque_body(stability_moments);

    // 5. Angular damping
    let damping_moments = compute_angular_damping(
        state.angular_velocity,
        interface_state.airspeed_ms,
        interface_state.altitude_m,
        &aero_calc.atmosphere,
    );
    accum.add_torque_body(damping_moments);

    // Translational: a = F / m
    let accel_world = accum.total_force / state.mass_kg;

    // Rotational: Euler's equation
    let omega = state.angular_velocity;
    let i_omega = state.inertia_tensor * omega;
    let gyroscopic = omega.cross(i_omega);
    let alpha_body = state.inv_inertia_tensor * (accum.total_torque - gyroscopic);

    let dq = quat_derivative(q, omega);

    StateDerivative {
        d_position: state.velocity,
        d_velocity: accel_world,
        d_orientation_q: dq,
        d_angular_velocity: alpha_body,
    }
}

/// Ground collision handler.
fn handle_ground_collision(
    mut state: PhysicsState6DoF,
    controls: &ControlState,
    dt: f64,
) -> PhysicsState6DoF {
    if state.position.y < GROUND_LEVEL {
        let penetration = GROUND_LEVEL - state.position.y;
        let vy = state.velocity.y;

        let mut normal_force = GROUND_SPRING_K * penetration - GROUND_DAMPING * vy.min(0.0);
        normal_force = normal_force.max(0.0);

        state.position.y = GROUND_LEVEL;

        if vy < 0.0 {
            state.velocity.y = -vy * 0.1;
        }

        // Ground friction
        let friction_coeff = 0.3;
        let h_speed = (state.velocity.x * state.velocity.x
            + state.velocity.z * state.velocity.z)
            .sqrt();
        if h_speed > 0.1 {
            let friction_force = friction_coeff * normal_force;
            let decel = (friction_force / state.mass_kg).min(h_speed);
            let friction_dir_x = state.velocity.x / h_speed;
            let friction_dir_z = state.velocity.z / h_speed;
            state.velocity.x -= friction_dir_x * decel;
            state.velocity.z -= friction_dir_z * decel;
        }

        // Wheel brake deceleration
        let brake_input = controls.brake.clamp(0.0, 1.0);
        if brake_input > 0.0 {
            let brake_factor = (1.0 - brake_input * BRAKE_COEFF * dt).max(0.0);
            state.velocity.x *= brake_factor;
            state.velocity.z *= brake_factor;
        }

        // Damp angular velocity on ground
        state.angular_velocity *= 0.95;
    }

    state
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─────────────────────────────────────────────────────────────
    // Existing tests (fixed for computed cruise speed)
    // ─────────────────────────────────────────────────────────────

    #[test]
    fn test_engine_init() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::default();
        let state = engine.init(&model);
        assert!((state.altitude_m - 300.0).abs() < 1.0);
        // Biplane ref_speed is ~23 m/s (low stall speed from huge wing area + high CL_max)
        assert!(state.airspeed_ms > 15.0,
            "Init speed should be above stall, got {:.1}", state.airspeed_ms);
    }

    #[test]
    fn test_engine_step_no_crash() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::default();
        let state = engine.init(&model);
        let controls = ControlState {
            throttle: 0.5,
            ..Default::default()
        };
        let new_state = engine.step(&state, &controls, 1.0 / 120.0);
        assert!(new_state.altitude_m > 0.0);
    }

    #[test]
    fn test_ground_collision() {
        let mut state = PhysicsState6DoF::default();
        state.position.y = -1.0;
        state.velocity.y = -10.0;
        let controls = ControlState::default();
        let result = handle_ground_collision(state, &controls, 1.0 / 120.0);
        assert!(result.position.y >= GROUND_LEVEL);
        assert!(result.velocity.y >= 0.0);
    }

    #[test]
    fn test_aircraft_moves_over_time() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::default();
        let init_state = engine.init(&model);
        let controls = ControlState {
            throttle: 0.5,
            ..Default::default()
        };

        let init_pos_z = init_state.position.z;
        eprintln!("Init: alt={:.1} spd={:.1}", init_state.altitude_m, init_state.airspeed_ms);

        let dt = 1.0 / 120.0;
        let mut state = init_state;
        for _ in 0..240 {
            state = engine.step(&state, &controls, dt);
        }

        let dz = state.position.z - init_pos_z;
        eprintln!("After 2s: dz={:.1}m spd={:.1}m/s", dz, state.airspeed_ms);
        assert!(dz.abs() > 20.0, "Aircraft should move forward, got {:.1}m", dz);
        assert!(state.airspeed_ms > 10.0, "Should still be flying, got {:.1} m/s", state.airspeed_ms);
    }

    #[test]
    fn test_controls_and_lift_diagnostics() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::default();
        let init_state = engine.init(&model);

        let dt = 1.0 / 120.0;

        // Phase 1: Cruise 2 seconds
        let mut state = init_state;
        let cruise_ctrl = ControlState { throttle: 0.5, ..Default::default() };
        for _ in 0..240 {
            state = engine.step(&state, &cruise_ctrl, dt);
        }
        let alt_after_cruise = state.altitude_m;

        // Phase 2: Pitch up 2 seconds
        let w_ctrl = ControlState { throttle: 0.5, pitch: 1.0, ..Default::default() };
        for _ in 0..240 {
            state = engine.step(&state, &w_ctrl, dt);
        }

        // Phase 3: Roll right 1 second
        let d_ctrl = ControlState { throttle: 0.5, roll: 1.0, ..Default::default() };
        for _ in 0..120 {
            state = engine.step(&state, &d_ctrl, dt);
        }

        // Biplane at ~24 m/s loses some altitude due to high drag at low speed
        assert!(alt_after_cruise > 270.0,
            "Should roughly maintain altitude, got {:.1}", alt_after_cruise);
        assert!(state.roll_rad.abs() > 0.1,
            "Roll should respond, got {:.3}", state.roll_rad);
    }

    // ─────────────────────────────────────────────────────────────
    // Multi-profile tests: init and FlightCharacteristics
    // ─────────────────────────────────────────────────────────────

    #[test]
    fn test_init_biplane_flight_chars() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::biplane();
        let state = engine.init(&model);
        let fc = engine.get_flight_characteristics();

        eprintln!("Biplane FC: stall={:.1}m/s ref={:.1}m/s max_aoa={:.2}rad area={:.1}m² T/W={:.2} wing_load={:.1}",
            fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad,
            fc.total_wing_area_m2, fc.thrust_to_weight, fc.wing_loading_kg_m2);

        // Biplane: massive wing area → very low stall speed
        assert!(fc.stall_speed_ms < 25.0,
            "Biplane stall should be very low, got {:.1}", fc.stall_speed_ms);
        assert!(fc.total_wing_area_m2 > 70.0,
            "Biplane should have >70m² wing area, got {:.1}", fc.total_wing_area_m2);
        assert!(fc.max_aoa_rad > 0.35,
            "Biplane stall AoA should be high, got {:.2}", fc.max_aoa_rad);
        assert!(fc.wing_loading_kg_m2 < 70.0,
            "Biplane wing loading should be low, got {:.1}", fc.wing_loading_kg_m2);
        assert!(state.airspeed_ms > fc.stall_speed_ms,
            "Init speed {:.1} should be above stall {:.1}", state.airspeed_ms, fc.stall_speed_ms);
    }

    #[test]
    fn test_init_monoplane_flight_chars() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::monoplane();
        let state = engine.init(&model);
        let fc = engine.get_flight_characteristics();

        eprintln!("Monoplane FC: stall={:.1}m/s ref={:.1}m/s max_aoa={:.2}rad area={:.1}m² T/W={:.2} wing_load={:.1}",
            fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad,
            fc.total_wing_area_m2, fc.thrust_to_weight, fc.wing_loading_kg_m2);

        // Monoplane (Cessna 172): moderate wing loading
        assert!(fc.stall_speed_ms > 20.0 && fc.stall_speed_ms < 40.0,
            "Monoplane stall should be 20-40 m/s, got {:.1}", fc.stall_speed_ms);
        assert!(fc.total_wing_area_m2 > 15.0 && fc.total_wing_area_m2 < 25.0,
            "Monoplane wing area 15-25m², got {:.1}", fc.total_wing_area_m2);
        assert!(fc.wing_loading_kg_m2 > 40.0 && fc.wing_loading_kg_m2 < 80.0,
            "Monoplane wing loading 40-80, got {:.1}", fc.wing_loading_kg_m2);
        assert!(state.airspeed_ms > fc.stall_speed_ms,
            "Init speed {:.1} should be above stall {:.1}", state.airspeed_ms, fc.stall_speed_ms);
    }

    #[test]
    fn test_init_jet_flight_chars() {
        let mut engine = AeroBlendPhysicsEngine::new();
        let model = AircraftModel::jet();
        let state = engine.init(&model);
        let fc = engine.get_flight_characteristics();

        eprintln!("Jet FC: stall={:.1}m/s ref={:.1}m/s max_aoa={:.2}rad area={:.1}m² T/W={:.2} wing_load={:.1}",
            fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad,
            fc.total_wing_area_m2, fc.thrust_to_weight, fc.wing_loading_kg_m2);

        // Jet (F-16): high wing loading, high T/W
        assert!(fc.stall_speed_ms > 35.0,
            "Jet stall should be >35 m/s, got {:.1}", fc.stall_speed_ms);
        assert!(fc.thrust_to_weight > 0.5,
            "Jet T/W should be >0.5, got {:.2}", fc.thrust_to_weight);
        assert!(fc.wing_loading_kg_m2 > 200.0,
            "Jet wing loading should be >200, got {:.1}", fc.wing_loading_kg_m2);
        // Jet init should be at least 60 m/s (enforced by engine.init)
        assert!(state.airspeed_ms >= 60.0,
            "Jet init speed should be >=60, got {:.1}", state.airspeed_ms);
    }

    // ─────────────────────────────────────────────────────────────
    // Multi-profile: level flight stability
    // ─────────────────────────────────────────────────────────────

    /// Helper: simulate level flight for `duration` seconds and return final state.
    fn simulate_level_flight(model: &AircraftModel, throttle: f64, duration_sec: f64) -> AircraftPhysicsState {
        let mut engine = AeroBlendPhysicsEngine::new();
        let init_state = engine.init(model);
        let controls = ControlState { throttle, ..Default::default() };
        let dt = 1.0 / 120.0;
        let steps = (duration_sec / dt) as usize;
        let mut state = init_state;
        for _ in 0..steps {
            state = engine.step(&state, &controls, dt);
        }
        state
    }

    #[test]
    fn test_biplane_level_flight() {
        let model = AircraftModel::biplane();
        let state = simulate_level_flight(&model, 0.6, 5.0);
        eprintln!("Biplane 5s cruise: alt={:.1} spd={:.1} climb={:.1} aoa={:.3}",
            state.altitude_m, state.airspeed_ms, state.climb_rate_ms, state.angle_of_attack_rad);
        // Should not crash or lose too much altitude in 5 seconds
        assert!(state.altitude_m > 200.0,
            "Biplane should stay airborne, alt={:.1}", state.altitude_m);
        assert!(state.airspeed_ms > 10.0,
            "Biplane should maintain speed, got {:.1}", state.airspeed_ms);
    }

    #[test]
    fn test_monoplane_level_flight() {
        let model = AircraftModel::monoplane();
        let state = simulate_level_flight(&model, 0.6, 5.0);
        eprintln!("Monoplane 5s cruise: alt={:.1} spd={:.1} climb={:.1} aoa={:.3}",
            state.altitude_m, state.airspeed_ms, state.climb_rate_ms, state.angle_of_attack_rad);
        assert!(state.altitude_m > 200.0,
            "Monoplane should stay airborne, alt={:.1}", state.altitude_m);
        assert!(state.airspeed_ms > 15.0,
            "Monoplane should maintain speed, got {:.1}", state.airspeed_ms);
    }

    #[test]
    fn test_jet_level_flight() {
        let model = AircraftModel::jet();
        let state = simulate_level_flight(&model, 0.5, 5.0);
        eprintln!("Jet 5s cruise: alt={:.1} spd={:.1} climb={:.1} aoa={:.3}",
            state.altitude_m, state.airspeed_ms, state.climb_rate_ms, state.angle_of_attack_rad);
        assert!(state.altitude_m > 200.0,
            "Jet should stay airborne, alt={:.1}", state.altitude_m);
        assert!(state.airspeed_ms > 40.0,
            "Jet should maintain speed, got {:.1}", state.airspeed_ms);
    }

    // ─────────────────────────────────────────────────────────────
    // Multi-profile: pitch up should not cause unrecoverable stall
    // ─────────────────────────────────────────────────────────────

    /// Helper: cruise, then pitch up, then release and check recovery.
    fn test_pitch_up_recovery(model: &AircraftModel, label: &str) {
        let mut engine = AeroBlendPhysicsEngine::new();
        let init_state = engine.init(model);
        let fc = engine.get_flight_characteristics();
        let dt = 1.0 / 120.0;

        // Phase 1: Cruise 2 seconds at 70% throttle
        let cruise_ctrl = ControlState { throttle: 0.7, ..Default::default() };
        let mut state = init_state;
        for _ in 0..240 {
            state = engine.step(&state, &cruise_ctrl, dt);
        }
        let alt_before = state.altitude_m;
        let spd_before = state.airspeed_ms;
        eprintln!("[{}] After cruise: alt={:.1} spd={:.1}", label, alt_before, spd_before);

        // Phase 2: Full pitch up for 3 seconds
        let up_ctrl = ControlState { throttle: 0.7, pitch: 1.0, ..Default::default() };
        let mut max_aoa_seen = 0.0_f64;
        for _ in 0..360 {
            state = engine.step(&state, &up_ctrl, dt);
            max_aoa_seen = max_aoa_seen.max(state.angle_of_attack_rad);
        }
        eprintln!("[{}] After pitch-up: alt={:.1} spd={:.1} max_aoa={:.3}rad({:.1}°)",
            label, state.altitude_m, state.airspeed_ms,
            max_aoa_seen, max_aoa_seen.to_degrees());

        // Phase 3: Release controls, recover for 5 seconds
        let neutral_ctrl = ControlState { throttle: 0.7, ..Default::default() };
        for _ in 0..600 {
            state = engine.step(&state, &neutral_ctrl, dt);
        }
        eprintln!("[{}] After recovery: alt={:.1} spd={:.1} climb={:.1}",
            label, state.altitude_m, state.airspeed_ms, state.climb_rate_ms);

        // Must recover speed: aircraft should not remain in deep stall
        assert!(state.airspeed_ms > fc.stall_speed_ms * 0.5,
            "[{}] Should recover from pitch-up, speed={:.1} vs stall={:.1}",
            label, state.airspeed_ms, fc.stall_speed_ms);
        // Altitude should not have dropped to zero (no crash)
        assert!(state.altitude_m > 50.0,
            "[{}] Should not crash, alt={:.1}", label, state.altitude_m);
    }

    #[test]
    fn test_biplane_pitch_up_recovery() {
        test_pitch_up_recovery(&AircraftModel::biplane(), "biplane");
    }

    #[test]
    fn test_monoplane_pitch_up_recovery() {
        test_pitch_up_recovery(&AircraftModel::monoplane(), "monoplane");
    }

    #[test]
    fn test_jet_pitch_up_recovery() {
        test_pitch_up_recovery(&AircraftModel::jet(), "jet");
    }

    // ─────────────────────────────────────────────────────────────
    // Multi-profile: roll response
    // ─────────────────────────────────────────────────────────────

    #[test]
    fn test_all_profiles_respond_to_roll() {
        for (label, model) in [
            ("biplane", AircraftModel::biplane()),
            ("monoplane", AircraftModel::monoplane()),
            ("jet", AircraftModel::jet()),
        ] {
            let mut engine = AeroBlendPhysicsEngine::new();
            let init_state = engine.init(&model);
            let dt = 1.0 / 120.0;

            // Cruise 1 second
            let cruise_ctrl = ControlState { throttle: 0.6, ..Default::default() };
            let mut state = init_state;
            for _ in 0..120 {
                state = engine.step(&state, &cruise_ctrl, dt);
            }

            // Roll right for 2 seconds
            let roll_ctrl = ControlState { throttle: 0.6, roll: 1.0, ..Default::default() };
            for _ in 0..240 {
                state = engine.step(&state, &roll_ctrl, dt);
            }

            eprintln!("[{}] After roll: roll={:.1}° heading={:.1}°",
                label, state.roll_rad.to_degrees(), state.heading_rad.to_degrees());

            assert!(state.roll_rad.abs() > 0.1,
                "[{}] Roll should respond, got {:.3}", label, state.roll_rad);
        }
    }
}
