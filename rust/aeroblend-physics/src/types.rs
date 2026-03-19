//! Core physics types shared across the physics crate.

use glam::{DMat3, DQuat, DVec3};
use aeroblend_math::{quat_to_rotation_matrix, rotation_matrix_to_quat, quat_to_euler};

/// Engine type enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineType {
    Propeller,
    Jet,
    Rotor,
    MultiRotor,
    Unknown,
}

/// Engine specification.
#[derive(Debug, Clone)]
pub struct EngineSpec {
    pub engine_type: EngineType,
    pub max_thrust_n: f64,
    pub position_local: DVec3,
    pub direction_local: DVec3,
}

impl Default for EngineSpec {
    fn default() -> Self {
        Self {
            engine_type: EngineType::Unknown,
            max_thrust_n: 50000.0,
            position_local: DVec3::ZERO,
            direction_local: DVec3::new(0.0, 0.0, 1.0),
        }
    }
}

/// Aerodynamic surface specification.
#[derive(Debug, Clone)]
pub struct AeroSurface {
    pub name: String,
    pub area_m2: f64,
    pub span_m: f64,
    pub chord_m: f64,
    pub position_local: DVec3,
    pub normal_local: DVec3,
    pub cl_alpha: f64,
    pub cd0: f64,
    pub cl_max: f64,
    pub alpha_stall_rad: f64,
    pub alpha_zero_lift_rad: f64,
}

impl Default for AeroSurface {
    fn default() -> Self {
        Self {
            name: String::new(),
            area_m2: 10.0,
            span_m: 10.0,
            chord_m: 1.0,
            position_local: DVec3::ZERO,
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 2.0 * std::f64::consts::PI,
            cd0: 0.02,
            cl_max: 1.5,
            alpha_stall_rad: 0.27,
            alpha_zero_lift_rad: 0.0,
        }
    }
}

/// Control input state.
#[derive(Debug, Clone)]
pub struct ControlState {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub throttle: f64,
    pub brake: f64,
    pub flaps: f64,
    pub gear_down: bool,
    pub airbrake: bool,
    pub mouse_aim_direction: DVec3,
}

impl Default for ControlState {
    fn default() -> Self {
        Self {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            throttle: 0.0,
            brake: 0.0,
            flaps: 0.0,
            gear_down: true,
            airbrake: false,
            mouse_aim_direction: DVec3::new(0.0, 0.0, 1.0),
        }
    }
}

/// Full aircraft physics state.
#[derive(Debug, Clone)]
pub struct AircraftPhysicsState {
    pub position: DVec3,
    pub velocity: DVec3,
    pub acceleration: DVec3,
    pub orientation: DMat3,
    pub angular_velocity: DVec3,
    pub mass_kg: f64,
    pub inertia_tensor: DMat3,

    // Derived
    pub airspeed_ms: f64,
    pub angle_of_attack_rad: f64,
    pub sideslip_rad: f64,
    pub altitude_m: f64,
    pub mach: f64,
    pub heading_rad: f64,
    pub pitch_rad: f64,
    pub roll_rad: f64,
    pub g_force: f64,
    pub climb_rate_ms: f64,
    pub ground_speed_ms: f64,
    pub throttle: f64,

    pub aero_surfaces: Vec<AeroSurface>,
    pub engines: Vec<EngineSpec>,

    /// Pre-computed flight characteristics for this aircraft.
    pub flight_chars: FlightCharacteristics,
}

impl Default for AircraftPhysicsState {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            acceleration: DVec3::ZERO,
            orientation: DMat3::IDENTITY,
            angular_velocity: DVec3::ZERO,
            mass_kg: 5000.0,
            inertia_tensor: DMat3::from_diagonal(DVec3::new(5000.0, 8000.0, 3000.0)),
            airspeed_ms: 0.0,
            angle_of_attack_rad: 0.0,
            sideslip_rad: 0.0,
            altitude_m: 0.0,
            mach: 0.0,
            heading_rad: 0.0,
            pitch_rad: 0.0,
            roll_rad: 0.0,
            g_force: 1.0,
            climb_rate_ms: 0.0,
            ground_speed_ms: 0.0,
            throttle: 0.0,
            aero_surfaces: Vec::new(),
            engines: Vec::new(),
            flight_chars: FlightCharacteristics::default(),
        }
    }
}

/// Mesh data for rendering.
#[derive(Debug, Clone, Default)]
pub struct MeshData {
    pub vertices: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
    pub name: String,
}

/// Complete aircraft model.
#[derive(Debug, Clone)]
pub struct AircraftModel {
    pub meshes: Vec<MeshData>,
    pub aero_surfaces: Vec<AeroSurface>,
    pub engines: Vec<EngineSpec>,
    pub mass_kg: f64,
    pub inertia_tensor: DMat3,
    pub bounding_box_min: DVec3,
    pub bounding_box_max: DVec3,
}

impl Default for AircraftModel {
    fn default() -> Self {
        Self::biplane()
    }
}

impl AircraftModel {
    /// Antonov An-2 style biplane: high stability, propeller, very hard to stall.
    ///
    /// Based on An-2 specs: 71.5 m² total wing area, 5500 kg MTOW,
    /// 1000 HP radial engine, stall speed ~40 km/h, cruise ~180 km/h.
    pub fn biplane() -> Self {
        let upper_wing = AeroSurface {
            name: "upper_wing".to_string(),
            area_m2: 43.6,
            span_m: 18.18,
            chord_m: 2.4,   // MAC ≈ 43.6 / 18.18
            position_local: DVec3::new(0.0, 1.8, 0.5),  // above fuselage, slightly forward
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 5.8,  // slightly below 2π due to biplane interference
            cd0: 0.018,     // low drag for large wing
            cl_max: 3.0,    // very high — leading-edge slats
            alpha_stall_rad: 0.40, // ~23° — extremely stall-resistant
            alpha_zero_lift_rad: -0.035, // slight camber, ~-2° zero-lift angle
        };

        let lower_wing = AeroSurface {
            name: "lower_wing".to_string(),
            area_m2: 27.9,
            span_m: 14.24,
            chord_m: 1.96,  // MAC ≈ 27.9 / 14.24
            position_local: DVec3::new(0.0, -0.5, 0.2),  // below fuselage
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 5.5,  // slightly less than upper (interference)
            cd0: 0.022,     // more drag from struts/wires
            cl_max: 2.5,
            alpha_stall_rad: 0.35,
            alpha_zero_lift_rad: -0.035,
        };

        let h_stabilizer = AeroSurface {
            name: "h_stabilizer".to_string(),
            area_m2: 8.0,
            span_m: 5.0,
            chord_m: 1.6,
            position_local: DVec3::new(0.0, 0.5, -7.0),  // tail, behind CG
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 4.5,
            cd0: 0.015,
            cl_max: 1.2,
            alpha_stall_rad: 0.25,
            alpha_zero_lift_rad: 0.0,
        };

        let v_stabilizer = AeroSurface {
            name: "v_stabilizer".to_string(),
            area_m2: 4.5,
            span_m: 2.5,
            chord_m: 1.8,
            position_local: DVec3::new(0.0, 1.5, -7.0),
            normal_local: DVec3::new(1.0, 0.0, 0.0),  // side-facing
            cl_alpha: 4.0,
            cd0: 0.015,
            cl_max: 1.0,
            alpha_stall_rad: 0.25,
            alpha_zero_lift_rad: 0.0,
        };

        // Single radial engine with propeller, nose-mounted
        let engine = EngineSpec {
            engine_type: EngineType::Propeller,
            max_thrust_n: 12_000.0,  // ~1000 HP propeller static thrust
            position_local: DVec3::new(0.0, 0.3, 5.5),  // nose
            direction_local: DVec3::new(0.0, 0.0, 1.0),  // forward
        };

        // An-2 inertia tensor (biplane = high roll inertia from wide wings + struts)
        // Ixx (pitch): ~12000 kg·m², Iyy (yaw): ~18000 kg·m², Izz (roll): ~8000 kg·m²
        let inertia = DMat3::from_diagonal(DVec3::new(12_000.0, 18_000.0, 8_000.0));

        Self {
            meshes: Vec::new(),
            aero_surfaces: vec![upper_wing, lower_wing, h_stabilizer, v_stabilizer],
            engines: vec![engine],
            mass_kg: 4_500.0,  // typical operating weight
            inertia_tensor: inertia,
            bounding_box_min: DVec3::new(-9.0, -2.5, -7.0),
            bounding_box_max: DVec3::new(9.0, 4.0, 6.0),
        }
    }

    /// Cessna 172-style monoplane: moderate stability, propeller, docile handling.
    ///
    /// Based on Cessna 172 specs: 16.2 m² wing area, 1111 kg MTOW,
    /// 160 HP engine, stall speed ~87 km/h (24 m/s), cruise ~226 km/h (63 m/s).
    pub fn monoplane() -> Self {
        let wing = AeroSurface {
            name: "main_wing".to_string(),
            area_m2: 16.2,
            span_m: 11.0,
            chord_m: 1.47,
            position_local: DVec3::new(0.0, 0.5, 0.3),
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 5.7,
            cd0: 0.025,
            cl_max: 1.6,
            alpha_stall_rad: 0.28,  // ~16°
            alpha_zero_lift_rad: -0.03,
        };

        let h_stabilizer = AeroSurface {
            name: "h_stabilizer".to_string(),
            area_m2: 3.5,
            span_m: 3.4,
            chord_m: 1.03,
            position_local: DVec3::new(0.0, 0.3, -4.5),
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 4.0,
            cd0: 0.015,
            cl_max: 1.0,
            alpha_stall_rad: 0.22,
            alpha_zero_lift_rad: 0.0,
        };

        let v_stabilizer = AeroSurface {
            name: "v_stabilizer".to_string(),
            area_m2: 1.8,
            span_m: 1.6,
            chord_m: 1.12,
            position_local: DVec3::new(0.0, 1.0, -4.5),
            normal_local: DVec3::new(1.0, 0.0, 0.0),
            cl_alpha: 3.5,
            cd0: 0.015,
            cl_max: 0.8,
            alpha_stall_rad: 0.22,
            alpha_zero_lift_rad: 0.0,
        };

        let engine = EngineSpec {
            engine_type: EngineType::Propeller,
            max_thrust_n: 4_000.0,  // ~160 HP propeller
            position_local: DVec3::new(0.0, 0.2, 2.5),
            direction_local: DVec3::new(0.0, 0.0, 1.0),
        };

        // Cessna 172 inertia tensor (lighter, smaller)
        let inertia = DMat3::from_diagonal(DVec3::new(1_400.0, 1_800.0, 800.0));

        Self {
            meshes: Vec::new(),
            aero_surfaces: vec![wing, h_stabilizer, v_stabilizer],
            engines: vec![engine],
            mass_kg: 1_111.0,
            inertia_tensor: inertia,
            bounding_box_min: DVec3::new(-5.5, -1.5, -5.0),
            bounding_box_max: DVec3::new(5.5, 2.5, 3.0),
        }
    }

    /// F-16-style jet fighter: high performance, fast, agile.
    ///
    /// Based on F-16 specs: 27.87 m² wing area, 12000 kg combat weight,
    /// 76.3 kN dry thrust, stall speed ~150 km/h (42 m/s), cruise ~900 km/h (250 m/s).
    pub fn jet() -> Self {
        let wing = AeroSurface {
            name: "main_wing".to_string(),
            area_m2: 27.87,
            span_m: 9.96,
            chord_m: 2.80,
            position_local: DVec3::new(0.0, 0.0, -0.5),
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 4.5,  // delta wing, lower lift slope
            cd0: 0.015,
            cl_max: 1.2,
            alpha_stall_rad: 0.35,  // ~20° — LEX vortex lift extends stall
            alpha_zero_lift_rad: 0.0,
        };

        let h_stabilizer = AeroSurface {
            name: "h_stabilizer".to_string(),
            area_m2: 5.6,
            span_m: 5.6,
            chord_m: 1.0,
            position_local: DVec3::new(0.0, 0.0, -5.5),
            normal_local: DVec3::new(0.0, 1.0, 0.0),
            cl_alpha: 3.8,
            cd0: 0.012,
            cl_max: 0.9,
            alpha_stall_rad: 0.25,
            alpha_zero_lift_rad: 0.0,
        };

        let v_stabilizer = AeroSurface {
            name: "v_stabilizer".to_string(),
            area_m2: 5.0,
            span_m: 3.0,
            chord_m: 1.67,
            position_local: DVec3::new(0.0, 2.0, -5.5),
            normal_local: DVec3::new(1.0, 0.0, 0.0),
            cl_alpha: 3.5,
            cd0: 0.012,
            cl_max: 0.8,
            alpha_stall_rad: 0.22,
            alpha_zero_lift_rad: 0.0,
        };

        let engine = EngineSpec {
            engine_type: EngineType::Jet,
            max_thrust_n: 76_300.0,  // dry thrust ~76.3 kN
            position_local: DVec3::new(0.0, 0.0, -6.0),
            direction_local: DVec3::new(0.0, 0.0, 1.0),
        };

        // F-16 inertia tensor (compact, heavy for its size)
        let inertia = DMat3::from_diagonal(DVec3::new(12_000.0, 75_000.0, 65_000.0));

        Self {
            meshes: Vec::new(),
            aero_surfaces: vec![wing, h_stabilizer, v_stabilizer],
            engines: vec![engine],
            mass_kg: 12_000.0,
            inertia_tensor: inertia,
            bounding_box_min: DVec3::new(-5.0, -1.5, -7.5),
            bounding_box_max: DVec3::new(5.0, 3.0, 7.5),
        }
    }
}

/// Internal 6DoF physics state using quaternion orientation.
#[derive(Debug, Clone)]
pub struct PhysicsState6DoF {
    pub position: DVec3,
    pub velocity: DVec3,
    pub orientation_q: DQuat,
    pub angular_velocity: DVec3,
    pub mass_kg: f64,
    pub inertia_tensor: DMat3,
    pub inv_inertia_tensor: DMat3,
    pub aero_surfaces: Vec<AeroSurface>,
    pub engines: Vec<EngineSpec>,
    pub throttle: f64,
}

impl Default for PhysicsState6DoF {
    fn default() -> Self {
        let inertia = DMat3::from_diagonal(DVec3::new(5000.0, 8000.0, 3000.0));
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation_q: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            mass_kg: 5000.0,
            inertia_tensor: inertia,
            inv_inertia_tensor: inertia.inverse(),
            aero_surfaces: Vec::new(),
            engines: Vec::new(),
            throttle: 0.0,
        }
    }
}

impl PhysicsState6DoF {
    pub fn from_interface_state(state: &AircraftPhysicsState) -> Self {
        let inertia = state.inertia_tensor;
        Self {
            position: state.position,
            velocity: state.velocity,
            orientation_q: rotation_matrix_to_quat(state.orientation),
            angular_velocity: state.angular_velocity,
            mass_kg: state.mass_kg,
            inertia_tensor: inertia,
            inv_inertia_tensor: inertia.inverse(),
            aero_surfaces: state.aero_surfaces.clone(),
            engines: state.engines.clone(),
            throttle: state.throttle,
        }
    }

    pub fn to_interface_state(&self) -> AircraftPhysicsState {
        let rot = quat_to_rotation_matrix(self.orientation_q);
        let speed = self.velocity.length();
        let inv_rot = rot.transpose();
        let v_body = inv_rot * self.velocity;

        let (aoa, sideslip) = if speed > 1.0 {
            (
                (-v_body.y).atan2(v_body.z),
                v_body.x.atan2(v_body.z),
            )
        } else {
            (0.0, 0.0)
        };

        let altitude = self.position.y;
        let speed_of_sound = 340.3;
        let mach = speed / speed_of_sound;

        let (roll, pitch, yaw) = quat_to_euler(self.orientation_q);

        let climb_rate = self.velocity.y;
        let ground_speed = (self.velocity.x * self.velocity.x
            + self.velocity.z * self.velocity.z)
            .sqrt();

        AircraftPhysicsState {
            position: self.position,
            velocity: self.velocity,
            acceleration: DVec3::ZERO,
            orientation: rot,
            angular_velocity: self.angular_velocity,
            mass_kg: self.mass_kg,
            inertia_tensor: self.inertia_tensor,
            airspeed_ms: speed,
            angle_of_attack_rad: aoa,
            sideslip_rad: sideslip,
            altitude_m: altitude,
            mach,
            heading_rad: yaw,
            pitch_rad: pitch,
            roll_rad: roll,
            g_force: 1.0,
            climb_rate_ms: climb_rate,
            ground_speed_ms: ground_speed,
            throttle: self.throttle,
            aero_surfaces: self.aero_surfaces.clone(),
            engines: self.engines.clone(),
            flight_chars: FlightCharacteristics::default(),
        }
    }
}

/// Pre-computed flight characteristics derived from aircraft geometry.
///
/// These parameters characterize an aircraft's flight envelope and are used
/// by the mouse-aim instructor to adapt control gains to each airframe.
#[derive(Debug, Clone, Copy)]
pub struct FlightCharacteristics {
    /// Stall speed at sea level (m/s): V_stall = sqrt(2W / (rho * S * CL_max))
    pub stall_speed_ms: f64,
    /// Reference cruise speed (m/s): ~1.3 * stall speed
    pub ref_speed_ms: f64,
    /// Maximum angle of attack before stall (rad), from aero surfaces
    pub max_aoa_rad: f64,
    /// Total wing area (m^2) across all lifting surfaces
    pub total_wing_area_m2: f64,
    /// Control authority: ratio of max control moment to inertia magnitude.
    /// Higher = more responsive, lower = more sluggish.
    pub control_authority: f64,
    /// Per-axis control authority: pitch torque / Ixx
    pub ca_pitch: f64,
    /// Per-axis control authority: roll torque / Izz
    pub ca_roll: f64,
    /// Per-axis control authority: yaw torque / Iyy
    pub ca_yaw: f64,
    /// Wing loading (kg/m^2): mass / total_wing_area
    pub wing_loading_kg_m2: f64,
    /// Thrust-to-weight ratio at full throttle
    pub thrust_to_weight: f64,
    /// Max pitch rate (rad/s): elevator moment / pitch inertia at ref speed
    pub max_pitch_rate_rad_s: f64,
    /// Max roll rate (rad/s): aileron moment / roll inertia at ref speed
    pub max_roll_rate_rad_s: f64,
}

impl Default for FlightCharacteristics {
    fn default() -> Self {
        Self {
            stall_speed_ms: 30.0,
            ref_speed_ms: 39.0,
            max_aoa_rad: 0.27,
            total_wing_area_m2: 30.0,
            control_authority: 1.0,
            ca_pitch: 1.0,
            ca_roll: 1.0,
            ca_yaw: 1.0,
            wing_loading_kg_m2: 166.0,
            thrust_to_weight: 0.35,
            max_pitch_rate_rad_s: 1.0,
            max_roll_rate_rad_s: 2.0,
        }
    }
}

/// Camera state.
#[derive(Debug, Clone)]
pub struct CameraState {
    pub position: DVec3,
    pub target: DVec3,
    pub up: DVec3,
    pub fov_deg: f64,
    pub near: f64,
    pub far: f64,
    pub mode: CameraMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    Chase,
    Cockpit,
    Free,
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            position: DVec3::new(0.0, 5.0, -15.0),
            target: DVec3::ZERO,
            up: DVec3::Y,
            fov_deg: 75.0,
            near: 0.1,
            // JSF-Rust SR-505: Add underscores to large numeric literals for readability
            far: 100_000.0,
            mode: CameraMode::Chase,
        }
    }
}
