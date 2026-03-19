//! `AeroBlend` core: `#[repr(C)]` POD types for the FFI boundary.
//!
//! These structs are designed to be passed between Rust and C++ across the FFI.
//! All use C-compatible memory layout with no heap allocations in the struct itself.

// JSF-Rust SR-801: Enable clippy lints for code quality
#![warn(clippy::all, clippy::pedantic)]
#![allow(clippy::similar_names)]

// JSF-Rust SR-803: Forbid unsafe code in non-FFI crates
#![forbid(unsafe_code)]

use std::os::raw::c_char;

use glam::{DMat3, DVec3};

use aeroblend_physics::types as internal;

// ─────────────────────────────────────────────────────────────
// POD types for FFI
// ─────────────────────────────────────────────────────────────

/// 3D vector (f64) - FFI safe.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Vec3C {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<DVec3> for Vec3C {
    fn from(v: DVec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

impl From<Vec3C> for DVec3 {
    fn from(v: Vec3C) -> Self {
        DVec3::new(v.x, v.y, v.z)
    }
}

/// 3x3 matrix (f64, column-major) - FFI safe.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Mat3C {
    pub m: [f64; 9],
}

impl Default for Mat3C {
    fn default() -> Self {
        Self {
            m: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }
}

impl From<DMat3> for Mat3C {
    fn from(mat: DMat3) -> Self {
        Self { m: mat.to_cols_array() }
    }
}

impl From<Mat3C> for DMat3 {
    fn from(m: Mat3C) -> Self {
        DMat3::from_cols_array(&m.m)
    }
}

/// 4x4 matrix (f32, column-major for OpenGL) - FFI safe.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Mat4C {
    pub m: [f32; 16],
}

impl Default for Mat4C {
    fn default() -> Self {
        Self {
            m: [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }
}

/// Quaternion (f64) - FFI safe.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct QuatC {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for QuatC {
    fn default() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }
}

/// Pre-computed flight characteristics - FFI safe POD.
///
/// These parameters characterize an aircraft's flight envelope and are used
/// by the mouse-aim instructor to adapt control gains per airframe.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct FlightCharacteristicsC {
    /// Stall speed at sea level (m/s)
    pub stall_speed_ms: f64,
    /// Reference cruise speed (m/s): ~1.3 * stall speed
    pub ref_speed_ms: f64,
    /// Maximum angle of attack before stall (rad)
    pub max_aoa_rad: f64,
    /// Total wing area (m^2)
    pub total_wing_area_m2: f64,
    /// Control authority: control moment / inertia ratio (average)
    pub control_authority: f64,
    /// Per-axis control authority: pitch torque / Ixx
    pub ca_pitch: f64,
    /// Per-axis control authority: roll torque / Izz
    pub ca_roll: f64,
    /// Per-axis control authority: yaw torque / Iyy
    pub ca_yaw: f64,
    /// Wing loading (kg/m^2)
    pub wing_loading_kg_m2: f64,
    /// Thrust-to-weight ratio
    pub thrust_to_weight: f64,
    /// Maximum pitch rate from elevator authority (rad/s)
    pub max_pitch_rate_rad_s: f64,
    /// Maximum roll rate from aileron authority (rad/s)
    pub max_roll_rate_rad_s: f64,
}

impl Default for FlightCharacteristicsC {
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

impl From<&internal::FlightCharacteristics> for FlightCharacteristicsC {
    fn from(fc: &internal::FlightCharacteristics) -> Self {
        Self {
            stall_speed_ms: fc.stall_speed_ms,
            ref_speed_ms: fc.ref_speed_ms,
            max_aoa_rad: fc.max_aoa_rad,
            total_wing_area_m2: fc.total_wing_area_m2,
            control_authority: fc.control_authority,
            ca_pitch: fc.ca_pitch,
            ca_roll: fc.ca_roll,
            ca_yaw: fc.ca_yaw,
            wing_loading_kg_m2: fc.wing_loading_kg_m2,
            thrust_to_weight: fc.thrust_to_weight,
            max_pitch_rate_rad_s: fc.max_pitch_rate_rad_s,
            max_roll_rate_rad_s: fc.max_roll_rate_rad_s,
        }
    }
}

/// Aircraft physics state - FFI safe POD.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AircraftPhysicsStateC {
    pub position: Vec3C,
    pub velocity: Vec3C,
    pub acceleration: Vec3C,
    pub orientation: Mat3C,
    pub angular_velocity: Vec3C,
    pub mass_kg: f64,
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
    pub flight_chars: FlightCharacteristicsC,
}

impl Default for AircraftPhysicsStateC {
    fn default() -> Self {
        Self {
            position: Vec3C::default(),
            velocity: Vec3C::default(),
            acceleration: Vec3C::default(),
            orientation: Mat3C::default(),
            angular_velocity: Vec3C::default(),
            mass_kg: 5000.0,
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
            flight_chars: FlightCharacteristicsC::default(),
        }
    }
}

impl From<&internal::AircraftPhysicsState> for AircraftPhysicsStateC {
    fn from(s: &internal::AircraftPhysicsState) -> Self {
        Self {
            position: s.position.into(),
            velocity: s.velocity.into(),
            acceleration: s.acceleration.into(),
            orientation: s.orientation.into(),
            angular_velocity: s.angular_velocity.into(),
            mass_kg: s.mass_kg,
            airspeed_ms: s.airspeed_ms,
            angle_of_attack_rad: s.angle_of_attack_rad,
            sideslip_rad: s.sideslip_rad,
            altitude_m: s.altitude_m,
            mach: s.mach,
            heading_rad: s.heading_rad,
            pitch_rad: s.pitch_rad,
            roll_rad: s.roll_rad,
            g_force: s.g_force,
            climb_rate_ms: s.climb_rate_ms,
            ground_speed_ms: s.ground_speed_ms,
            throttle: s.throttle,
            flight_chars: FlightCharacteristicsC::from(&s.flight_chars),
        }
    }
}

/// Control input state - FFI safe.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ControlStateC {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub throttle: f64,
    pub brake: f64,
    pub flaps: f64,
    pub gear_down: u8, // bool as u8 for C ABI
    pub airbrake: u8,
    pub firing: u8,
    pub mouse_aim_direction: Vec3C,
}

impl Default for ControlStateC {
    fn default() -> Self {
        Self {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            throttle: 0.0,
            brake: 0.0,
            flaps: 0.0,
            gear_down: 1,
            airbrake: 0,
            firing: 0,
            mouse_aim_direction: Vec3C { x: 0.0, y: 0.0, z: 1.0 },
        }
    }
}

impl From<&ControlStateC> for internal::ControlState {
    fn from(c: &ControlStateC) -> Self {
        Self {
            pitch: c.pitch,
            yaw: c.yaw,
            roll: c.roll,
            throttle: c.throttle,
            brake: c.brake,
            flaps: c.flaps,
            gear_down: c.gear_down != 0,
            airbrake: c.airbrake != 0,
            mouse_aim_direction: c.mouse_aim_direction.into(),
        }
    }
}

/// Camera state - FFI safe.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CameraStateC {
    pub position: Vec3C,
    pub target: Vec3C,
    pub up: Vec3C,
    pub fov_deg: f64,
    pub near: f64,
    pub far: f64,
    pub mode: u32, // 0=chase, 1=cockpit, 2=free
}

impl Default for CameraStateC {
    fn default() -> Self {
        Self {
            position: Vec3C { x: 0.0, y: 5.0, z: -15.0 },
            target: Vec3C::default(),
            up: Vec3C { x: 0.0, y: 1.0, z: 0.0 },
            fov_deg: 75.0,
            near: 0.1,
            // JSF-Rust SR-505: Add underscores to large numeric literals for readability
            far: 100_000.0,
            mode: 0,
        }
    }
}

impl From<&internal::CameraState> for CameraStateC {
    fn from(c: &internal::CameraState) -> Self {
        Self {
            position: c.position.into(),
            target: c.target.into(),
            up: c.up.into(),
            fov_deg: c.fov_deg,
            near: c.near,
            far: c.far,
            mode: match c.mode {
                internal::CameraMode::Chase => 0,
                internal::CameraMode::Cockpit => 1,
                internal::CameraMode::Free => 2,
            },
        }
    }
}

/// Single mesh data for GPU upload.
#[repr(C)]
#[derive(Debug)]
pub struct MeshDataC {
    pub vertices: *const [f32; 3],
    pub normals: *const [f32; 3],
    pub uvs: *const [f32; 2],
    pub indices: *const u32,
    pub vertex_count: u32,
    pub index_count: u32,
    pub name: *const c_char,
}

/// Aircraft model data.
#[repr(C)]
#[derive(Debug)]
pub struct AircraftModelC {
    pub mesh_count: u32,
    pub mass_kg: f64,
    pub bbox_min: Vec3C,
    pub bbox_max: Vec3C,
}

// ============================================================
// 무장 시스템 FFI 타입
// ============================================================

/// 기총 프리셋 ID
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GunPresetC {
    /// .50 cal M2 Browning (12.7mm)
    BrowningM2 = 0,
    /// 20mm AN/M2 cannon
    AnM2Cannon = 1,
    /// 7.7mm Vickers
    Vickers77 = 2,
}

impl Default for GunPresetC {
    fn default() -> Self { GunPresetC::BrowningM2 }
}

/// 기총 스펙 (C 호환)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct GunSpecC {
    pub caliber_mm: f64,
    pub mass_kg: f64,
    pub muzzle_velocity_ms: f64,
    pub drag_coefficient: f64,
    pub cross_section_m2: f64,
    pub fire_rate_rpm: f64,
    pub muzzle_position: Vec3C,
    pub muzzle_direction: Vec3C,
    pub ammo_capacity: u32,
    pub spread_rad: f64,
}

impl Default for GunSpecC {
    fn default() -> Self {
        Self {
            caliber_mm: 12.7,
            mass_kg: 0.045,
            muzzle_velocity_ms: 890.0,
            drag_coefficient: 0.295,
            cross_section_m2: std::f64::consts::PI * (0.00635_f64).powi(2),
            fire_rate_rpm: 750.0,
            muzzle_position: Vec3C::default(),
            muzzle_direction: Vec3C { x: 0.0, y: 0.0, z: 1.0 },
            ammo_capacity: 500,
            spread_rad: 0.003,
        }
    }
}

/// 탄환 렌더링 상태 (Rust → C++)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ProjectileStateC {
    pub position: Vec3C,
    pub velocity: Vec3C,
    pub active: u8,
}

impl Default for ProjectileStateC {
    fn default() -> Self {
        Self {
            position: Vec3C::default(),
            velocity: Vec3C::default(),
            active: 0,
        }
    }
}

/// 무장 시스템 요약 상태 (Rust → C++)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WeaponStateC {
    pub gun_count: u32,
    pub total_ammo_remaining: u32,
    pub total_ammo_capacity: u32,
    pub is_firing: u8,
    pub gun_preset: GunPresetC,
}

impl Default for WeaponStateC {
    fn default() -> Self {
        Self {
            gun_count: 0,
            total_ammo_remaining: 0,
            total_ammo_capacity: 0,
            is_firing: 0,
            gun_preset: GunPresetC::default(),
        }
    }
}
