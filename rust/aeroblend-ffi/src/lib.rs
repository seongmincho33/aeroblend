//! AeroBlend FFI: extern "C" functions exported for C++ consumption.
//!
//! Provides the C ABI interface to the Rust physics engine, model loader,
//! and camera system.

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;
use std::sync::Mutex;

use aeroblend_ballistics;
use aeroblend_core::*;
use aeroblend_math::*;
use aeroblend_physics::engine::AeroBlendPhysicsEngine;
use aeroblend_physics::types as internal;

// ─────────────────────────────────────────────────────────────
// Opaque types
// ─────────────────────────────────────────────────────────────

/// Opaque physics engine handle.
pub struct PhysicsEngine {
    inner: AeroBlendPhysicsEngine,
    state: internal::AircraftPhysicsState,
}

/// Opaque loaded model handle.
pub struct LoadedModel {
    model: internal::AircraftModel,
    mesh_names: Vec<CString>,
}

/// 무장 시스템 불투명 핸들 (C++에서는 포인터로만 사용)
pub struct WeaponSystemHandle {
    inner: aeroblend_ballistics::WeaponSystem,
    preset: GunPresetC,
}

// ─────────────────────────────────────────────────────────────
// Camera state (persistent across frames)
// ─────────────────────────────────────────────────────────────

// JSF-Rust SR-401: Replace static mut with Mutex for thread-safe mutable global state
static CHASE_POS: Mutex<[f64; 3]> = Mutex::new([0.0, 5.0, -15.0]);

const CHASE_DISTANCE: f64 = 20.0;
const CHASE_HEIGHT: f64 = 6.0;
const CHASE_SMOOTHING: f64 = 5.0;
const COCKPIT_OFFSET: [f64; 3] = [0.0, 1.2, 2.0];

// ─────────────────────────────────────────────────────────────
// Physics Engine lifecycle
// ─────────────────────────────────────────────────────────────

/// Create a new physics engine instance.
///
/// # Returns
/// Non-null pointer to a `PhysicsEngine` that must be freed with `aeroblend_physics_destroy`.
#[no_mangle]
pub extern "C" fn aeroblend_physics_create() -> *mut PhysicsEngine {
    let engine = Box::new(PhysicsEngine {
        inner: AeroBlendPhysicsEngine::new(),
        state: internal::AircraftPhysicsState::default(),
    });
    Box::into_raw(engine)
}

/// Destroy a physics engine instance and free its memory.
///
/// # Safety
/// `engine` must be either null or a valid pointer returned by `aeroblend_physics_create`.
/// Must not be called more than once with the same pointer.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_physics_destroy(engine: *mut PhysicsEngine) {
    // SAFETY: Caller guarantees that `engine` is either null or a valid pointer
    // previously returned by `aeroblend_physics_create`, and that it has not been
    // freed before. This function takes ownership and frees the memory exactly once.
    if !engine.is_null() {
        drop(Box::from_raw(engine));
    }
}

// ─────────────────────────────────────────────────────────────
// Model loading
// ─────────────────────────────────────────────────────────────

/// Load an aircraft model from a glTF/glb file.
///
/// # Safety
/// `path` must be either null or a valid pointer to a null-terminated C string.
///
/// # Returns
/// Non-null pointer on success, null on failure. Must be freed with `aeroblend_model_destroy`.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_model_load(path: *const c_char) -> *mut LoadedModel {
    // SAFETY: Caller guarantees that `path` is either null or points to a valid
    // null-terminated C string with a lifetime at least as long as this function call.
    if path.is_null() {
        return ptr::null_mut();
    }
    let path_str = match CStr::from_ptr(path).to_str() {
        Ok(s) => s,
        Err(_) => return ptr::null_mut(),
    };

    match aeroblend_importer::load_aircraft_model(path_str) {
        Ok(model) => {
            let mesh_names: Vec<CString> = model
                .meshes
                .iter()
                .map(|m| CString::new(m.name.as_str()).unwrap_or_default())
                .collect();
            let loaded = Box::new(LoadedModel { model, mesh_names });
            Box::into_raw(loaded)
        }
        Err(_) => ptr::null_mut(),
    }
}

/// Destroy a loaded model and free its memory.
///
/// # Safety
/// `model` must be either null or a valid pointer returned by `aeroblend_model_load`.
/// Must not be called more than once with the same pointer.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_model_destroy(model: *mut LoadedModel) {
    // SAFETY: Caller guarantees that `model` is either null or a valid pointer
    // previously returned by `aeroblend_model_load`, and has not been freed before.
    if !model.is_null() {
        drop(Box::from_raw(model));
    }
}

/// Get aircraft model metadata (mesh count, mass, bounding box).
///
/// # Safety
/// `model` must be either null or a valid pointer to a `LoadedModel`.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_model_get_data(model: *const LoadedModel) -> AircraftModelC {
    // SAFETY: Caller guarantees that `model` is either null or a valid pointer
    // to a LoadedModel with a lifetime managed by the caller.
    if model.is_null() {
        return AircraftModelC {
            mesh_count: 0,
            mass_kg: 5000.0,
            bbox_min: Vec3C::default(),
            bbox_max: Vec3C::default(),
        };
    }
    let m = &(*model).model;
    AircraftModelC {
        mesh_count: m.meshes.len() as u32,
        mass_kg: m.mass_kg,
        bbox_min: glam::DVec3::new(
            m.bounding_box_min.x,
            m.bounding_box_min.y,
            m.bounding_box_min.z,
        ).into(),
        bbox_max: glam::DVec3::new(
            m.bounding_box_max.x,
            m.bounding_box_max.y,
            m.bounding_box_max.z,
        ).into(),
    }
}

/// Get mesh data at the given index.
///
/// # Safety
/// `model` must be either null or a valid pointer to a `LoadedModel`.
/// Returned pointers are only valid as long as the `LoadedModel` exists.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_model_get_mesh(
    model: *const LoadedModel,
    index: u32,
) -> MeshDataC {
    // SAFETY: Caller guarantees that `model` is either null or a valid pointer
    // to a LoadedModel. Returned pointers are valid as long as the LoadedModel exists.
    let null_mesh = MeshDataC {
        vertices: ptr::null(),
        normals: ptr::null(),
        uvs: ptr::null(),
        indices: ptr::null(),
        vertex_count: 0,
        index_count: 0,
        name: ptr::null(),
    };

    if model.is_null() {
        return null_mesh;
    }
    let lm = &*model;
    let idx = index as usize;
    if idx >= lm.model.meshes.len() {
        return null_mesh;
    }
    let mesh = &lm.model.meshes[idx];

    MeshDataC {
        vertices: mesh.vertices.as_ptr(),
        normals: mesh.normals.as_ptr(),
        uvs: mesh.uvs.as_ptr(),
        indices: mesh.indices.as_ptr(),
        vertex_count: mesh.vertices.len() as u32,
        index_count: mesh.indices.len() as u32,
        name: if idx < lm.mesh_names.len() {
            lm.mesh_names[idx].as_ptr()
        } else {
            ptr::null()
        },
    }
}

// ─────────────────────────────────────────────────────────────
// Physics simulation
// ─────────────────────────────────────────────────────────────

/// Initialize physics engine with an aircraft model.
///
/// # Safety
/// `engine` and `model` must be either null or valid pointers.
/// `engine` must not be accessed concurrently during this call.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_physics_init(
    engine: *mut PhysicsEngine,
    model: *const LoadedModel,
) -> AircraftPhysicsStateC {
    // SAFETY: Caller guarantees that `engine` and `model` are either null or valid pointers.
    // `engine` must be mutable and exclusively accessed during this call.
    if engine.is_null() || model.is_null() {
        return AircraftPhysicsStateC::default();
    }
    let eng = &mut *engine;
    let lm = &*model;
    eng.state = eng.inner.init(&lm.model);
    eprintln!("[physics] init_model: {} surfaces, {} engines, mass={:.0}kg, alt={:.0}m, spd={:.0}m/s",
        eng.state.aero_surfaces.len(), eng.state.engines.len(),
        eng.state.mass_kg, eng.state.altitude_m, eng.state.airspeed_ms);
    for (i, s) in eng.state.aero_surfaces.iter().enumerate() {
        eprintln!("[physics]   surf[{}]: {} area={:.1} chord={:.1} span={:.1} a0={:.4}",
            i, s.name, s.area_m2, s.chord_m, s.span_m, s.alpha_zero_lift_rad);
        eprintln!("[physics]     pos=({:.2},{:.2},{:.2}) normal=({:.2},{:.2},{:.2})",
            s.position_local.x, s.position_local.y, s.position_local.z,
            s.normal_local.x, s.normal_local.y, s.normal_local.z);
    }
    for (i, e) in eng.state.engines.iter().enumerate() {
        eprintln!("[physics]   engine[{}]: thrust={:.0}N pos=({:.2},{:.2},{:.2}) dir=({:.2},{:.2},{:.2})",
            i, e.max_thrust_n,
            e.position_local.x, e.position_local.y, e.position_local.z,
            e.direction_local.x, e.direction_local.y, e.direction_local.z);
    }
    let fc = &eng.state.flight_chars;
    eprintln!("[physics] flight_chars: stall={:.1}m/s ref={:.1}m/s max_aoa={:.1}deg wing_area={:.1}m2",
        fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad.to_degrees(), fc.total_wing_area_m2);
    eprintln!("[physics]   ctrl_auth={:.3} wing_load={:.1}kg/m2 T/W={:.3}",
        fc.control_authority, fc.wing_loading_kg_m2, fc.thrust_to_weight);
    AircraftPhysicsStateC::from(&eng.state)
}

/// Initialize physics engine with default flying state (no model needed).
///
/// Sets up the engine at 500m altitude, 80 m/s forward velocity, 50% throttle
/// with default aerodynamic surfaces and engines.
///
/// # Safety
/// `engine` must be either null or a valid pointer to a `PhysicsEngine`.
/// `engine` must not be accessed concurrently during this call.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_physics_init_default(
    engine: *mut PhysicsEngine,
) -> AircraftPhysicsStateC {
    // SAFETY: Caller guarantees that `engine` is either null or a valid pointer.
    // `engine` must be mutable and exclusively accessed during this call.
    if engine.is_null() {
        return AircraftPhysicsStateC::default();
    }
    let eng = &mut *engine;
    let default_model = internal::AircraftModel::default();
    eng.state = eng.inner.init(&default_model);
    eprintln!("[physics] init_default: {} surfaces, {} engines, mass={:.0}kg, alt={:.0}m, spd={:.0}m/s",
        eng.state.aero_surfaces.len(), eng.state.engines.len(),
        eng.state.mass_kg, eng.state.altitude_m, eng.state.airspeed_ms);
    for (i, s) in eng.state.aero_surfaces.iter().enumerate() {
        eprintln!("[physics]   surf[{}]: {} area={:.1} a0={:.4}", i, s.name, s.area_m2, s.alpha_zero_lift_rad);
    }
    let fc = &eng.state.flight_chars;
    eprintln!("[physics] flight_chars: stall={:.1}m/s ref={:.1}m/s max_aoa={:.1}deg wing_area={:.1}m2",
        fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad.to_degrees(), fc.total_wing_area_m2);
    eprintln!("[physics]   ctrl_auth={:.3} wing_load={:.1}kg/m2 T/W={:.3}",
        fc.control_authority, fc.wing_loading_kg_m2, fc.thrust_to_weight);
    AircraftPhysicsStateC::from(&eng.state)
}

/// Initialize physics engine with a built-in aircraft profile.
///
/// Profile IDs: 0 = biplane (default), 1 = monoplane (Cessna 172), 2 = jet (F-16).
///
/// # Safety
/// `engine` must be either null or a valid pointer to a `PhysicsEngine`.
/// `engine` must not be accessed concurrently during this call.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_physics_init_profile(
    engine: *mut PhysicsEngine,
    profile_id: u32,
) -> AircraftPhysicsStateC {
    // SAFETY: Caller guarantees that `engine` is either null or a valid pointer.
    // `engine` must be mutable and exclusively accessed during this call.
    if engine.is_null() {
        return AircraftPhysicsStateC::default();
    }
    let eng = &mut *engine;
    let model = match profile_id {
        0 => internal::AircraftModel::biplane(),
        1 => internal::AircraftModel::monoplane(),
        2 => internal::AircraftModel::jet(),
        _ => internal::AircraftModel::default(),
    };
    let profile_name = match profile_id {
        0 => "biplane",
        1 => "monoplane",
        2 => "jet",
        _ => "default",
    };
    eng.state = eng.inner.init(&model);
    let fc = &eng.state.flight_chars;
    eprintln!("[physics] init_profile({}): {} surfaces, {} engines, mass={:.0}kg, spd={:.0}m/s",
        profile_name, eng.state.aero_surfaces.len(), eng.state.engines.len(),
        eng.state.mass_kg, eng.state.airspeed_ms);
    eprintln!("[physics]   stall={:.1}m/s ref={:.1}m/s max_aoa={:.1}° T/W={:.3}",
        fc.stall_speed_ms, fc.ref_speed_ms, fc.max_aoa_rad.to_degrees(), fc.thrust_to_weight);
    AircraftPhysicsStateC::from(&eng.state)
}

/// Step the physics simulation forward by `dt` seconds.
///
/// # Safety
/// `engine` and `controls` must be either null or valid pointers.
/// `engine` must not be accessed concurrently during this call.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_physics_step(
    engine: *mut PhysicsEngine,
    controls: *const ControlStateC,
    dt: f64,
) -> AircraftPhysicsStateC {
    // SAFETY: Caller guarantees that `engine` and `controls` are either null or valid pointers.
    // `engine` must be mutable and exclusively accessed during this call.
    if engine.is_null() || controls.is_null() {
        return AircraftPhysicsStateC::default();
    }
    let eng = &mut *engine;
    let ctrl: internal::ControlState = (&*controls).into();
    eng.state = eng.inner.step(&eng.state, &ctrl, dt);
    AircraftPhysicsStateC::from(&eng.state)
}

/// Get pre-computed flight characteristics for the current aircraft.
///
/// # Safety
/// `engine` must be either null or a valid pointer to a `PhysicsEngine`.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_physics_get_flight_chars(
    engine: *const PhysicsEngine,
) -> FlightCharacteristicsC {
    // SAFETY: Caller guarantees that `engine` is either null or a valid pointer.
    if engine.is_null() {
        return FlightCharacteristicsC::default();
    }
    let fc = (*engine).inner.get_flight_characteristics();
    FlightCharacteristicsC::from(&fc)
}

// ─────────────────────────────────────────────────────────────
// Camera
// ─────────────────────────────────────────────────────────────

/// Update chase camera position following the aircraft.
///
/// # Safety
/// `state` and `current_cam` must be either null or valid pointers.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_camera_update_chase(
    state: *const AircraftPhysicsStateC,
    current_cam: *const CameraStateC,
    dt: f64,
) -> CameraStateC {
    // SAFETY: Caller guarantees that `state` and `current_cam` are either null or valid pointers
    // with lifetimes extending at least through this function call.
    if state.is_null() || current_cam.is_null() {
        return CameraStateC::default();
    }
    let s = &*state;
    let orientation = glam::DMat3::from_cols_array(&s.orientation.m);

    // Forward direction (Z column of orientation)
    let forward = orientation.col(2);
    let up_world = glam::DVec3::Y;
    let pos: glam::DVec3 = s.position.into();

    let desired = pos - forward * CHASE_DISTANCE + up_world * CHASE_HEIGHT;

    // JSF-Rust SR-401: Safe access to mutable global state via Mutex
    let mut chase_pos = CHASE_POS.lock().unwrap();

    // Snap to desired position if camera is too far away (e.g. first frame or mode switch)
    let dx = desired.x - chase_pos[0];
    let dy = desired.y - chase_pos[1];
    let dz = desired.z - chase_pos[2];
    let dist_sq = dx * dx + dy * dy + dz * dz;
    let snap_threshold_sq = (CHASE_DISTANCE * 3.0) * (CHASE_DISTANCE * 3.0);

    if dist_sq > snap_threshold_sq {
        chase_pos[0] = desired.x;
        chase_pos[1] = desired.y;
        chase_pos[2] = desired.z;
    } else {
        let alpha = 1.0 - (-CHASE_SMOOTHING * dt).exp();
        chase_pos[0] += alpha * dx;
        chase_pos[1] += alpha * dy;
        chase_pos[2] += alpha * dz;
    }

    // Clamp: camera must stay above aircraft position
    let min_y = pos.y + CHASE_HEIGHT * 0.3;
    if chase_pos[1] < min_y {
        chase_pos[1] = min_y;
    }

    // Look ahead of the aircraft so camera follows the flight direction
    let look_ahead = pos + forward * 10.0;

    CameraStateC {
        position: Vec3C {
            x: chase_pos[0],
            y: chase_pos[1],
            z: chase_pos[2],
        },
        target: look_ahead.into(),
        up: Vec3C { x: 0.0, y: 1.0, z: 0.0 },
        fov_deg: (*current_cam).fov_deg,
        near: (*current_cam).near,
        far: (*current_cam).far,
        mode: 0,
    }
}

/// Update cockpit camera position inside the aircraft.
///
/// # Safety
/// `state` must be either null or a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_camera_update_cockpit(
    state: *const AircraftPhysicsStateC,
) -> CameraStateC {
    // SAFETY: Caller guarantees that `state` is either null or a valid pointer.
    if state.is_null() {
        return CameraStateC::default();
    }
    let s = &*state;
    let orientation = glam::DMat3::from_cols_array(&s.orientation.m);
    let pos: glam::DVec3 = s.position.into();

    let offset = glam::DVec3::new(COCKPIT_OFFSET[0], COCKPIT_OFFSET[1], COCKPIT_OFFSET[2]);
    let cam_pos = pos + orientation * offset;
    let forward = orientation.col(2);
    let target = cam_pos + forward * 100.0;
    let up = orientation.col(1);

    CameraStateC {
        position: cam_pos.into(),
        target: target.into(),
        up: up.into(),
        fov_deg: 75.0,
        near: 0.1,
        // JSF-Rust SR-505: Add underscores to large numeric literals for readability
        far: 100_000.0,
        mode: 1,
    }
}

// ─────────────────────────────────────────────────────────────
// Matrix builders
// ─────────────────────────────────────────────────────────────

/// Build OpenGL view matrix from camera state.
///
/// # Safety
/// `camera` must be either null or a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_build_view_matrix(
    camera: *const CameraStateC,
) -> Mat4C {
    // SAFETY: Caller guarantees that `camera` is either null or a valid pointer.
    if camera.is_null() {
        return Mat4C::default();
    }
    let c = &*camera;
    let eye: glam::DVec3 = c.position.into();
    let target: glam::DVec3 = c.target.into();
    let up: glam::DVec3 = c.up.into();
    let view = look_at(eye, target, up);
    Mat4C { m: dmat4_to_f32_array(view) }
}

/// Build OpenGL perspective projection matrix.
///
/// # Safety
/// `camera` must be either null or a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_build_projection_matrix(
    camera: *const CameraStateC,
    aspect: f64,
) -> Mat4C {
    // SAFETY: Caller guarantees that `camera` is either null or a valid pointer.
    if camera.is_null() {
        return Mat4C::default();
    }
    let c = &*camera;
    let proj = perspective(c.fov_deg, aspect, c.near, c.far);
    Mat4C { m: dmat4_to_f32_array(proj) }
}

/// Build OpenGL model matrix from orientation and position.
///
/// # Safety
/// `orientation` and `position` must be either null or valid pointers.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_build_model_matrix(
    orientation: *const Mat3C,
    position: *const Vec3C,
) -> Mat4C {
    // SAFETY: Caller guarantees that `orientation` and `position` are either null or valid pointers.
    if orientation.is_null() || position.is_null() {
        return Mat4C::default();
    }
    let rot = glam::DMat3::from_cols_array(&(*orientation).m);
    let pos: glam::DVec3 = (*position).into();
    let model = mat3_to_model_matrix(rot, pos);
    Mat4C { m: dmat4_to_f32_array(model) }
}

// ─────────────────────────────────────────────────────────────
// 무장 시스템 FFI
// ─────────────────────────────────────────────────────────────

/// 기총 수렴 거리 (m) — 양쪽 기총의 발사선이 기축 중심선 상에서 교차하는 거리.
///
/// WW2 전투기 표준: 200~400m. 오빗 카메라(~20m 후방)에서 트레이서가
/// 초록색 십자선을 향해 수렴하는 시각적 효과를 만든다.
const CONVERGENCE_DISTANCE_M: f64 = 400.0;

/// 중력 가속도 (m/s²) — superelevation 계산용
const G_ACCEL: f64 = 9.80665;

/// 프리셋에 해당하는 기총 스펙 2정(좌우 대칭)을 생성한다.
///
/// 수렴각(convergence) + 중력 보정(superelevation)을 적용하여
/// 두 기총의 탄도가 CONVERGENCE_DISTANCE_M 거리에서 기축 중심선과 교차한다.
///
/// 수렴 기하학:
///   - 좌측 기총 포구: (-0.5, 0, Z_muzzle)
///   - 우측 기총 포구: (+0.5, 0, Z_muzzle)
///   - 수렴 지점: (0, drop, Z_muzzle + D_conv)
///   - 각 기총 방향 = normalize(수렴점 - 포구 위치)
///
/// 중력 보정 (superelevation):
///   t_flight = D_conv / V_muzzle
///   drop = 0.5 × g × t²
///   발사 방향의 Y 성분을 drop만큼 상향 보정
fn guns_from_preset(preset: GunPresetC) -> Vec<aeroblend_ballistics::GunSpec> {
    let make_gun = match preset {
        GunPresetC::BrowningM2 => aeroblend_ballistics::m2_browning_gun,
        GunPresetC::AnM2Cannon => aeroblend_ballistics::an_m2_20mm_gun,
        GunPresetC::Vickers77 => aeroblend_ballistics::vickers_7_7mm_gun,
    };
    let mut left = make_gun();
    let mut right = make_gun();

    // 좌우 대칭 배치 (X축 기준 ±0.5m)
    let gun_offset_x = 0.5;
    left.muzzle_position_local.x = -gun_offset_x;
    right.muzzle_position_local.x = gun_offset_x;

    // 중력 보정: 탄환이 수렴 거리까지 날아가는 동안의 낙하량
    let muzzle_vel = left.projectile_type.muzzle_velocity_ms;
    let t_flight = CONVERGENCE_DISTANCE_M / muzzle_vel;
    let gravity_drop = 0.5 * G_ACCEL * t_flight * t_flight;

    // 수렴 방향 계산 (포구 위치 → 수렴점)
    // 수렴점: 기축 중심선(X=0) 상, 포구 Z 위치에서 D_conv 전방, 중력 보정 Y만큼 위
    // 좌측 포구(-0.5, 0, Z) → 수렴점(0, drop, Z+D) → 방향 = (+0.5, drop, D)
    // 우측 포구(+0.5, 0, Z) → 수렴점(0, drop, Z+D) → 방향 = (-0.5, drop, D)
    let left_dir = glam::DVec3::new(gun_offset_x, gravity_drop, CONVERGENCE_DISTANCE_M).normalize();
    let right_dir = glam::DVec3::new(-gun_offset_x, gravity_drop, CONVERGENCE_DISTANCE_M).normalize();

    left.muzzle_direction_local = left_dir;
    right.muzzle_direction_local = right_dir;

    vec![left, right]
}

/// 무장 시스템 생성 — 프리셋별 기본 기총 2정 (좌우 대칭).
///
/// # Returns
/// 반드시 `aeroblend_weapons_destroy`로 해제해야 하는 핸들 포인터.
#[no_mangle]
pub extern "C" fn aeroblend_weapons_create(preset: GunPresetC) -> *mut WeaponSystemHandle {
    let guns = guns_from_preset(preset);
    let system = aeroblend_ballistics::WeaponSystem::new(guns);
    Box::into_raw(Box::new(WeaponSystemHandle { inner: system, preset }))
}

/// 무장 시스템 해제.
///
/// # Safety
/// `weapons`는 null이거나 `aeroblend_weapons_create`가 반환한 유효한 포인터여야 한다.
/// 동일 포인터로 두 번 이상 호출하면 안 된다.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_weapons_destroy(weapons: *mut WeaponSystemHandle) {
    // SAFETY: 호출자가 weapons 포인터 유효성을 보장한다.
    if !weapons.is_null() {
        drop(Box::from_raw(weapons));
    }
}

/// 무장 시스템 업데이트 — 매 물리 프레임 호출 (120Hz).
///
/// 항공기 상태에서 위치/자세/속도를 추출하고, firing 플래그에 따라
/// 기총 발사 및 탄환 물리를 한 스텝 진행한다.
///
/// # Safety
/// `weapons`와 `aircraft_state`는 null이거나 유효한 포인터여야 한다.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_weapons_update(
    weapons: *mut WeaponSystemHandle,
    aircraft_state: *const AircraftPhysicsStateC,
    firing: u8,
    dt: f64,
) -> WeaponStateC {
    // SAFETY: 호출자가 포인터 유효성을 보장한다.
    if weapons.is_null() || aircraft_state.is_null() {
        return WeaponStateC::default();
    }
    let ws = &mut (*weapons);
    let state = &*aircraft_state;

    // 항공기 상태에서 위치/자세/속도 추출
    let position = glam::DVec3::new(state.position.x, state.position.y, state.position.z);
    let velocity = glam::DVec3::new(state.velocity.x, state.velocity.y, state.velocity.z);

    // 회전 행렬 변환 (column-major Mat3C → DMat3)
    let orientation = glam::DMat3::from_cols_array(&state.orientation.m);

    // 발사 상태 설정
    for gun in &mut ws.inner.guns {
        gun.firing = firing != 0;
    }

    // 업데이트
    ws.inner.update(dt, position, orientation, velocity);

    // 상태 반환
    let total_remaining: u32 = ws.inner.guns.iter().map(|g| g.ammo_remaining).sum();
    let total_capacity: u32 = ws.inner.guns.iter().map(|g| g.spec.ammo).sum();
    WeaponStateC {
        gun_count: ws.inner.guns.len() as u32,
        total_ammo_remaining: total_remaining,
        total_ammo_capacity: total_capacity,
        is_firing: if ws.inner.guns.iter().any(|g| g.firing && g.ammo_remaining > 0) { 1 } else { 0 },
        gun_preset: ws.preset,
    }
}

/// 활성 탄환 목록 복사 — 렌더링용.
///
/// `out_buffer`에 최대 `buffer_size`개의 활성 탄환 상태를 기록하고,
/// 실제 기록된 수를 반환한다.
///
/// # Safety
/// `weapons`는 null이거나 유효한 포인터여야 한다.
/// `out_buffer`는 최소 `buffer_size` 크기의 배열을 가리켜야 한다.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_weapons_get_projectiles(
    weapons: *const WeaponSystemHandle,
    out_buffer: *mut ProjectileStateC,
    buffer_size: u32,
) -> u32 {
    // SAFETY: 호출자가 포인터 유효성과 버퍼 크기를 보장한다.
    if weapons.is_null() || out_buffer.is_null() {
        return 0;
    }
    let ws = &*weapons;
    let mut count = 0u32;
    for p in &ws.inner.projectiles {
        if count >= buffer_size {
            break;
        }
        if !p.active {
            continue;
        }
        let out = &mut *out_buffer.add(count as usize);
        out.position = Vec3C { x: p.position.x, y: p.position.y, z: p.position.z };
        out.velocity = Vec3C { x: p.velocity.x, y: p.velocity.y, z: p.velocity.z };
        out.active = 1;
        count += 1;
    }
    count
}

/// 전체 재장전 — 모든 기총의 잔탄을 최대치로 복원.
///
/// # Safety
/// `weapons`는 null이거나 유효한 포인터여야 한다.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_weapons_reload(weapons: *mut WeaponSystemHandle) {
    // SAFETY: 호출자가 포인터 유효성을 보장한다.
    if !weapons.is_null() {
        let ws = &mut *weapons;
        for gun in &mut ws.inner.guns {
            gun.ammo_remaining = gun.spec.ammo;
        }
    }
}

/// 잔탄수 조회 — 전체 기총의 합산 잔탄수를 반환.
///
/// # Safety
/// `weapons`는 null이거나 유효한 포인터여야 한다.
#[no_mangle]
pub unsafe extern "C" fn aeroblend_weapons_get_ammo(weapons: *const WeaponSystemHandle) -> u32 {
    // SAFETY: 호출자가 포인터 유효성을 보장한다.
    if weapons.is_null() {
        return 0;
    }
    let ws = &*weapons;
    ws.inner.guns.iter().map(|g| g.ammo_remaining).sum()
}

// ─────────────────────────────────────────────────────────────
// 테스트
// ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use aeroblend_core::GunPresetC;

    /// 좌우 기총 수렴각 기하학 검증.
    ///
    /// 양쪽 기총의 발사선이 CONVERGENCE_DISTANCE_M 지점에서
    /// 기축 중심선(X=0)에서 만나는지 확인한다.
    #[test]
    fn test_gun_convergence_geometry() {
        let guns = guns_from_preset(GunPresetC::BrowningM2);
        assert_eq!(guns.len(), 2, "좌우 기총 2정이어야 함");

        let left = &guns[0];
        let right = &guns[1];

        // 포구 위치 확인
        assert!(
            (left.muzzle_position_local.x - (-0.5)).abs() < 1e-9,
            "좌측 포구 X = -0.5m: 실제={:.4}",
            left.muzzle_position_local.x
        );
        assert!(
            (right.muzzle_position_local.x - 0.5).abs() < 1e-9,
            "우측 포구 X = +0.5m: 실제={:.4}",
            right.muzzle_position_local.x
        );

        // 발사 방향이 정규화되어 있는지 확인
        let left_len = left.muzzle_direction_local.length();
        let right_len = right.muzzle_direction_local.length();
        assert!(
            (left_len - 1.0).abs() < 1e-9,
            "좌측 발사 방향이 정규화되지 않음: |d|={:.6}",
            left_len
        );
        assert!(
            (right_len - 1.0).abs() < 1e-9,
            "우측 발사 방향이 정규화되지 않음: |d|={:.6}",
            right_len
        );

        // 수렴점 계산: 각 포구에서 발사선을 CONVERGENCE_DISTANCE_M / dir.z 거리 연장
        // dir.z ≈ 1.0 (거의 전방) 이므로 t ≈ D_conv / dir_z
        let left_pos = left.muzzle_position_local;
        let left_dir = left.muzzle_direction_local;
        let t_left = CONVERGENCE_DISTANCE_M / left_dir.z;
        let conv_left_x = left_pos.x + left_dir.x * t_left;

        let right_pos = right.muzzle_position_local;
        let right_dir = right.muzzle_direction_local;
        let t_right = CONVERGENCE_DISTANCE_M / right_dir.z;
        let conv_right_x = right_pos.x + right_dir.x * t_right;

        // 두 발사선이 X=0 근방에서 교차해야 함 (0.5m 허용 오차)
        assert!(
            conv_left_x.abs() < 0.5,
            "좌측 발사선이 X=0에서 수렴하지 않음: X={:.4}m at D={}m",
            conv_left_x,
            CONVERGENCE_DISTANCE_M
        );
        assert!(
            conv_right_x.abs() < 0.5,
            "우측 발사선이 X=0에서 수렴하지 않음: X={:.4}m at D={}m",
            conv_right_x,
            CONVERGENCE_DISTANCE_M
        );

        // 좌우 수렴점이 서로 일치해야 함 (대칭성 확인)
        let y_left = left_pos.y + left_dir.y * t_left;
        let y_right = right_pos.y + right_dir.y * t_right;
        assert!(
            (y_left - y_right).abs() < 0.01,
            "좌우 수렴점 Y가 다름: left={:.4} right={:.4}",
            y_left,
            y_right
        );

        // 중력 보정 방향이 양수(위쪽)인지 확인 — 탄환 낙하 보정
        assert!(
            left_dir.y > 0.0,
            "좌측 발사 방향 Y(중력 보정)이 양수여야 함: Y={:.6}",
            left_dir.y
        );
        assert!(
            right_dir.y > 0.0,
            "우측 발사 방향 Y(중력 보정)이 양수여야 함: Y={:.6}",
            right_dir.y
        );

        // 주 발사 방향이 전방(+Z)인지 확인 — Z가 압도적으로 커야 함
        assert!(
            left_dir.z > 0.999,
            "좌측 발사 방향이 거의 전방이어야 함: Z={:.6}",
            left_dir.z
        );
        assert!(
            right_dir.z > 0.999,
            "우측 발사 방향이 거의 전방이어야 함: Z={:.6}",
            right_dir.z
        );
    }

    /// 20mm 기관포 프리셋 수렴각 검증 — 초속이 달라도 수렴 기하학이 성립하는지 확인.
    #[test]
    fn test_gun_convergence_20mm() {
        let guns = guns_from_preset(GunPresetC::AnM2Cannon);
        assert_eq!(guns.len(), 2, "좌우 기총 2정이어야 함");

        let left = &guns[0];
        let right = &guns[1];
        let left_dir = left.muzzle_direction_local;
        let right_dir = right.muzzle_direction_local;

        // 발사선 수렴점 X 좌표가 0에 가까워야 함
        let t_left = CONVERGENCE_DISTANCE_M / left_dir.z;
        let t_right = CONVERGENCE_DISTANCE_M / right_dir.z;
        let conv_left_x = left.muzzle_position_local.x + left_dir.x * t_left;
        let conv_right_x = right.muzzle_position_local.x + right_dir.x * t_right;

        assert!(
            conv_left_x.abs() < 0.5,
            "20mm 좌측 수렴 X={:.4}m",
            conv_left_x
        );
        assert!(
            conv_right_x.abs() < 0.5,
            "20mm 우측 수렴 X={:.4}m",
            conv_right_x
        );
    }

    /// 좌우 기총 방향의 X 성분 대칭성 확인 — 기총 대칭 배치의 핵심 요건.
    #[test]
    fn test_gun_direction_symmetry() {
        for preset in [GunPresetC::BrowningM2, GunPresetC::AnM2Cannon, GunPresetC::Vickers77] {
            let guns = guns_from_preset(preset);
            let left_dir = guns[0].muzzle_direction_local;
            let right_dir = guns[1].muzzle_direction_local;

            // X 성분은 크기는 같고 부호가 반대여야 함
            assert!(
                (left_dir.x + right_dir.x).abs() < 1e-9,
                "좌우 기총 X 방향 성분이 대칭이 아님: left.x={:.6}, right.x={:.6}",
                left_dir.x,
                right_dir.x
            );

            // Y, Z 성분은 동일해야 함 (동일한 고도/전방)
            assert!(
                (left_dir.y - right_dir.y).abs() < 1e-9,
                "좌우 기총 Y 방향 성분이 다름: left.y={:.6}, right.y={:.6}",
                left_dir.y,
                right_dir.y
            );
            assert!(
                (left_dir.z - right_dir.z).abs() < 1e-9,
                "좌우 기총 Z 방향 성분이 다름: left.z={:.6}, right.z={:.6}",
                left_dir.z,
                right_dir.z
            );
        }
    }
}
