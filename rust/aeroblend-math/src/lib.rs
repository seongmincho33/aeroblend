//! `AeroBlend` math library.
//!
//! Provides `Vec3` (f64), `Quat` (f64), `Mat3` (f64), `Mat4f` (f32 for OpenGL),
//! quaternion operations, Euler angle conversions, and matrix builders.
//!
//! Coordinate system: right-handed, Y-up.
//!   +X = right, +Y = up, +Z = forward
//!   Angles in radians unless stated otherwise.

// JSF-Rust SR-801: Enable clippy lints for code quality
#![warn(clippy::all, clippy::pedantic)]
#![allow(clippy::similar_names)] // allow q1, q2 etc.
#![allow(clippy::many_single_char_names)] // allow x, y, z etc.

// JSF-Rust SR-803: Forbid unsafe code in non-FFI crates
#![forbid(unsafe_code)]

use glam::{DMat3, DMat4, DQuat, DVec3};

// Re-export glam types we use
pub use glam::{DMat3 as Mat3, DVec3 as Vec3d, DQuat as Quatd};

// ───────────────────────────────────────────────────────────────────
// Quaternion utilities (matching Python's [w, x, y, z] convention)
// glam uses [x, y, z, w] internally, so we provide conversion helpers.
// ───────────────────────────────────────────────────────────────────

/// Create identity quaternion.
// JSF-Rust SR-804: Pure function result should not be ignored
#[inline]
#[must_use]
pub fn quat_identity() -> DQuat {
    DQuat::IDENTITY
}

/// Normalize a quaternion. Returns identity if near-zero.
#[inline]
#[must_use]
pub fn quat_normalize(q: DQuat) -> DQuat {
    let n = q.length();
    if n < 1e-12 {
        DQuat::IDENTITY
    } else {
        q / n
    }
}

/// Hamilton product of two quaternions.
#[inline]
#[must_use]
pub fn quat_multiply(q1: DQuat, q2: DQuat) -> DQuat {
    q1 * q2
}

/// Quaternion conjugate (inverse for unit quaternion).
#[inline]
#[must_use]
pub fn quat_conjugate(q: DQuat) -> DQuat {
    q.conjugate()
}

/// Rotate vector v by quaternion q: q * v * q^-1.
#[inline]
#[must_use]
pub fn quat_rotate_vector(q: DQuat, v: DVec3) -> DVec3 {
    q * v
}

/// Convert unit quaternion to 3x3 rotation matrix (body -> world).
#[must_use]
pub fn quat_to_rotation_matrix(q: DQuat) -> DMat3 {
    let q = q.normalize();
    DMat3::from_quat(q)
}

/// Convert 3x3 rotation matrix to quaternion.
/// Uses glam's built-in conversion (Shepperd's method).
#[must_use]
pub fn rotation_matrix_to_quat(m: DMat3) -> DQuat {
    DQuat::from_mat3(&m).normalize()
}

/// Create quaternion from Euler angles (roll, pitch, yaw) in radians.
///
/// Rotation order: yaw (Y) -> pitch (X) -> roll (Z) for Y-up right-handed.
/// Axes: roll=Z, pitch=X, yaw=Y.
#[must_use]
pub fn quat_from_euler(roll: f64, pitch: f64, yaw: f64) -> DQuat {
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();

    // [x, y, z, w] in glam's convention
    // From Python: w = cy*cp*cr + sy*sp*sr, x = cy*sp*cr + sy*cp*sr,
    //              y = sy*cp*cr - cy*sp*sr, z = cy*cp*sr - sy*sp*cr
    DQuat::from_xyzw(
        cy * sp * cr + sy * cp * sr, // x
        sy * cp * cr - cy * sp * sr, // y
        cy * cp * sr - sy * sp * cr, // z
        cy * cp * cr + sy * sp * sr, // w
    )
}

/// Extract Euler angles (roll, pitch, yaw) from quaternion.
///
/// Returns (roll, pitch, yaw) in radians.
/// Axes: roll=Z, pitch=X, yaw=Y (Y-up right-handed, intrinsic Y*X*Z).
#[must_use]
pub fn quat_to_euler(q: DQuat) -> (f64, f64, f64) {
    let q = q.normalize();
    let (x, y, z, w) = (q.x, q.y, q.z, q.w);

    // Pitch (X-axis rotation)
    let sinp = 2.0 * (w * x - y * z);
    let sinp = sinp.clamp(-1.0, 1.0);
    let pitch = sinp.asin();

    // Roll (Z-axis rotation)
    let roll = (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (x * x + z * z));

    // Yaw (Y-axis rotation)
    let yaw = (2.0 * (x * z + w * y)).atan2(1.0 - 2.0 * (x * x + y * y));

    (roll, pitch, yaw)
}

/// Quaternion time derivative from body angular velocity.
/// `dq/dt = 0.5 * q * omega_quat`, where `omega_quat = [omega_x, omega_y, omega_z, 0]`
#[must_use]
pub fn quat_derivative(q: DQuat, omega_body: DVec3) -> DQuat {
    let omega_q = DQuat::from_xyzw(omega_body.x, omega_body.y, omega_body.z, 0.0);
    let result = q * omega_q;
    DQuat::from_xyzw(
        result.x * 0.5,
        result.y * 0.5,
        result.z * 0.5,
        result.w * 0.5,
    )
}

// ───────────────────────────────────────────────────────────────────
// Matrix builders
// ───────────────────────────────────────────────────────────────────

/// Build 4x4 model matrix from 3x3 rotation + position (f64).
#[must_use]
pub fn mat3_to_model_matrix(rotation: DMat3, position: DVec3) -> DMat4 {
    let cols = rotation.to_cols_array_2d();
    DMat4::from_cols_array_2d(&[
        [cols[0][0], cols[0][1], cols[0][2], 0.0],
        [cols[1][0], cols[1][1], cols[1][2], 0.0],
        [cols[2][0], cols[2][1], cols[2][2], 0.0],
        [position.x, position.y, position.z, 1.0],
    ])
}

/// Build right-handed look-at view matrix (OpenGL convention).
#[must_use]
pub fn look_at(eye: DVec3, target: DVec3, up: DVec3) -> DMat4 {
    let f = target - eye;
    let f_len = f.length();
    if f_len < 1e-9 {
        return DMat4::IDENTITY;
    }
    let f = f / f_len;

    let s = f.cross(up);
    let s_len = s.length();
    let s = if s_len < 1e-9 {
        DVec3::new(1.0, 0.0, 0.0)
    } else {
        s / s_len
    };

    let u = s.cross(f);

    DMat4::from_cols_array_2d(&[
        [s.x, u.x, -f.x, 0.0],
        [s.y, u.y, -f.y, 0.0],
        [s.z, u.z, -f.z, 0.0],
        [-s.dot(eye), -u.dot(eye), f.dot(eye), 1.0],
    ])
}

/// Build perspective projection matrix (OpenGL convention).
#[must_use]
pub fn perspective(fov_deg: f64, aspect: f64, near: f64, far: f64) -> DMat4 {
    let fov_rad = fov_deg.to_radians();
    let t = (fov_rad / 2.0).tan();

    DMat4::from_cols_array_2d(&[
        [1.0 / (aspect * t), 0.0, 0.0, 0.0],
        [0.0, 1.0 / t, 0.0, 0.0],
        [0.0, 0.0, -(far + near) / (far - near), -1.0],
        [0.0, 0.0, -(2.0 * far * near) / (far - near), 0.0],
    ])
}

// ───────────────────────────────────────────────────────────────────
// f32 conversions for OpenGL (Mat4_C)
// ───────────────────────────────────────────────────────────────────

/// Convert `DMat4` (f64) to `Mat4` (f32) column-major array for OpenGL.
#[inline]
#[must_use]
pub fn dmat4_to_f32_array(m: DMat4) -> [f32; 16] {
    let cols = m.to_cols_array();
    let mut out = [0.0f32; 16];
    for i in 0..16 {
        // JSF-Rust SR-501: Lossy f64 -> f32 conversion, precision loss acceptable for OpenGL
        #[allow(clippy::cast_possible_truncation)]
        {
            out[i] = cols[i] as f32;
        }
    }
    out
}

/// Convert `DMat3` (f64) to f64 column-major array.
#[inline]
#[must_use]
pub fn dmat3_to_f64_array(m: DMat3) -> [f64; 9] {
    m.to_cols_array()
}

// ───────────────────────────────────────────────────────────────────
// Body/World coordinate transforms
// ───────────────────────────────────────────────────────────────────

/// Transform vector from body frame to world frame.
#[inline]
#[must_use]
pub fn body_to_world(v_body: DVec3, orientation_q: DQuat) -> DVec3 {
    quat_rotate_vector(orientation_q, v_body)
}

/// Transform vector from world frame to body frame.
#[inline]
#[must_use]
pub fn world_to_body(v_world: DVec3, orientation_q: DQuat) -> DVec3 {
    quat_rotate_vector(quat_conjugate(orientation_q), v_world)
}

// ───────────────────────────────────────────────────────────────────
// Constants
// ───────────────────────────────────────────────────────────────────

pub const G: f64 = 9.80665;
pub const PI: f64 = std::f64::consts::PI;
pub const TWO_PI: f64 = 2.0 * PI;

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    #[test]
    fn test_quat_identity() {
        let q = quat_identity();
        assert!((q.w - 1.0).abs() < EPS);
        assert!(q.x.abs() < EPS);
        assert!(q.y.abs() < EPS);
        assert!(q.z.abs() < EPS);
    }

    #[test]
    fn test_quat_rotate_vector_identity() {
        let q = quat_identity();
        let v = DVec3::new(1.0, 2.0, 3.0);
        let rotated = quat_rotate_vector(q, v);
        assert!((rotated.x - 1.0).abs() < EPS);
        assert!((rotated.y - 2.0).abs() < EPS);
        assert!((rotated.z - 3.0).abs() < EPS);
    }

    #[test]
    fn test_quat_to_rotation_matrix_identity() {
        let q = quat_identity();
        let m = quat_to_rotation_matrix(q);
        let expected = DMat3::IDENTITY;
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (m.col(i)[j] - expected.col(i)[j]).abs() < EPS,
                    "Mismatch at ({i}, {j})"
                );
            }
        }
    }

    #[test]
    fn test_euler_roundtrip() {
        let roll = 0.3;
        let pitch = 0.2;
        let yaw = 0.5;
        let q = quat_from_euler(roll, pitch, yaw);
        let (r2, p2, y2) = quat_to_euler(q);
        assert!((r2 - roll).abs() < 1e-10, "roll: {r2} vs {roll}");
        assert!((p2 - pitch).abs() < 1e-10, "pitch: {p2} vs {pitch}");
        assert!((y2 - yaw).abs() < 1e-10, "yaw: {y2} vs {yaw}");
    }

    #[test]
    fn test_rotation_matrix_roundtrip() {
        let q_orig = quat_from_euler(0.1, 0.2, 0.3);
        let m = quat_to_rotation_matrix(q_orig);
        let q_back = rotation_matrix_to_quat(m);
        // Quaternions q and -q represent same rotation
        let dot = q_orig.x * q_back.x + q_orig.y * q_back.y
            + q_orig.z * q_back.z + q_orig.w * q_back.w;
        assert!(dot.abs() > 1.0 - 1e-10, "dot = {dot}");
    }

    #[test]
    fn test_look_at_basic() {
        let eye = DVec3::new(0.0, 0.0, -10.0);
        let target = DVec3::ZERO;
        let up = DVec3::Y;
        let view = look_at(eye, target, up);
        // Should not be identity
        assert!(!(view - DMat4::IDENTITY).abs_diff_eq(DMat4::ZERO, 1e-9));
    }

    #[test]
    fn test_perspective_basic() {
        // JSF-Rust SR-505: Add underscores to large numeric literals for readability
        let proj = perspective(75.0, 16.0 / 9.0, 0.1, 100_000.0);
        // [0][0] should be 1 / (aspect * tan(fov/2))
        let fov_rad = 75.0f64.to_radians();
        let t = (fov_rad / 2.0).tan();
        let expected_00 = 1.0 / ((16.0 / 9.0) * t);
        assert!((proj.col(0).x - expected_00).abs() < 1e-10);
    }

    #[test]
    fn test_model_matrix() {
        let rotation = DMat3::IDENTITY;
        let position = DVec3::new(10.0, 20.0, 30.0);
        let model = mat3_to_model_matrix(rotation, position);
        assert!((model.col(3).x - 10.0).abs() < EPS);
        assert!((model.col(3).y - 20.0).abs() < EPS);
        assert!((model.col(3).z - 30.0).abs() < EPS);
        assert!((model.col(3).w - 1.0).abs() < EPS);
    }

    #[test]
    fn test_body_world_roundtrip() {
        let q = quat_from_euler(0.5, 0.3, 0.7);
        let v = DVec3::new(1.0, 2.0, 3.0);
        let v_world = body_to_world(v, q);
        let v_back = world_to_body(v_world, q);
        assert!((v_back.x - v.x).abs() < 1e-10);
        assert!((v_back.y - v.y).abs() < 1e-10);
        assert!((v_back.z - v.z).abs() < 1e-10);
    }

    #[test]
    fn test_quat_derivative() {
        let q = quat_identity();
        let omega = DVec3::new(0.0, 0.0, 1.0); // rotation about Z
        let dq = quat_derivative(q, omega);
        // dq/dt = 0.5 * q * [0, omega] for identity q, should be [0, 0, 0.5, 0]
        // w component: 0.5 * (1*0 - 0*0 - 0*0 - 0*1) = 0
        // z component: 0.5 * (1*1 + 0*0 - 0*0 + 0*0) = 0.5
        assert!(dq.w.abs() < EPS);
        assert!(dq.x.abs() < EPS);
        assert!(dq.y.abs() < EPS);
        assert!((dq.z - 0.5).abs() < EPS);
    }
}
