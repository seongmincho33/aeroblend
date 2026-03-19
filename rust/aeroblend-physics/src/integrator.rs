//! Numerical integrators for the 6DoF physics simulation.
//!
//! Provides RK4 (Runge-Kutta 4th order) integration for both translational
//! and rotational dynamics with quaternion orientation.

use glam::{DQuat, DVec3};

use aeroblend_math::quat_normalize;
use crate::types::PhysicsState6DoF;

/// Time derivative of the 6DoF state.
#[derive(Debug, Clone)]
pub struct StateDerivative {
    pub d_position: DVec3,
    pub d_velocity: DVec3,
    pub d_orientation_q: DQuat,
    pub d_angular_velocity: DVec3,
}

/// Apply derivative to state: state + deriv * dt.
fn apply_derivative(
    state: &PhysicsState6DoF,
    deriv: &StateDerivative,
    dt: f64,
) -> PhysicsState6DoF {
    let mut new = state.clone();
    new.position = state.position + deriv.d_position * dt;
    new.velocity = state.velocity + deriv.d_velocity * dt;

    let dq = DQuat::from_xyzw(
        state.orientation_q.x + deriv.d_orientation_q.x * dt,
        state.orientation_q.y + deriv.d_orientation_q.y * dt,
        state.orientation_q.z + deriv.d_orientation_q.z * dt,
        state.orientation_q.w + deriv.d_orientation_q.w * dt,
    );
    new.orientation_q = quat_normalize(dq);
    new.angular_velocity = state.angular_velocity + deriv.d_angular_velocity * dt;
    new
}

/// Scale all derivative components by scalar.
fn scale_derivative(d: &StateDerivative, s: f64) -> StateDerivative {
    StateDerivative {
        d_position: d.d_position * s,
        d_velocity: d.d_velocity * s,
        d_orientation_q: DQuat::from_xyzw(
            d.d_orientation_q.x * s,
            d.d_orientation_q.y * s,
            d.d_orientation_q.z * s,
            d.d_orientation_q.w * s,
        ),
        d_angular_velocity: d.d_angular_velocity * s,
    }
}

/// Add two derivatives element-wise.
fn add_derivatives(a: &StateDerivative, b: &StateDerivative) -> StateDerivative {
    StateDerivative {
        d_position: a.d_position + b.d_position,
        d_velocity: a.d_velocity + b.d_velocity,
        d_orientation_q: DQuat::from_xyzw(
            a.d_orientation_q.x + b.d_orientation_q.x,
            a.d_orientation_q.y + b.d_orientation_q.y,
            a.d_orientation_q.z + b.d_orientation_q.z,
            a.d_orientation_q.w + b.d_orientation_q.w,
        ),
        d_angular_velocity: a.d_angular_velocity + b.d_angular_velocity,
    }
}

/// 4th-order Runge-Kutta integration step.
pub fn integrate_rk4<F>(
    state: &PhysicsState6DoF,
    t: f64,
    dt: f64,
    derivative_func: &F,
) -> PhysicsState6DoF
where
    F: Fn(&PhysicsState6DoF, f64) -> StateDerivative,
{
    // k1
    let k1 = derivative_func(state, t);

    // k2
    let s2 = apply_derivative(state, &k1, dt * 0.5);
    let k2 = derivative_func(&s2, t + dt * 0.5);

    // k3
    let s3 = apply_derivative(state, &k2, dt * 0.5);
    let k3 = derivative_func(&s3, t + dt * 0.5);

    // k4
    let s4 = apply_derivative(state, &k3, dt);
    let k4 = derivative_func(&s4, t + dt);

    // Weighted average: (k1 + 2*k2 + 2*k3 + k4) / 6
    let weighted = add_derivatives(
        &add_derivatives(&k1, &scale_derivative(&k2, 2.0)),
        &add_derivatives(&scale_derivative(&k3, 2.0), &k4),
    );
    let weighted = scale_derivative(&weighted, 1.0 / 6.0);

    let mut result = apply_derivative(state, &weighted, dt);
    result.orientation_q = quat_normalize(result.orientation_q);
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rk4_constant_velocity() {
        // Simple test: constant velocity, no forces
        let state = PhysicsState6DoF {
            velocity: DVec3::new(0.0, 0.0, 10.0),
            ..Default::default()
        };

        let deriv_fn = |s: &PhysicsState6DoF, _t: f64| StateDerivative {
            d_position: s.velocity,
            d_velocity: DVec3::ZERO,
            d_orientation_q: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            d_angular_velocity: DVec3::ZERO,
        };

        let result = integrate_rk4(&state, 0.0, 1.0, &deriv_fn);
        assert!((result.position.z - 10.0).abs() < 1e-10);
        assert!((result.velocity.z - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_rk4_constant_acceleration() {
        // Constant acceleration: a = [0, -9.8, 0]
        let state = PhysicsState6DoF::default();

        let deriv_fn = |s: &PhysicsState6DoF, _t: f64| StateDerivative {
            d_position: s.velocity,
            d_velocity: DVec3::new(0.0, -9.80665, 0.0),
            d_orientation_q: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            d_angular_velocity: DVec3::ZERO,
        };

        let result = integrate_rk4(&state, 0.0, 1.0, &deriv_fn);
        // v = at, s = 0.5*a*t²
        assert!((result.velocity.y - (-9.80665)).abs() < 1e-6);
        assert!((result.position.y - (-0.5 * 9.80665)).abs() < 1e-6);
    }
}
