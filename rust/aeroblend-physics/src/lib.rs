//! `AeroBlend` physics engine.
//!
//! 6DoF flight dynamics with RK4 integration, ISA atmosphere,
//! airfoil models, aerodynamic force computation, and ground collision.

// JSF-Rust SR-801: Enable clippy lints for code quality
#![warn(clippy::all, clippy::pedantic)]
#![allow(clippy::similar_names)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::too_many_lines)]
#![allow(clippy::must_use_candidate)] // too noisy for internal functions
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::wildcard_imports)] // allow for internal module imports
#![allow(clippy::float_cmp)] // acceptable in tests with known values
#![allow(clippy::too_many_arguments)] // physics functions need many parameters

// JSF-Rust SR-803: Forbid unsafe code in non-FFI crates
#![forbid(unsafe_code)]

pub mod atmosphere;
pub mod aerodynamics;
pub mod integrator;
pub mod engine;
pub mod types;
