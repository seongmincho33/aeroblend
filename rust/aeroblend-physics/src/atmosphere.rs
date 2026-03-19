//! ISA (International Standard Atmosphere) model.
//!
//! All units are SI: metres, Pascals, kg/m³, Kelvin, m/s.

/// Gravitational acceleration (m/s²).
pub const G: f64 = 9.80665;
/// Specific gas constant for dry air (J/(kg·K)).
pub const R_AIR: f64 = 287.05287;
/// Ratio of specific heats for air.
pub const GAMMA_AIR: f64 = 1.4;

/// Atmospheric properties at a given altitude.
#[derive(Debug, Clone, Copy)]
pub struct AtmosphereProperties {
    pub pressure_pa: f64,
    pub density_kg_m3: f64,
    pub temperature_k: f64,
    pub speed_of_sound_ms: f64,
}

/// International Standard Atmosphere (ISA) model.
///
/// Troposphere (0-11 km): linear temperature lapse.
/// Tropopause / lower stratosphere (11-20 km): isothermal at 216.65 K.
/// Above 20 km: clamped to 20 km values.
pub struct ISAAtmosphere {
    p_tropo: f64,
    rho_tropo: f64,
}

impl ISAAtmosphere {
    // Sea-level reference values
    const T0: f64 = 288.15;       // K
    // JSF-Rust SR-505: Add underscores to large numeric literals for readability
    const P0: f64 = 101_325.0;    // Pa
    const RHO0: f64 = 1.225;      // kg/m³
    const LAPSE_RATE: f64 = 0.0065; // K/m (troposphere)

    // Tropopause boundary
    const H_TROPOPAUSE: f64 = 11000.0; // m
    const T_TROPOPAUSE: f64 = 216.65;  // K
    const H_MAX: f64 = 20000.0;        // m

    pub fn new() -> Self {
        let exponent = G / (Self::LAPSE_RATE * R_AIR);
        let temp_ratio = Self::T_TROPOPAUSE / Self::T0;
        let p_tropo = Self::P0 * temp_ratio.powf(exponent);
        let rho_tropo = Self::RHO0 * temp_ratio.powf(exponent - 1.0);
        Self { p_tropo, rho_tropo }
    }

    /// Return atmospheric properties at the given altitude.
    pub fn get_properties(&self, altitude_m: f64) -> AtmosphereProperties {
        let h = altitude_m.clamp(0.0, Self::H_MAX);

        let (pressure, density, temperature) = if h <= Self::H_TROPOPAUSE {
            // Troposphere
            let t = Self::T0 - Self::LAPSE_RATE * h;
            let exponent = G / (Self::LAPSE_RATE * R_AIR);
            let temp_ratio = t / Self::T0;
            let p = Self::P0 * temp_ratio.powf(exponent);
            let rho = Self::RHO0 * temp_ratio.powf(exponent - 1.0);
            (p, rho, t)
        } else {
            // Lower stratosphere (isothermal)
            let t = Self::T_TROPOPAUSE;
            let dh = h - Self::H_TROPOPAUSE;
            let exp_factor = (-G * dh / (R_AIR * t)).exp();
            let p = self.p_tropo * exp_factor;
            let rho = self.rho_tropo * exp_factor;
            (p, rho, t)
        };

        let speed_of_sound = (GAMMA_AIR * R_AIR * temperature).sqrt();

        AtmosphereProperties {
            pressure_pa: pressure,
            density_kg_m3: density,
            temperature_k: temperature,
            speed_of_sound_ms: speed_of_sound,
        }
    }

    #[inline]
    pub fn density(&self, altitude_m: f64) -> f64 {
        self.get_properties(altitude_m).density_kg_m3
    }

    #[inline]
    pub fn temperature(&self, altitude_m: f64) -> f64 {
        self.get_properties(altitude_m).temperature_k
    }

    #[inline]
    pub fn pressure(&self, altitude_m: f64) -> f64 {
        self.get_properties(altitude_m).pressure_pa
    }

    #[inline]
    pub fn speed_of_sound(&self, altitude_m: f64) -> f64 {
        self.get_properties(altitude_m).speed_of_sound_ms
    }
}

impl Default for ISAAtmosphere {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sea_level() {
        let atm = ISAAtmosphere::new();
        let props = atm.get_properties(0.0);
        // JSF-Rust SR-505: Add underscores to large numeric literals for readability
        assert!((props.pressure_pa - 101_325.0).abs() < 0.1);
        assert!((props.density_kg_m3 - 1.225).abs() < 0.001);
        assert!((props.temperature_k - 288.15).abs() < 0.01);
        assert!((props.speed_of_sound_ms - 340.294).abs() < 0.1);
    }

    #[test]
    fn test_5000m() {
        let atm = ISAAtmosphere::new();
        let props = atm.get_properties(5000.0);
        // ISA at 5000m: T=255.65 K, P≈54048 Pa, rho≈0.7364 kg/m³
        assert!((props.temperature_k - 255.65).abs() < 0.01);
        assert!((props.pressure_pa - 54048.0).abs() < 50.0);
        assert!((props.density_kg_m3 - 0.7364).abs() < 0.002);
    }

    #[test]
    fn test_tropopause() {
        let atm = ISAAtmosphere::new();
        let props = atm.get_properties(11000.0);
        assert!((props.temperature_k - 216.65).abs() < 0.01);
    }

    #[test]
    fn test_stratosphere() {
        let atm = ISAAtmosphere::new();
        let props = atm.get_properties(15000.0);
        assert!((props.temperature_k - 216.65).abs() < 0.01);
        // Pressure should be less than tropopause
        let tropo = atm.get_properties(11000.0);
        assert!(props.pressure_pa < tropo.pressure_pa);
    }

    #[test]
    fn test_clamp_negative() {
        let atm = ISAAtmosphere::new();
        let props = atm.get_properties(-100.0);
        let sea = atm.get_properties(0.0);
        assert!((props.pressure_pa - sea.pressure_pa).abs() < 0.001);
    }

    #[test]
    fn test_clamp_above_max() {
        let atm = ISAAtmosphere::new();
        let props = atm.get_properties(25000.0);
        let max = atm.get_properties(20000.0);
        assert!((props.pressure_pa - max.pressure_pa).abs() < 0.001);
    }
}
