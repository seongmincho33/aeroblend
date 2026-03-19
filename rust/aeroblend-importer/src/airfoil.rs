//! Airfoil cross-section extraction from mesh geometry.
//!
//! Slices a wing mesh along its span to extract 2D airfoil profiles,
//! then derives aerodynamic coefficients from thickness and camber.

use aeroblend_physics::types::MeshData;

/// Airfoil profile extracted from mesh geometry.
#[derive(Debug, Clone)]
pub struct AirfoilProfile {
    pub thickness_ratio: f64,
    pub camber_fraction: f64,
    pub max_thickness_position: f64,
    pub max_camber_position: f64,
    pub le_radius_ratio: f64,
    pub slice_count: usize,
}

/// Aerodynamic coefficients derived from airfoil geometry.
#[derive(Debug, Clone)]
pub struct AeroCoefficients {
    pub cl_alpha: f64,
    pub alpha_zero_lift_rad: f64,
    pub cl_max: f64,
    pub alpha_stall_rad: f64,
    pub cd0: f64,
}

/// Extract an airfoil profile from mesh geometry by slicing along the span.
///
/// Returns `None` if the mesh is too coarse, non-airfoil-shaped, or fails sanity checks.
pub fn extract_airfoil_profile(mesh: &MeshData) -> Option<AirfoilProfile> {
    if mesh.vertices.len() < 3 || mesh.indices.len() < 3 {
        return None;
    }

    // 1. Bounding box and axis determination
    let (bb_min, bb_max) = bounding_box(&mesh.vertices);
    let extent = [
        bb_max[0] - bb_min[0],
        bb_max[1] - bb_min[1],
        bb_max[2] - bb_min[2],
    ];

    // Sort axes by extent: span = longest, chord = second, thickness = shortest
    let mut axes: [(usize, f64); 3] = [(0, extent[0]), (1, extent[1]), (2, extent[2])];
    axes.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    let span_axis = axes[0].0;
    let chord_axis = axes[1].0;
    let thickness_axis = axes[2].0;
    let chord_length = axes[1].1;

    if chord_length < 1e-6 || axes[2].1 < 1e-6 {
        return None;
    }

    // 2. Slice at 20%, 35%, 50%, 65%, 80% of span
    let span_min = bb_min[span_axis];
    let span_length = bb_max[span_axis] - span_min;
    if span_length < 1e-6 {
        return None;
    }

    let fractions = [0.20, 0.35, 0.50, 0.65, 0.80];
    let mut valid_profiles = Vec::new();

    for &frac in &fractions {
        let plane_pos = span_min + frac * span_length;
        let points = slice_mesh(mesh, span_axis, chord_axis, thickness_axis, plane_pos);

        if points.len() < 6 {
            continue;
        }

        if let Some(profile) = analyze_cross_section(&points) {
            valid_profiles.push(profile);
        }
    }

    if valid_profiles.is_empty() {
        return None;
    }

    // 3. Average across slices
    let n = valid_profiles.len() as f64;
    let avg = AirfoilProfile {
        thickness_ratio: valid_profiles.iter().map(|p| p.thickness_ratio).sum::<f64>() / n,
        camber_fraction: valid_profiles.iter().map(|p| p.camber_fraction).sum::<f64>() / n,
        max_thickness_position: valid_profiles
            .iter()
            .map(|p| p.max_thickness_position)
            .sum::<f64>()
            / n,
        max_camber_position: valid_profiles
            .iter()
            .map(|p| p.max_camber_position)
            .sum::<f64>()
            / n,
        le_radius_ratio: valid_profiles
            .iter()
            .map(|p| p.le_radius_ratio)
            .sum::<f64>()
            / n,
        slice_count: valid_profiles.len(),
    };

    // 4. Sanity check
    if avg.thickness_ratio < 0.02 || avg.thickness_ratio > 0.40 {
        return None;
    }
    if avg.camber_fraction.abs() > 0.15 {
        return None;
    }

    Some(avg)
}

/// Derive aerodynamic coefficients from an airfoil profile using empirical correlations.
pub fn derive_aero_coefficients(profile: &AirfoilProfile) -> AeroCoefficients {
    let tc = profile.thickness_ratio;
    let cam = profile.camber_fraction;
    let x_max = profile.max_thickness_position.max(0.1);

    let cl_alpha =
        (2.0 * std::f64::consts::PI * (1.0 - 1.07 * tc)).clamp(3.0, 7.0);

    let alpha_zero_lift_rad = -2.0 * cam;

    let cl_max = (0.9 + 4.5 * cam - 16.0 * tc * tc).clamp(0.8, 2.0);

    let alpha_stall_deg = (12.0 + 18.0 * cam - 50.0 * tc).clamp(8.0, 20.0);
    let alpha_stall_rad = alpha_stall_deg.to_radians();

    // cd0 = 2 * Cf * FF, where Cf=0.003, FF=1+0.6/x_max*(t/c)+100*(t/c)^4
    let cf = 0.003;
    let ff = 1.0 + 0.6 / x_max * tc + 100.0 * tc.powi(4);
    let cd0 = 2.0 * cf * ff;

    AeroCoefficients {
        cl_alpha,
        alpha_zero_lift_rad,
        cl_max,
        alpha_stall_rad,
        cd0,
    }
}

// ─────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────

fn bounding_box(vertices: &[[f32; 3]]) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];
    for v in vertices {
        for i in 0..3 {
            // JSF-Rust SR-501: Use From/Into for type conversion instead of `as` cast
            min[i] = min[i].min(f64::from(v[i]));
            max[i] = max[i].max(f64::from(v[i]));
        }
    }
    (min, max)
}

/// Slice the mesh with a plane perpendicular to `span_axis` at `plane_pos`.
/// Returns 2D points in (chord, thickness) space.
fn slice_mesh(
    mesh: &MeshData,
    span_axis: usize,
    chord_axis: usize,
    thickness_axis: usize,
    plane_pos: f64,
) -> Vec<[f64; 2]> {
    let mut points = Vec::new();
    let tri_count = mesh.indices.len() / 3;

    for t in 0..tri_count {
        let i0 = mesh.indices[t * 3] as usize;
        let i1 = mesh.indices[t * 3 + 1] as usize;
        let i2 = mesh.indices[t * 3 + 2] as usize;

        if i0 >= mesh.vertices.len()
            || i1 >= mesh.vertices.len()
            || i2 >= mesh.vertices.len()
        {
            continue;
        }

        intersect_triangle_plane(
            &mesh.vertices[i0],
            &mesh.vertices[i1],
            &mesh.vertices[i2],
            span_axis,
            chord_axis,
            thickness_axis,
            plane_pos,
            &mut points,
        );
    }

    points
}

/// Intersect a triangle with a plane, appending 0 or 2 points in (chord, thickness) space.
fn intersect_triangle_plane(
    v0: &[f32; 3],
    v1: &[f32; 3],
    v2: &[f32; 3],
    span_axis: usize,
    chord_axis: usize,
    thickness_axis: usize,
    plane_pos: f64,
    out: &mut Vec<[f64; 2]>,
) {
    let verts = [v0, v1, v2];
    // JSF-Rust SR-501: Use From/Into for type conversion instead of `as` cast
    let d = [
        f64::from(verts[0][span_axis]) - plane_pos,
        f64::from(verts[1][span_axis]) - plane_pos,
        f64::from(verts[2][span_axis]) - plane_pos,
    ];

    let edges = [(0usize, 1usize), (1, 2), (2, 0)];
    let start = out.len();

    for &(i, j) in &edges {
        if out.len() - start >= 2 {
            break;
        }
        if d[i] * d[j] < 0.0 {
            let t = d[i] / (d[i] - d[j]);
            let chord_val = f64::from(verts[i][chord_axis])
                + t * (f64::from(verts[j][chord_axis]) - f64::from(verts[i][chord_axis]));
            let thick_val = f64::from(verts[i][thickness_axis])
                + t * (f64::from(verts[j][thickness_axis]) - f64::from(verts[i][thickness_axis]));
            out.push([chord_val, thick_val]);
        } else if d[i].abs() < 1e-9 && d[j].abs() >= 1e-9 && out.len() - start < 2 {
            out.push([
                f64::from(verts[i][chord_axis]),
                f64::from(verts[i][thickness_axis]),
            ]);
        }
    }

    // Ensure we have exactly 0 or 2 points from this triangle
    if out.len() - start == 1 {
        out.pop();
    }
}

/// Analyze a 2D cross-section to extract profile parameters.
fn analyze_cross_section(points: &[[f64; 2]]) -> Option<AirfoilProfile> {
    if points.len() < 6 {
        return None;
    }

    // Find chord range
    let chord_min = points.iter().map(|p| p[0]).fold(f64::MAX, f64::min);
    let chord_max = points.iter().map(|p| p[0]).fold(f64::MIN, f64::max);
    let chord = chord_max - chord_min;
    if chord < 1e-6 {
        return None;
    }

    // Compute chord line from LE/TE centroids for proper camber measurement
    let le_tol = chord * 0.05;
    let te_tol = chord * 0.05;
    let le_ys: Vec<f64> = points
        .iter()
        .filter(|p| p[0] - chord_min < le_tol)
        .map(|p| p[1])
        .collect();
    let te_ys: Vec<f64> = points
        .iter()
        .filter(|p| chord_max - p[0] < te_tol)
        .map(|p| p[1])
        .collect();

    #[allow(clippy::cast_precision_loss)]
    let y_le = if !le_ys.is_empty() {
        le_ys.iter().sum::<f64>() / le_ys.len() as f64
    } else {
        points.iter().map(|p| p[1]).sum::<f64>() / points.len() as f64
    };
    #[allow(clippy::cast_precision_loss)]
    let y_te = if !te_ys.is_empty() {
        te_ys.iter().sum::<f64>() / te_ys.len() as f64
    } else {
        y_le
    };

    // Split into upper (above chord line) and lower (below chord line)
    let mut upper: Vec<[f64; 2]> = Vec::new();
    let mut lower: Vec<[f64; 2]> = Vec::new();
    for p in points {
        let chord_line_y = y_le + (y_te - y_le) * (p[0] - chord_min) / chord;
        if p[1] >= chord_line_y {
            upper.push(*p);
        } else {
            lower.push(*p);
        }
    }

    if upper.len() < 3 || lower.len() < 3 {
        return None;
    }

    // Sort by chord position
    upper.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());
    lower.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());

    // Sample at 20 chord stations
    let n_stations = 20;
    let mut max_thickness = 0.0_f64;
    let mut max_thickness_pos = 0.3_f64;
    let mut max_camber = 0.0_f64;
    let mut max_camber_pos = 0.3_f64;

    for i in 0..n_stations {
        let x_frac = (i as f64 + 0.5) / n_stations as f64;
        let x = chord_min + x_frac * chord;
        let chord_line_y = y_le + (y_te - y_le) * x_frac;

        let y_upper = interpolate_at(&upper, x);
        let y_lower = interpolate_at(&lower, x);

        if let (Some(yu), Some(yl)) = (y_upper, y_lower) {
            let thickness = (yu - yl).abs();
            let camber = (yu + yl) / 2.0 - chord_line_y;

            if thickness > max_thickness {
                max_thickness = thickness;
                max_thickness_pos = x_frac;
            }
            if camber.abs() > max_camber.abs() {
                max_camber = camber;
                max_camber_pos = x_frac;
            }
        }
    }

    let thickness_ratio = max_thickness / chord;
    let camber_fraction = max_camber / chord;

    // Leading edge radius estimate: thickness at 1% chord
    let x_le = chord_min + 0.01 * chord;
    let le_thickness = match (interpolate_at(&upper, x_le), interpolate_at(&lower, x_le)) {
        (Some(yu), Some(yl)) => (yu - yl).abs(),
        _ => 0.0,
    };
    let le_radius_ratio = le_thickness / chord;

    Some(AirfoilProfile {
        thickness_ratio,
        camber_fraction,
        max_thickness_position: max_thickness_pos,
        max_camber_position: max_camber_pos,
        le_radius_ratio,
        slice_count: 1,
    })
}

/// Linear interpolation of y at x from sorted (x, y) points.
fn interpolate_at(points: &[[f64; 2]], x: f64) -> Option<f64> {
    if points.is_empty() {
        return None;
    }
    if x <= points[0][0] {
        return Some(points[0][1]);
    }
    if x >= points[points.len() - 1][0] {
        return Some(points[points.len() - 1][1]);
    }

    for i in 0..points.len() - 1 {
        if points[i][0] <= x && x <= points[i + 1][0] {
            let dx = points[i + 1][0] - points[i][0];
            if dx.abs() < 1e-12 {
                return Some(points[i][1]);
            }
            let t = (x - points[i][0]) / dx;
            return Some(points[i][1] + t * (points[i + 1][1] - points[i][1]));
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a wing mesh from upper/lower surface functions.
    /// Coordinate system: X=span, Y=thickness, Z=chord.
    fn make_wing_mesh(
        upper_fn: impl Fn(f64) -> f64,
        lower_fn: impl Fn(f64) -> f64,
        chord: f64,
        span: f64,
        n_chord: usize,
        n_span: usize,
    ) -> MeshData {
        let mut vertices = Vec::new();

        for j in 0..=n_span {
            let x_span = -span / 2.0 + (j as f64 / n_span as f64) * span;

            // Upper surface points
            for i in 0..=n_chord {
                let xc = i as f64 / n_chord as f64;
                let y = upper_fn(xc) * chord;
                let z = xc * chord;
                vertices.push([x_span as f32, y as f32, z as f32]);
            }

            // Lower surface points
            for i in 0..=n_chord {
                let xc = i as f64 / n_chord as f64;
                let y = lower_fn(xc) * chord;
                let z = xc * chord;
                vertices.push([x_span as f32, y as f32, z as f32]);
            }
        }

        let pts_per_station = 2 * (n_chord + 1);
        let mut indices = Vec::new();

        // Triangulate upper surface
        for j in 0..n_span {
            for i in 0..n_chord {
                let v00 = (j * pts_per_station + i) as u32;
                let v01 = (j * pts_per_station + i + 1) as u32;
                let v10 = ((j + 1) * pts_per_station + i) as u32;
                let v11 = ((j + 1) * pts_per_station + i + 1) as u32;
                indices.extend_from_slice(&[v00, v10, v01, v01, v10, v11]);
            }
        }

        // Triangulate lower surface
        let lower_offset = n_chord + 1;
        for j in 0..n_span {
            for i in 0..n_chord {
                let v00 = (j * pts_per_station + lower_offset + i) as u32;
                let v01 = (j * pts_per_station + lower_offset + i + 1) as u32;
                let v10 = ((j + 1) * pts_per_station + lower_offset + i) as u32;
                let v11 = ((j + 1) * pts_per_station + lower_offset + i + 1) as u32;
                indices.extend_from_slice(&[v00, v01, v10, v01, v11, v10]);
            }
        }

        // Close LE and TE between upper and lower surfaces
        for j in 0..n_span {
            let u_le = (j * pts_per_station) as u32;
            let l_le = (j * pts_per_station + lower_offset) as u32;
            let u_le_next = ((j + 1) * pts_per_station) as u32;
            let l_le_next = ((j + 1) * pts_per_station + lower_offset) as u32;
            indices.extend_from_slice(&[u_le, l_le, u_le_next, l_le, l_le_next, u_le_next]);

            let u_te = (j * pts_per_station + n_chord) as u32;
            let l_te = (j * pts_per_station + lower_offset + n_chord) as u32;
            let u_te_next = ((j + 1) * pts_per_station + n_chord) as u32;
            let l_te_next = ((j + 1) * pts_per_station + lower_offset + n_chord) as u32;
            indices.extend_from_slice(&[u_te, u_te_next, l_te, l_te, u_te_next, l_te_next]);
        }

        MeshData {
            vertices,
            normals: Vec::new(),
            uvs: Vec::new(),
            indices,
            name: "test_wing".to_string(),
        }
    }

    /// NACA 4-digit thickness distribution: yt(x/c) for max thickness t.
    fn naca_thickness(xc: f64, t: f64) -> f64 {
        let xc = xc.clamp(0.0, 1.0);
        5.0 * t
            * (0.2969 * xc.sqrt() - 0.1260 * xc - 0.3516 * xc.powi(2)
                + 0.2843 * xc.powi(3)
                - 0.1015 * xc.powi(4))
    }

    /// NACA 4-digit camber line: yc(x/c) for max camber m at position p.
    fn naca_camber(xc: f64, m: f64, p: f64) -> f64 {
        if p <= 0.0 || m == 0.0 {
            return 0.0;
        }
        let xc = xc.clamp(0.0, 1.0);
        if xc < p {
            m / (p * p) * (2.0 * p * xc - xc * xc)
        } else {
            m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * xc - xc * xc)
        }
    }

    fn make_naca_0012_wing() -> MeshData {
        let t = 0.12;
        make_wing_mesh(
            |xc| naca_thickness(xc, t),
            |xc| -naca_thickness(xc, t),
            2.0,  // chord
            10.0, // span
            30,
            8,
        )
    }

    fn make_naca_2412_wing() -> MeshData {
        let (m, p, t) = (0.02, 0.4, 0.12);
        make_wing_mesh(
            |xc| naca_camber(xc, m, p) + naca_thickness(xc, t),
            |xc| naca_camber(xc, m, p) - naca_thickness(xc, t),
            2.0,
            10.0,
            30,
            8,
        )
    }

    #[test]
    fn test_extract_empty_mesh() {
        let mesh = MeshData::default();
        assert!(extract_airfoil_profile(&mesh).is_none());
    }

    #[test]
    fn test_extract_too_few_vertices() {
        let mesh = MeshData {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            indices: vec![0, 1, 0],
            ..Default::default()
        };
        assert!(extract_airfoil_profile(&mesh).is_none());
    }

    #[test]
    fn test_extract_symmetric_airfoil() {
        let mesh = make_naca_0012_wing();
        let profile = extract_airfoil_profile(&mesh).expect("should extract NACA 0012");
        // t/c should be close to 0.12
        assert!(
            (profile.thickness_ratio - 0.12).abs() < 0.03,
            "t/c = {}, expected ~0.12",
            profile.thickness_ratio
        );
        // Symmetric → camber near zero
        assert!(
            profile.camber_fraction.abs() < 0.01,
            "camber = {}, expected ~0",
            profile.camber_fraction
        );
        // Max thickness position ~0.3 for NACA 0012
        assert!(
            profile.max_thickness_position > 0.15 && profile.max_thickness_position < 0.50,
            "max_t_pos = {}",
            profile.max_thickness_position
        );
    }

    #[test]
    fn test_extract_cambered_airfoil() {
        let mesh = make_naca_2412_wing();
        let profile = extract_airfoil_profile(&mesh).expect("should extract NACA 2412");
        // t/c should be close to 0.12
        assert!(
            (profile.thickness_ratio - 0.12).abs() < 0.03,
            "t/c = {}, expected ~0.12",
            profile.thickness_ratio
        );
        // Should detect positive camber
        assert!(
            profile.camber_fraction > 0.005,
            "camber = {}, expected positive",
            profile.camber_fraction
        );
    }

    #[test]
    fn test_derive_symmetric_coefficients() {
        let profile = AirfoilProfile {
            thickness_ratio: 0.12,
            camber_fraction: 0.0,
            max_thickness_position: 0.30,
            max_camber_position: 0.0,
            le_radius_ratio: 0.02,
            slice_count: 5,
        };
        let coeffs = derive_aero_coefficients(&profile);
        // cl_alpha = 2π*(1-1.07*0.12) ≈ 5.47
        assert!(coeffs.cl_alpha > 5.0 && coeffs.cl_alpha < 6.5);
        // Symmetric → alpha_zero_lift = 0
        assert!(coeffs.alpha_zero_lift_rad.abs() < 1e-10);
        // Reasonable cl_max
        assert!(coeffs.cl_max >= 0.8 && coeffs.cl_max <= 2.0);
        // Reasonable alpha_stall
        assert!(coeffs.alpha_stall_rad > 0.1 && coeffs.alpha_stall_rad < 0.4);
        // Reasonable cd0
        assert!(coeffs.cd0 > 0.005 && coeffs.cd0 < 0.03);
    }

    #[test]
    fn test_derive_cambered_coefficients() {
        let profile = AirfoilProfile {
            thickness_ratio: 0.12,
            camber_fraction: 0.02,
            max_thickness_position: 0.30,
            max_camber_position: 0.40,
            le_radius_ratio: 0.02,
            slice_count: 5,
        };
        let coeffs = derive_aero_coefficients(&profile);
        // Cambered → negative alpha_zero_lift
        assert!(
            coeffs.alpha_zero_lift_rad < 0.0,
            "a0l = {}, expected < 0",
            coeffs.alpha_zero_lift_rad
        );
        assert!(
            (coeffs.alpha_zero_lift_rad - (-0.04)).abs() < 0.01,
            "a0l = {}",
            coeffs.alpha_zero_lift_rad
        );
    }

    #[test]
    fn test_derive_coefficients_clamping() {
        // Very thick airfoil
        let thick = AirfoilProfile {
            thickness_ratio: 0.35,
            camber_fraction: 0.0,
            max_thickness_position: 0.30,
            max_camber_position: 0.0,
            le_radius_ratio: 0.05,
            slice_count: 5,
        };
        let c = derive_aero_coefficients(&thick);
        assert!(c.cl_alpha >= 3.0, "cl_alpha should be clamped >= 3.0");
        assert!(c.alpha_stall_rad >= 8.0_f64.to_radians());

        // Very thin airfoil
        let thin = AirfoilProfile {
            thickness_ratio: 0.03,
            camber_fraction: 0.0,
            max_thickness_position: 0.30,
            max_camber_position: 0.0,
            le_radius_ratio: 0.005,
            slice_count: 5,
        };
        let c = derive_aero_coefficients(&thin);
        assert!(c.cl_alpha <= 7.0, "cl_alpha should be clamped <= 7.0");
    }

    #[test]
    fn test_interpolate_at_basic() {
        let pts = vec![[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]];
        assert!((interpolate_at(&pts, 0.5).unwrap() - 0.5).abs() < 1e-10);
        assert!((interpolate_at(&pts, 1.5).unwrap() - 0.5).abs() < 1e-10);
        // Extrapolation clamps
        assert!((interpolate_at(&pts, -1.0).unwrap() - 0.0).abs() < 1e-10);
        assert!((interpolate_at(&pts, 3.0).unwrap() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_at_empty() {
        assert!(interpolate_at(&[], 0.5).is_none());
    }
}
