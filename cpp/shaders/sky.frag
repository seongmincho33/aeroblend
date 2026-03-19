#version 330 core

uniform vec3 u_sun_dir;
uniform vec3 u_sky_top;
uniform vec3 u_sky_horizon;
uniform vec3 u_sky_bottom;
uniform float u_altitude;

in vec3 v_direction;

out vec4 frag_color;

void main() {
    vec3 dir = normalize(v_direction);
    float y = dir.y;

    // Altitude-based sky darkening
    float alt_factor = clamp(u_altitude / 15000.0, 0.0, 1.0);
    vec3 space_color = vec3(0.02, 0.02, 0.1);
    vec3 adjusted_sky_top = mix(u_sky_top, space_color, alt_factor);
    vec3 adjusted_horizon = mix(u_sky_horizon, space_color * 2.0, alt_factor * 0.6);

    // Sky gradient
    vec3 color;
    if (y > 0.0) {
        float t = pow(y, 0.4);
        color = mix(adjusted_horizon, adjusted_sky_top, t);
    } else {
        float t = pow(-y, 0.6);
        color = mix(adjusted_horizon, u_sky_bottom, t);
    }

    // Sun glow
    float sun_dot = max(dot(dir, normalize(u_sun_dir)), 0.0);
    float sun_glow = pow(sun_dot, 128.0) * 1.5;
    float halo = pow(sun_dot, 8.0) * 0.15;
    color += vec3(1.0, 0.95, 0.8) * (sun_glow + halo);

    frag_color = vec4(color, 1.0);
}
