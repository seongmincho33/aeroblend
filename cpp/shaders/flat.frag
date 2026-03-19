#version 330 core

uniform vec3 u_color;
uniform int u_grid_enabled;

in vec3 v_world_pos;

out vec4 frag_color;

void main() {
    vec3 col = u_color;

    if (u_grid_enabled == 1) {
        // Two-level grid: 100m fine grid + 1000m coarse grid
        float fine_size = 100.0;
        float coarse_size = 1000.0;

        // Fine grid lines (subtle)
        vec2 fine_grid = abs(fract(v_world_pos.xz / fine_size + 0.5) - 0.5) * fine_size;
        float fine_dist = min(fine_grid.x, fine_grid.y);
        float fine_line = 1.0 - smoothstep(0.0, 2.0, fine_dist);

        // Coarse grid lines (more visible)
        vec2 coarse_grid = abs(fract(v_world_pos.xz / coarse_size + 0.5) - 0.5) * coarse_size;
        float coarse_dist = min(coarse_grid.x, coarse_grid.y);
        float coarse_line = 1.0 - smoothstep(0.0, 4.0, coarse_dist);

        // Blend grid lines into base color
        col = mix(col, col * 1.3, fine_line * 0.3);
        col = mix(col, col * 1.6, coarse_line * 0.5);
    }

    frag_color = vec4(col, 1.0);
}
