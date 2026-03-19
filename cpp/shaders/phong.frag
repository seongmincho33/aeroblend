#version 330 core

uniform vec3 u_light_dir;
uniform vec3 u_light_color;
uniform vec3 u_ambient_color;
uniform vec3 u_camera_pos;
uniform vec3 u_base_color;
uniform float u_specular_strength;
uniform float u_shininess;

in vec3 v_world_pos;
in vec3 v_normal;
in vec2 v_uv;

out vec4 frag_color;

void main() {
    vec3 N = normalize(v_normal);
    vec3 L = normalize(u_light_dir);
    vec3 V = normalize(u_camera_pos - v_world_pos);
    vec3 R = reflect(-L, N);

    // Ambient
    vec3 ambient = u_ambient_color * u_base_color;

    // Diffuse
    float diff = max(dot(N, L), 0.0);
    vec3 diffuse = diff * u_light_color * u_base_color;

    // Specular (Blinn-Phong)
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), u_shininess);
    vec3 specular = u_specular_strength * spec * u_light_color;

    frag_color = vec4(ambient + diffuse + specular, 1.0);
}
