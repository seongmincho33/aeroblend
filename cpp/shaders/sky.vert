#version 330 core

uniform mat4 u_view;
uniform mat4 u_proj;

layout(location = 0) in vec3 in_position;

out vec3 v_direction;

void main() {
    v_direction = in_position;
    // Remove translation from view matrix for skybox effect
    mat4 view_no_translate = mat4(mat3(u_view));
    vec4 pos = u_proj * view_no_translate * vec4(in_position, 1.0);
    gl_Position = pos.xyww;  // depth = 1.0 (far plane)
}
