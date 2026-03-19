#include <aeroblend/sky.h>
#include <cmath>

namespace {

// Unit cube vertices for skybox
const float kCubeVerts[] = {
    // +X
     1, -1, -1,   1, -1,  1,   1,  1,  1,   1,  1, -1,
    // -X
    -1, -1,  1,  -1, -1, -1,  -1,  1, -1,  -1,  1,  1,
    // +Y
    -1,  1, -1,   1,  1, -1,   1,  1,  1,  -1,  1,  1,
    // -Y
    -1, -1,  1,   1, -1,  1,   1, -1, -1,  -1, -1, -1,
    // +Z
    -1, -1,  1,  -1,  1,  1,   1,  1,  1,   1, -1,  1,
    // -Z
     1, -1, -1,   1,  1, -1,  -1,  1, -1,  -1, -1, -1,
};

const uint32_t kCubeIndices[] = {
     0,  1,  2,   2,  3,  0,
     4,  5,  6,   6,  7,  4,
     8,  9, 10,  10, 11,  8,
    12, 13, 14,  14, 15, 12,
    16, 17, 18,  18, 19, 16,
    20, 21, 22,  22, 23, 20,
};

} // namespace

void SkyRenderer::create_cube() {
    cube_.upload_arrays(kCubeVerts, 24, nullptr, 0, kCubeIndices, 36);
}

bool SkyRenderer::init(const std::string& shader_dir) {
    bool success = shader_.load(shader_dir + "/sky.vert", shader_dir + "/sky.frag");
    if (success) {
        create_cube();
    }
    return success;
}

void SkyRenderer::render(const float* view_mat, const float* proj_mat, float altitude) {
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);

    shader_.use();
    shader_.set_mat4("u_view", view_mat);
    shader_.set_mat4("u_proj", proj_mat);

    // Sun direction (high in sky, slightly to the side)
    float sun_len = sqrtf(0.5f*0.5f + 0.8f*0.8f + 0.3f*0.3f);
    shader_.set_vec3("u_sun_dir", 0.5f/sun_len, 0.8f/sun_len, 0.3f/sun_len);

    // Sky colors
    shader_.set_vec3("u_sky_top", 0.15f, 0.3f, 0.8f);
    shader_.set_vec3("u_sky_horizon", 0.6f, 0.75f, 0.95f);
    shader_.set_vec3("u_sky_bottom", 0.3f, 0.4f, 0.35f);
    shader_.set_float("u_altitude", altitude);

    cube_.draw();

    glEnable(GL_CULL_FACE);
    glDepthFunc(GL_LESS);
}

void SkyRenderer::shutdown() {
    // GpuMesh and Shader clean up in destructors
}
