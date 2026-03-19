#include <aeroblend/terrain.h>
#include <vector>
#include <cmath>

namespace {
constexpr float kGroundSize = 10000.0f;
constexpr float kRunwayLength = 3000.0f;
constexpr float kRunwayWidth = 45.0f;
constexpr float kMarkingWidth = 1.0f;
constexpr float kMarkingGap = 30.0f;
} // namespace

void TerrainRenderer::create_ground() {
    float s = kGroundSize;
    float verts[] = {
        -s, 0, -s,
         s, 0, -s,
         s, 0,  s,
        -s, 0,  s,
    };
    float norms[] = {
        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
    };
    uint32_t idx[] = { 0, 1, 2, 2, 3, 0 };
    ground_.upload_arrays(verts, 4, norms, 4, idx, 6);
}

void TerrainRenderer::create_runway() {
    float hw = kRunwayWidth / 2.0f;
    float hl = kRunwayLength / 2.0f;
    float y = 0.01f; // slightly above ground to avoid z-fighting
    float verts[] = {
        -hw, y, -hl,
         hw, y, -hl,
         hw, y,  hl,
        -hw, y,  hl,
    };
    float norms[] = {
        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
    };
    uint32_t idx[] = { 0, 1, 2, 2, 3, 0 };
    runway_.upload_arrays(verts, 4, norms, 4, idx, 6);
}

void TerrainRenderer::create_markings() {
    std::vector<float> verts;
    std::vector<uint32_t> indices;

    float hw = kMarkingWidth / 2.0f;
    float hl = kRunwayLength / 2.0f;
    float y = 0.02f;
    uint32_t base = 0;

    // Center line dashes
    for (float z = -hl + kMarkingGap; z < hl - kMarkingGap; z += kMarkingGap * 2.0f) {
        float z0 = z;
        float z1 = z + kMarkingGap;
        float v[] = { -hw, y, z0, hw, y, z0, hw, y, z1, -hw, y, z1 };
        for (int i = 0; i < 12; i++) verts.push_back(v[i]);
        uint32_t idx[] = { base, base+1, base+2, base+2, base+3, base };
        for (int i = 0; i < 6; i++) indices.push_back(idx[i]);
        base += 4;
    }

    // Edge lines (left and right)
    float edge_hw = kRunwayWidth / 2.0f - 1.0f;
    for (int side = -1; side <= 1; side += 2) {
        float x = side * edge_hw;
        float v[] = { x - hw, y, -hl, x + hw, y, -hl, x + hw, y, hl, x - hw, y, hl };
        for (int i = 0; i < 12; i++) verts.push_back(v[i]);
        uint32_t idx[] = { base, base+1, base+2, base+2, base+3, base };
        for (int i = 0; i < 6; i++) indices.push_back(idx[i]);
        base += 4;
    }

    if (!verts.empty()) {
        markings_.upload_arrays(verts.data(),
                                static_cast<int>(verts.size() / 3),
                                nullptr, 0,
                                indices.data(),
                                static_cast<int>(indices.size()));
    }
}

bool TerrainRenderer::init(const std::string& shader_dir) {
    bool success = shader_.load(shader_dir + "/flat.vert", shader_dir + "/flat.frag");
    if (success) {
        create_ground();
        create_runway();
        create_markings();
    }
    return success;
}

void TerrainRenderer::render(const float* view_mat, const float* proj_mat) {
    shader_.use();

    // Build MVP for each element
    // For flat shader: u_mvp = proj * view * model
    // Ground and runway are at world origin, so model = identity.
    // We compute mvp = proj * view on the CPU.
    float mvp[16];
    // mvp = proj * view (both are column-major 4x4)
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            float sum = 0;
            for (int k = 0; k < 4; k++) {
                sum += proj_mat[k * 4 + r] * view_mat[c * 4 + k];
            }
            mvp[c * 4 + r] = sum;
        }
    }

    // Ground (dark green with grid)
    shader_.set_mat4("u_mvp", mvp);
    shader_.set_int("u_grid_enabled", 1);
    shader_.set_vec3("u_color", 0.2f, 0.35f, 0.15f);
    ground_.draw();

    // Runway (dark gray, no grid)
    shader_.set_int("u_grid_enabled", 0);
    shader_.set_vec3("u_color", 0.25f, 0.25f, 0.28f);
    runway_.draw();

    // Markings (white, no grid)
    shader_.set_vec3("u_color", 0.9f, 0.9f, 0.9f);
    markings_.draw();
}

void TerrainRenderer::shutdown() {
    // Destructors handle cleanup
}
