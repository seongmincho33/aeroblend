#include <aeroblend/renderer.h>
#include <cmath>
#include <cstdio>

namespace {

// Extract 3x3 normal matrix from model matrix (upper-left 3x3, as float)
void extract_normal_matrix(const float* model4x4, float* out3x3) {
    out3x3[0] = model4x4[0]; out3x3[1] = model4x4[1]; out3x3[2] = model4x4[2];
    out3x3[3] = model4x4[4]; out3x3[4] = model4x4[5]; out3x3[5] = model4x4[6];
    out3x3[6] = model4x4[8]; out3x3[7] = model4x4[9]; out3x3[8] = model4x4[10];
}

} // namespace

bool Renderer::init(const std::string& shader_dir) {
    bool success = phong_.load(shader_dir + "/phong.vert", shader_dir + "/phong.frag");

    if (!success) {
        fprintf(stderr, "Failed to load phong shader\n");
    } else if (!sky_.init(shader_dir)) {
        fprintf(stderr, "Failed to init sky\n");
        success = false;
    } else if (!terrain_.init(shader_dir)) {
        fprintf(stderr, "Failed to init terrain\n");
        success = false;
    } else {
        // Normalize sun direction
        float len = sqrtf(sun_dir_[0]*sun_dir_[0] + sun_dir_[1]*sun_dir_[1] + sun_dir_[2]*sun_dir_[2]);
        sun_dir_[0] /= len;
        sun_dir_[1] /= len;
        sun_dir_[2] /= len;
    }

    return success;
}

void Renderer::load_aircraft(const LoadedModel* model) {
    aircraft_meshes_.clear();

    if (model != nullptr) {
        AircraftModelC data = aeroblend_model_get_data(model);
        printf("Loading aircraft: %u meshes, mass=%.0fkg\n", data.mesh_count, data.mass_kg);

        for (uint32_t i = 0; i < data.mesh_count; i++) {
            MeshDataC mesh = aeroblend_model_get_mesh(model, i);
            if (mesh.vertex_count > 0) {
                GpuMesh gpu;
                gpu.upload(mesh);
                printf("  Mesh %u: '%s' (%u verts, %u indices)\n",
                       i, (mesh.name != nullptr) ? mesh.name : "?",
                       mesh.vertex_count, mesh.index_count);
                aircraft_meshes_.push_back(std::move(gpu));
            }
        }
    }
}

void Renderer::render(const AircraftPhysicsStateC& state,
                      const Mat4C& view, const Mat4C& proj,
                      const Vec3C& cam_pos) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 1. Sky
    sky_.render(view.m, proj.m, static_cast<float>(state.altitude_m));

    // 2. Terrain
    terrain_.render(view.m, proj.m);

    // 3. Aircraft
    render_aircraft_at(state.orientation, state.position, view, proj, cam_pos);
}

void Renderer::render_aircraft_at(const Mat3C& orientation, const Vec3C& position,
                                  const Mat4C& view, const Mat4C& proj,
                                  const Vec3C& cam_pos) {
    if (aircraft_meshes_.empty()) { return; }

    glDisable(GL_CULL_FACE);

    Mat4C model_mat = aeroblend_build_model_matrix(&orientation, &position);
    // Fix Blender→glTF coordinate mismatch: model nose points -Z, engine uses +Z
    // Apply 180° Y rotation by negating columns 0 and 2
    for (int i = 0; i < 4; i++) {
        model_mat.m[i] = -model_mat.m[i];       // negate col 0
        model_mat.m[8 + i] = -model_mat.m[8 + i]; // negate col 2
    }
    float normal_mat[9];
    extract_normal_matrix(model_mat.m, normal_mat);

    phong_.use();
    phong_.set_mat4("u_model", model_mat.m);
    phong_.set_mat4("u_view", view.m);
    phong_.set_mat4("u_proj", proj.m);
    phong_.set_mat3("u_normal_matrix", normal_mat);
    phong_.set_vec3("u_light_dir", sun_dir_[0], sun_dir_[1], sun_dir_[2]);
    phong_.set_vec3("u_light_color", 1.0f, 0.98f, 0.9f);
    phong_.set_vec3("u_ambient_color", 0.15f, 0.18f, 0.25f);
    phong_.set_vec3("u_camera_pos",
                   static_cast<float>(cam_pos.x),
                   static_cast<float>(cam_pos.y),
                   static_cast<float>(cam_pos.z));
    phong_.set_vec3("u_base_color", 0.6f, 0.6f, 0.65f);
    phong_.set_float("u_specular_strength", 0.5f);
    phong_.set_float("u_shininess", 32.0f);

    for (auto& mesh : aircraft_meshes_) {
        mesh.draw();
    }

    glEnable(GL_CULL_FACE);
}

void Renderer::shutdown() {
    aircraft_meshes_.clear();
    sky_.shutdown();
    terrain_.shutdown();
}
