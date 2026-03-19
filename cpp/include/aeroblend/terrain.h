#ifndef AEROBLEND_TERRAIN_H
#define AEROBLEND_TERRAIN_H

#include <aeroblend/shader.h>
#include <aeroblend/gpu_mesh.h>

class TerrainRenderer {
public:
    bool init(const std::string& shader_dir);
    void render(const float* view_mat, const float* proj_mat);
    void shutdown();

private:
    Shader shader_;
    GpuMesh ground_;
    GpuMesh runway_;
    GpuMesh markings_;

    void create_ground();
    void create_runway();
    void create_markings();
};

#endif // AEROBLEND_TERRAIN_H
