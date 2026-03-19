#ifndef AEROBLEND_SKY_H
#define AEROBLEND_SKY_H

#include <aeroblend/shader.h>
#include <aeroblend/gpu_mesh.h>

class SkyRenderer {
public:
    bool init(const std::string& shader_dir);
    void render(const float* view_mat, const float* proj_mat, float altitude);
    void shutdown();

private:
    Shader shader_;
    GpuMesh cube_;

    void create_cube();
};

#endif // AEROBLEND_SKY_H
