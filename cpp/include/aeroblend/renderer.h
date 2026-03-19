#ifndef AEROBLEND_RENDERER_H
#define AEROBLEND_RENDERER_H

#include <aeroblend/ffi.h>
#include <aeroblend/shader.h>
#include <aeroblend/gpu_mesh.h>
#include <aeroblend/sky.h>
#include <aeroblend/terrain.h>
#include <vector>
#include <string>

class Renderer {
public:
    bool init(const std::string& shader_dir);
    void load_aircraft(const LoadedModel* model);
    void render(const AircraftPhysicsStateC& state,
                const Mat4C& view, const Mat4C& proj,
                const Vec3C& cam_pos);
    // 지정 위치/방향에 항공기 메쉬를 렌더링 (더미 편대기용)
    void render_aircraft_at(const Mat3C& orientation, const Vec3C& position,
                            const Mat4C& view, const Mat4C& proj,
                            const Vec3C& cam_pos);
    void shutdown();

private:
    Shader phong_;
    SkyRenderer sky_;
    TerrainRenderer terrain_;
    std::vector<GpuMesh> aircraft_meshes_;

    // Sun direction (fixed for now)
    float sun_dir_[3] = {0.5f, 0.8f, 0.3f};
};

#endif // AEROBLEND_RENDERER_H
