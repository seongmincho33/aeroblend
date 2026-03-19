#ifndef AEROBLEND_PROJECTILE_RENDERER_H
#define AEROBLEND_PROJECTILE_RENDERER_H

#include <aeroblend/ffi.h>
#include <aeroblend/shader.h>
#include <cstdint>
#include <string>

namespace aeroblend {

/// 탄환 트레이서 렌더러
/// GL_LINES로 탄환 궤적을 노란색 선분으로 렌더링
class ProjectileRenderer {
public:
    ProjectileRenderer() = default;
    ~ProjectileRenderer();

    /// OpenGL 리소스 초기화 (VAO, VBO, 셰이더)
    bool init(const std::string& shader_dir);

    /// 활성 탄환을 트레이서로 렌더링
    /// @param projectiles 탄환 상태 배열
    /// @param count 활성 탄환 수
    /// @param view 뷰 행렬 (column-major float[16])
    /// @param projection 투영 행렬 (column-major float[16])
    void render(const ProjectileStateC* projectiles, uint32_t count,
                const Mat4C& view, const Mat4C& projection);

    /// OpenGL 리소스 해제
    void cleanup();

private:
    Shader shader_;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    bool initialized_ = false;

    // 트레이서 선분 데이터 (position + color per vertex)
    // 각 탄환: 2 vertices (start, end) × (3 pos + 3 color) = 12 floats
    static constexpr uint32_t kMaxProjectiles = 512;
    static constexpr uint32_t kFloatsPerVertex = 6;  // xyz + rgb
    static constexpr uint32_t kVerticesPerTracer = 2;
    float vertex_data_[kMaxProjectiles * kVerticesPerTracer * kFloatsPerVertex] = {};
};

} // namespace aeroblend

#endif // AEROBLEND_PROJECTILE_RENDERER_H
