#include <aeroblend/projectile_renderer.h>
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace aeroblend {

ProjectileRenderer::~ProjectileRenderer() {
    cleanup();
}

bool ProjectileRenderer::init(const std::string& shader_dir) {
    // 트레이서 셰이더 로드
    bool success = shader_.load(shader_dir + "/tracer.vert", shader_dir + "/tracer.frag");
    if (!success) {
        fprintf(stderr, "Failed to load tracer shader\n");
        return false;
    }

    // VAO/VBO 생성
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // 최대 크기로 버퍼 할당 (GL_DYNAMIC_DRAW)
    const GLsizeiptr buf_size = static_cast<GLsizeiptr>(
        kMaxProjectiles * kVerticesPerTracer * kFloatsPerVertex * sizeof(float));
    glBufferData(GL_ARRAY_BUFFER, buf_size, nullptr, GL_DYNAMIC_DRAW);

    // 어트리뷰트 0: 위치 (vec3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          kFloatsPerVertex * sizeof(float),
                          reinterpret_cast<void*>(0));

    // 어트리뷰트 1: 색상 (vec3)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                          kFloatsPerVertex * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    initialized_ = true;
    return true;
}

void ProjectileRenderer::render(const ProjectileStateC* projectiles, uint32_t count,
                                const Mat4C& view, const Mat4C& projection) {
    if (!initialized_ || projectiles == nullptr || count == 0) {
        return;
    }

    // 활성 탄환만 수집, 최대 개수 제한
    uint32_t active_count = 0;

    for (uint32_t i = 0; i < count && active_count < kMaxProjectiles; ++i) {
        const auto& p = projectiles[i];
        if (p.active == 0) {
            continue;
        }

        // 속도 벡터 크기 계산
        const float vx = static_cast<float>(p.velocity.x);
        const float vy = static_cast<float>(p.velocity.y);
        const float vz = static_cast<float>(p.velocity.z);
        const float speed = sqrtf(vx * vx + vy * vy + vz * vz);

        if (speed < 1.0f) {
            continue;  // 정지 탄환 무시
        }

        // 트레이서 길이: 속력 × 0.02초, 최대 10m
        const float tracer_len = std::min(speed * 0.02f, 10.0f);

        // 속도 방향 정규화
        const float inv_speed = 1.0f / speed;
        const float dx = vx * inv_speed;
        const float dy = vy * inv_speed;
        const float dz = vz * inv_speed;

        // 시작점: 현재 위치
        const float px = static_cast<float>(p.position.x);
        const float py = static_cast<float>(p.position.y);
        const float pz = static_cast<float>(p.position.z);

        // 끝점: 위치 - 방향 × 트레이서 길이 (꼬리)
        const float ex = px - dx * tracer_len;
        const float ey = py - dy * tracer_len;
        const float ez = pz - dz * tracer_len;

        // 색상: 노란색 (#FFAA00) 기본, 속도 감소 시 빨간색으로 페이드
        // 기준 속도 800 m/s (일반 기총 초구속도 범위)
        const float speed_ratio = std::min(speed / 800.0f, 1.0f);
        const float r = 1.0f;
        const float g = 0.67f * speed_ratio;  // 느릴수록 빨간색
        const float b = 0.0f;

        // 꼬리 색상 (어두운 빨간색)
        const float tail_r = 1.0f * 0.5f;
        const float tail_g = g * 0.3f;
        const float tail_b = 0.0f;

        // 정점 데이터 기록 (시작점 = 밝은 노란색, 끝점 = 어두운 빨간색)
        const uint32_t base = active_count * kVerticesPerTracer * kFloatsPerVertex;

        // 시작점 (탄두)
        vertex_data_[base + 0] = px;
        vertex_data_[base + 1] = py;
        vertex_data_[base + 2] = pz;
        vertex_data_[base + 3] = r;
        vertex_data_[base + 4] = g;
        vertex_data_[base + 5] = b;

        // 끝점 (꼬리)
        vertex_data_[base + 6] = ex;
        vertex_data_[base + 7] = ey;
        vertex_data_[base + 8] = ez;
        vertex_data_[base + 9] = tail_r;
        vertex_data_[base + 10] = tail_g;
        vertex_data_[base + 11] = tail_b;

        ++active_count;
    }

    if (active_count == 0) {
        return;
    }

    // VBO 업데이트
    const GLsizeiptr data_size = static_cast<GLsizeiptr>(
        active_count * kVerticesPerTracer * kFloatsPerVertex * sizeof(float));

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, data_size, vertex_data_);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // 셰이더 바인드, 유니폼 설정
    shader_.use();
    shader_.set_mat4("uView", view.m);
    shader_.set_mat4("uProjection", projection.m);

    // 깊이 쓰기 비활성 (트레이서가 다른 오브젝트에 가려지지만, 깊이 버퍼를 오염시키지 않음)
    glDepthMask(GL_FALSE);

    // 선 굵기 설정
    glLineWidth(2.0f);

    // 드로우
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(active_count * kVerticesPerTracer));
    glBindVertexArray(0);

    // 상태 복원
    glDepthMask(GL_TRUE);
    glLineWidth(1.0f);
}

void ProjectileRenderer::cleanup() {
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    initialized_ = false;
}

} // namespace aeroblend
