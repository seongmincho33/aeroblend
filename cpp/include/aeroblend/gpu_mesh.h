#ifndef AEROBLEND_GPU_MESH_H
#define AEROBLEND_GPU_MESH_H

#include <glad/gl.h>
#include <aeroblend/ffi.h>
#include <string>

class GpuMesh {
public:
    GpuMesh() = default;
    ~GpuMesh();

    // Move-only: transfer GL resource ownership
    GpuMesh(GpuMesh&& other) noexcept
        : vao_(other.vao_), vbo_pos_(other.vbo_pos_), vbo_norm_(other.vbo_norm_),
          vbo_uv_(other.vbo_uv_), ebo_(other.ebo_), index_count_(other.index_count_),
          name_(std::move(other.name_)) {
        other.vao_ = 0;
        other.vbo_pos_ = 0;
        other.vbo_norm_ = 0;
        other.vbo_uv_ = 0;
        other.ebo_ = 0;
        other.index_count_ = 0;
    }

    GpuMesh& operator=(GpuMesh&& other) noexcept {
        if (this != &other) {
            cleanup();
            vao_ = other.vao_;
            vbo_pos_ = other.vbo_pos_;
            vbo_norm_ = other.vbo_norm_;
            vbo_uv_ = other.vbo_uv_;
            ebo_ = other.ebo_;
            index_count_ = other.index_count_;
            name_ = std::move(other.name_);
            other.vao_ = 0;
            other.vbo_pos_ = 0;
            other.vbo_norm_ = 0;
            other.vbo_uv_ = 0;
            other.ebo_ = 0;
            other.index_count_ = 0;
        }
        return *this;
    }

    // No copying (GL resources are not copyable)
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    // Upload mesh data to GPU. Copies data, so Rust pointers can be freed after.
    void upload(const MeshDataC& data);

    // Upload raw arrays (for procedural geometry).
    void upload_arrays(const float* verts, int vert_count,
                       const float* normals, int normal_count,
                       const uint32_t* indices, int index_count,
                       const float* uvs = nullptr, int uv_count = 0);

    void draw() const;
    void draw_lines() const;

    const std::string& name() const { return name_; }
    GLuint vao() const { return vao_; }
    int index_count() const { return index_count_; }

private:
    GLuint vao_ = 0;
    GLuint vbo_pos_ = 0;
    GLuint vbo_norm_ = 0;
    GLuint vbo_uv_ = 0;
    GLuint ebo_ = 0;
    int index_count_ = 0;
    std::string name_;

    void cleanup();
};

#endif // AEROBLEND_GPU_MESH_H
