#include <aeroblend/gpu_mesh.h>

GpuMesh::~GpuMesh() {
    cleanup();
}

void GpuMesh::cleanup() {
    if (ebo_) { glDeleteBuffers(1, &ebo_); ebo_ = 0; }
    if (vbo_uv_) { glDeleteBuffers(1, &vbo_uv_); vbo_uv_ = 0; }
    if (vbo_norm_) { glDeleteBuffers(1, &vbo_norm_); vbo_norm_ = 0; }
    if (vbo_pos_) { glDeleteBuffers(1, &vbo_pos_); vbo_pos_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    index_count_ = 0;
}

void GpuMesh::upload(const MeshDataC& data) {
    if (data.vertex_count > 0) {
        name_ = (data.name != nullptr) ? data.name : "unnamed";

        upload_arrays(
            reinterpret_cast<const float*>(data.vertices),
            static_cast<int>(data.vertex_count),
            reinterpret_cast<const float*>(data.normals),
            static_cast<int>(data.vertex_count),
            data.indices, static_cast<int>(data.index_count),
            reinterpret_cast<const float*>(data.uvs),
            static_cast<int>(data.vertex_count)
        );
    }
}

void GpuMesh::upload_arrays(const float* verts, int vert_count,
                            const float* normals, int normal_count,
                            const uint32_t* indices, int idx_count,
                            const float* uvs, int uv_count) {
    cleanup();

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    // Position buffer (vec3)
    glGenBuffers(1, &vbo_pos_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_pos_);
    glBufferData(GL_ARRAY_BUFFER, vert_count * 3 * sizeof(float), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    // Normal buffer (vec3)
    if (normals && normal_count > 0) {
        glGenBuffers(1, &vbo_norm_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_norm_);
        glBufferData(GL_ARRAY_BUFFER, normal_count * 3 * sizeof(float), normals, GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
    }

    // UV buffer (vec2)
    if (uvs && uv_count > 0) {
        glGenBuffers(1, &vbo_uv_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_uv_);
        glBufferData(GL_ARRAY_BUFFER, uv_count * 2 * sizeof(float), uvs, GL_STATIC_DRAW);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(2);
    }

    // Index buffer
    if (indices && idx_count > 0) {
        glGenBuffers(1, &ebo_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx_count * sizeof(uint32_t), indices, GL_STATIC_DRAW);
        index_count_ = idx_count;
    } else {
        index_count_ = vert_count;
    }

    glBindVertexArray(0);
}

void GpuMesh::draw() const {
    if ((vao_ != 0U) && (index_count_ > 0)) {
        glBindVertexArray(vao_);
        if (ebo_ != 0U) {
            glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
        } else {
            glDrawArrays(GL_TRIANGLES, 0, index_count_);
        }
        glBindVertexArray(0);
    }
}

void GpuMesh::draw_lines() const {
    if ((vao_ != 0U) && (index_count_ > 0)) {
        glBindVertexArray(vao_);
        if (ebo_ != 0U) {
            glDrawElements(GL_LINES, index_count_, GL_UNSIGNED_INT, nullptr);
        } else {
            glDrawArrays(GL_LINES, 0, index_count_);
        }
        glBindVertexArray(0);
    }
}
