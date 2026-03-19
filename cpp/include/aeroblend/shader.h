#ifndef AEROBLEND_SHADER_H
#define AEROBLEND_SHADER_H

#include <glad/gl.h>
#include <string>

class Shader {
public:
    Shader() = default;
    ~Shader();

    bool load(const std::string& vert_path, const std::string& frag_path);
    void use() const;

    void set_mat4(const char* name, const float* m) const;
    void set_mat3(const char* name, const float* m) const;
    void set_vec3(const char* name, float x, float y, float z) const;
    void set_float(const char* name, float v) const;
    void set_int(const char* name, int v) const;

    GLuint id() const { return program_; }

private:
    GLuint program_ = 0;
    GLuint compile(GLenum type, const char* src);
};

#endif // AEROBLEND_SHADER_H
