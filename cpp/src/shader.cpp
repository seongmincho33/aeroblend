#include <aeroblend/shader.h>
#include <cstdio>
#include <fstream>
#include <sstream>

Shader::~Shader() {
    if (program_ != 0U) { glDeleteProgram(program_); }
}

namespace {

std::string read_file(const std::string& path) {
    std::string result;
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "Cannot open shader: %s\n", path.c_str());
    } else {
        std::stringstream buf;
        buf << f.rdbuf();
        result = buf.str();
    }
    return result;
}

} // namespace

GLuint Shader::compile(GLenum type, const char* src) {
    GLuint result = glCreateShader(type);
    glShaderSource(result, 1, &src, nullptr);
    glCompileShader(result);
    GLint ok = 0;
    glGetShaderiv(result, GL_COMPILE_STATUS, &ok);
    if (ok == 0) {
        char log[512];
        glGetShaderInfoLog(result, sizeof(log), nullptr, log);
        fprintf(stderr, "Shader compile error:\n%s\n", log);
        glDeleteShader(result);
        result = 0U;
    }
    return result;
}

bool Shader::load(const std::string& vert_path, const std::string& frag_path) {
    bool success = false;
    std::string vs_src = read_file(vert_path);
    std::string fs_src = read_file(frag_path);

    if (!vs_src.empty() && !fs_src.empty()) {
        GLuint vs = compile(GL_VERTEX_SHADER, vs_src.c_str());
        GLuint fs = compile(GL_FRAGMENT_SHADER, fs_src.c_str());

        if ((vs != 0U) && (fs != 0U)) {
            program_ = glCreateProgram();
            glAttachShader(program_, vs);
            glAttachShader(program_, fs);
            glLinkProgram(program_);

            GLint ok = 0;
            glGetProgramiv(program_, GL_LINK_STATUS, &ok);
            if (ok == 0) {
                char log[512];
                glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
                fprintf(stderr, "Shader link error:\n%s\n", log);
            }
            success = (ok != 0);
        }

        if (vs != 0U) { glDeleteShader(vs); }
        if (fs != 0U) { glDeleteShader(fs); }
    }
    return success;
}

void Shader::use() const {
    glUseProgram(program_);
}

void Shader::set_mat4(const char* name, const float* m) const {
    glUniformMatrix4fv(glGetUniformLocation(program_, name), 1, GL_FALSE, m);
}

void Shader::set_mat3(const char* name, const float* m) const {
    glUniformMatrix3fv(glGetUniformLocation(program_, name), 1, GL_FALSE, m);
}

void Shader::set_vec3(const char* name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(program_, name), x, y, z);
}

void Shader::set_float(const char* name, float v) const {
    glUniform1f(glGetUniformLocation(program_, name), v);
}

void Shader::set_int(const char* name, int v) const {
    glUniform1i(glGetUniformLocation(program_, name), v);
}
