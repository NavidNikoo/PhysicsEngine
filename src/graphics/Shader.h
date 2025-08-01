#pragma once

#include <string>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>

class Shader {
public:
    unsigned int ID;

    Shader(const char* vertexPath, const char* fragmentPath);
    void use();
    void setMat4(const std::string &name, const float* value);
    void setVec3(const std::string& name, const glm::vec3& value) {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }

private:
    void checkCompileErrors(unsigned int shader, const std::string& type);
};
