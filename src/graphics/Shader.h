#pragma once

#include <string>
#include <glad/glad.h>

class Shader {
public:
    unsigned int ID;

    Shader(const char* vertexPath, const char* fragmentPath);
    void use();
    void setMat4(const std::string &name, const float* value);

private:
    void checkCompileErrors(unsigned int shader, const std::string& type);
};
