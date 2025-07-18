#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include "Shader.h"

class Renderer {
public:
    Renderer();
    void DrawCube(const glm::mat4& transform, Shader& shader);
private:
    unsigned int VAO, VBO;
};
