#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include "Shader.h"
#include "physics/collision/AABB.h"

class Renderer {
public:
    Renderer();
    ~Renderer();

    Shader wireShader;

    void DrawCube(const glm::mat4& transform, const glm::vec3& color, Shader& shader);
    void DrawPlane(const glm::mat4& transform, Shader& shader);
    void DrawWireAABB(const AABB& aabb, const glm::vec3& color, const glm::mat4& viewProj);

private:
    unsigned int cubeVAO, cubeVBO;
    unsigned int planeVAO, planeVBO;

    unsigned int wireVAO, wireVBO, wireEBO; // For persistent debug AABB rendering
};
