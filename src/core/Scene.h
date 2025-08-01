#pragma once

#include <vector>
#include <GLFW/glfw3.h>
#include "physics/bodies/RigidBody.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "physics/collision/ContactSolver.h"

class Scene {
public:
    Scene();
    ContactSolver solver;

    void StepPhysics(float dt);
    void Render(Renderer& renderer, Shader& shader);
    void RenderDebug(Renderer& renderer, const glm::mat4& viewProj);
    void HandleInput(GLFWwindow* window);

private:
    std::vector<RigidBody> bodies;

    // ✅ Fix: Declare the correct collision function
    void ResolveCollision(RigidBody& a, RigidBody& b, const glm::vec3& overlap);
};