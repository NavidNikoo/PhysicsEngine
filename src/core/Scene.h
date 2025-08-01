#pragma once

#include <vector>
#include <GLFW/glfw3.h>
#include "physics/bodies/RigidBody.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "physics/collision/ContactSolver.h"
#include "physics/collision/ContactManifold.h"

class Scene {
public:
    Scene();
    ContactSolver solver;

    void StepPhysics(float dt);
    void Render(Renderer& renderer, Shader& shader);
    void RenderDebug(Renderer& renderer, const glm::mat4& viewProj);
    void HandleInput(GLFWwindow* window);
    void StepPhysicsWithSubdivision(float dt);
    bool WouldTunnel(const RigidBody& body, float dt);
    void RenderContactPoints(Renderer& renderer, const glm::mat4& viewProj);

private:
    std::vector<RigidBody> bodies;
    std::vector<ContactManifold> lastFrameManifolds;

    // ✅ Fix: Declare the correct collision function
    void ResolveCollision(RigidBody& a, RigidBody& b, const glm::vec3& overlap);
};