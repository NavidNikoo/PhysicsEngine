#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <vector>
#include <numeric>   // for std::iota
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>  // or any other glm/gtx/ include
#include <glm/gtx/string_cast.hpp>  // Needed for glm::to_string

#include "Scene.h"
#include "physics/collision/ContactSolver.h"
#include "physics/shapes/BoxShape.h"
#include "physics/collision/ContactManifold.h"
#include "physics/collision/SATCollision.h"
#include <cmath>
#include <iostream>

Scene::Scene() {
    RigidBody floor(0.0f, glm::vec3(0.0f, -1.0f, 0.0f));
    floor.SetShapeAndSize(glm::vec3(40.0f, 2.0f, 40.0f));
    floor.color = glm::vec3(0.3f, 0.8f, 0.3f);
    floor.isStatic = true;
    floor.velocity = glm::vec3(0.0f);
    floor.hasAwakened = true;  // ✅ mark floor as always awakened
    floor.orientation = glm::quat(1, 0, 0, 0);              // 🔒 Reset rotation
    floor.angularVelocity = glm::vec3(0.0f);                // 🔒 Prevent rotation
    floor.inverseInertiaTensor = glm::mat3(0.0f);           // 🔒 Freeze inertia
    /*AABB floorAABB = floor.GetAABB();
    std::cout << "Floor AABB: Min=" << glm::to_string(floorAABB.min)
              << ", Max=" << glm::to_string(floorAABB.max) << "\n";*/
    bodies.push_back(floor);
}

void Scene::StepPhysics(float dt) {
    dt = std::clamp(dt, 0.001f, 0.016f);

    std::cout << "\n====================[ StepPhysics ]====================\n";
    std::cout << "Bodies in scene: " << bodies.size() << " | dt = " << dt << "\n";

    for (RigidBody& body : bodies) {
        if (body.isStatic || body.isSleeping) continue;

        const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
        body.ApplyForce(gravity * body.mass);

        body.IntegrateVelocity(dt);
        body.IntegrateAngularVelocity(dt);
        body.IntegrateOrientation(dt);
    }

    const int solverIterations = 15;
    std::vector<size_t> order(bodies.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t i, size_t j) {
        return bodies[i].GetAABB().min.x < bodies[j].GetAABB().min.x;
    });

    std::vector<ContactManifold> manifolds;
    int potentialPairs = 0, aabbPass = 0, satPass = 0;

    for (int i = 0; i < bodies.size(); ++i) {
        for (int j = i + 1; j < bodies.size(); ++j) {
            ++potentialPairs;
            const AABB& aabbA = bodies[i].GetAABB();
            const AABB& aabbB = bodies[j].GetAABB();

            if (!aabbA.Overlaps(aabbB)) {
                std::cout << "❌ AABB rejected: body " << i << " and body " << j << "\n";
                continue;
            }

            ++aabbPass;
            std::cout << "✅ AABB overlap: body " << i << " and body " << j << "\n";

            ContactManifold m = SATCollision::DetectCollision(bodies[i], bodies[j]);
            if (m.hasCollision) {
                ++satPass;
                std::cout << "✅ SAT collision detected. Contacts: " << m.contacts.size()
                          << ", Penetration: " << m.penetration << "\n";
                manifolds.push_back(m);
            } else {
                std::cout << "❌ SAT no collision for body " << i << " and body " << j << "\n";
            }
        }
    }

    std::cout << "🔍 Pair checks: " << potentialPairs
              << ", AABB pass: " << aabbPass
              << ", SAT pass: " << satPass << "\n";

    for (ContactManifold& manifold : manifolds) {
        if (!manifold.hasCollision || manifold.contacts.empty()) {
            std::cout << "🚫 Skipped contact resolution (no contacts)\n";
            continue;
        } else {
            std::cout << "✅ Resolving manifold with " << manifold.contacts.size()
                      << " contacts, Penetration = " << manifold.penetration << "\n";
        }

        for (const ContactPoint& cp : manifold.contacts) {
            std::cout << "🧩 Contact | Point: " << glm::to_string(cp.point)
                      << ", Normal: " << glm::to_string(cp.normal)
                      << ", Pen: " << cp.penetration << "\n";

            solver.Resolve(*manifold.a, *manifold.b, cp.point, cp.normal, cp.penetration, dt);
        }
    }

    for (RigidBody& body : bodies) {
        if (body.isStatic || body.isSleeping) continue;

        body.IntegratePosition(dt);
        body.IntegrateOrientation(dt);

        float velSq = glm::length2(body.velocity);
        float angVelSq = glm::length2(body.angularVelocity);
        const float velTol = 0.0001f;
        const float angVelTol = 0.0001f;

        if (!body.hasAwakened && (velSq > 0.001f || angVelSq > 0.001f)) {
            body.hasAwakened = true;
            body.color = glm::vec3(
                0.2f + static_cast<float>(rand()) / RAND_MAX * 0.8f,
                0.2f + static_cast<float>(rand()) / RAND_MAX * 0.8f,
                0.2f + static_cast<float>(rand()) / RAND_MAX * 0.8f
            );
        }

        if (velSq < velTol && angVelSq < angVelTol) {
            body.sleepCounter++;
            if (body.sleepCounter > body.sleepCounterThreshold) {
                body.isSleeping = true;
                body.velocity = glm::vec3(0.0f);
                body.angularVelocity = glm::vec3(0.0f);
                std::cout << "💤 Body sleeping\n";
            }
        } else {
            body.sleepCounter = 0;
        }
    }

    std::cout << "====================[ End StepPhysics ]====================\n";
}


void Scene::Render(Renderer& renderer, Shader& shader) {
    for (RigidBody& body : bodies) {
        glm::vec3 size = body.shape ? body.shape->GetSize() : glm::vec3(1.0f);  // FIXED HERE
        glm::mat4 model = glm::translate(glm::mat4(1.0f), body.position);
        model = glm::scale(model, size);
        model *= glm::toMat4(body.orientation); // Add rotation
        glm::vec3 renderColor = (!body.hasAwakened)
            ? glm::vec3(0.7f)  // force gray before awake
            : (body.isSleeping ? body.color * 0.3f : body.color);
        renderer.DrawCube(model, renderColor, shader);
    }
}

void Scene::RenderDebug(Renderer& renderer, const glm::mat4& viewProj) {
    for (RigidBody& body : bodies) {
        AABB aabb = body.GetAABB();
        renderer.DrawWireAABB(aabb, glm::vec3(1.0f, 1.0f, 0.0f), viewProj);
    }
}

void Scene::HandleInput(GLFWwindow* window) {
    static bool spacePressedLastFrame = false;
    static bool rPressedLastFrame = false;

    bool spacePressed = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    bool rPressed = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;

    if (spacePressed && !spacePressedLastFrame) {
        float x = ((rand() % 200) - 100) / 50.0f;
        float z = ((rand() % 200) - 100) / 50.0f;
        float y = 6.0f + (float)bodies.size() * 1.2f;

        glm::vec3 fullSize(1.0f);
        glm::vec3 halfExtents = fullSize * 0.5f;

        RigidBody box(1.0f, glm::vec3(x, y, z), fullSize);
        box.SetShapeAndSize(fullSize);
        box.hasAwakened = false;
        bodies.push_back(box);
    }

    if (rPressed && !rPressedLastFrame) {
        bodies.clear();

        RigidBody floor(0.0f, glm::vec3(0.0f, -1.0f, 0.0f));
        floor.size = glm::vec3(40.0f, 2.0f, 40.0f);
        floor.shape = std::make_shared<BoxShape>(floor.size * 0.5f);
        floor.color = glm::vec3(0.3f, 0.8f, 0.3f);
        floor.isStatic = true;
        floor.velocity = glm::vec3(0.0f);
        floor.orientation = glm::quat(1, 0, 0, 0);              // 🔒 Lock rotation
        floor.angularVelocity = glm::vec3(0.0f);                // 🔒 No angular motion
        floor.inverseInertiaTensor = glm::mat3(0.0f);           // 🔒 No rotatione
        floor.staticFriction = 0.9f;
        floor.dynamicFriction = 0.8f;
        floor.hasAwakened = true;
        bodies.push_back(floor);
    }

    if (bodies.size() == 1) {  // If only floor exists
        glm::vec3 fullSize(1.0f);
        glm::vec3 halfExtents = fullSize * 0.5f;
        glm::vec3 startPos = glm::vec3(0.0f, 0.0f, 0.0f);  // On floor

        RigidBody box(1.0f, startPos, fullSize);
        box.shape = std::make_shared<BoxShape>(halfExtents);
        box.hasAwakened = false;
        bodies.push_back(box);
        std::cout << "🚀 Spawned test box at " << glm::to_string(startPos) << "\n";
    }


    spacePressedLastFrame = spacePressed;
    rPressedLastFrame = rPressed;
}

