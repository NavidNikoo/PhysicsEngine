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

    // 1. APPLY FORCES AND INTEGRATE VELOCITIES FIRST
    for (RigidBody& body : bodies) {
        if (body.isStatic || body.isSleeping) continue;

        const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
        body.ApplyForce(gravity * body.mass);

        // Integrate velocities (but NOT positions yet)
        body.IntegrateVelocity(dt);
        body.IntegrateAngularVelocity(dt);

        // DEBUG: Check for fast-moving objects
        float speed = glm::length(body.velocity);
        if (speed > 10.0f) {
            std::cout << "⚠️ Fast object detected: speed=" << speed
                      << " pos=" << glm::to_string(body.position) << std::endl;
        }
    }

    // 2. COLLISION DETECTION AND RESPONSE
    std::vector<ContactManifold> manifolds;
    int potentialPairs = 0, aabbPass = 0, satPass = 0;

    for (int i = 0; i < bodies.size(); ++i) {
        for (int j = i + 1; j < bodies.size(); ++j) {
            ++potentialPairs;
            const AABB& aabbA = bodies[i].GetAABB();
            const AABB& aabbB = bodies[j].GetAABB();

            if (!aabbA.Overlaps(aabbB)) {
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
            }
        }
    }

    std::cout << "🔍 Pair checks: " << potentialPairs
              << ", AABB pass: " << aabbPass
              << ", SAT pass: " << satPass << "\n";

    // 3. RESOLVE COLLISIONS
    for (ContactManifold& manifold : manifolds) {
        if (!manifold.hasCollision || manifold.contacts.empty()) continue;

        for (const ContactPoint& cp : manifold.contacts) {
            solver.Resolve(*manifold.a, *manifold.b, cp.point, cp.normal, cp.penetration, dt);
        }
    }

    lastFrameManifolds = manifolds;
    // 4. INTEGRATE POSITIONS ONLY ONCE (AFTER collision resolution)
    for (RigidBody& body : bodies) {
        if (body.isStatic || body.isSleeping) continue;

        // NOW integrate positions with the corrected velocities
        body.IntegratePosition(dt);
        body.IntegrateOrientation(dt);

        // Sleep/wake logic
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

// Add this method to your Scene class
void Scene::StepPhysicsWithSubdivision(float dt) {
    dt = std::clamp(dt, 0.001f, 0.016f);

    // Check if any object is moving too fast
    float maxSpeed = 0.0f;
    for (const RigidBody& body : bodies) {
        if (!body.isStatic) {
            maxSpeed = std::max(maxSpeed, glm::length(body.velocity));
        }
    }

    // If objects are moving too fast, subdivide the timestep
    int subdivisions = 1;
    const float maxSafeSpeed = 3.0f; // Lower threshold for better collision detection
    if (maxSpeed > maxSafeSpeed) {
        subdivisions = static_cast<int>(std::ceil(maxSpeed / maxSafeSpeed));
        subdivisions = std::min(subdivisions, 8); // Increase max subdivisions
        std::cout << "⚡ Fast motion detected (" << maxSpeed << " m/s), using "
                  << subdivisions << " subdivisions\n";
    }

    float subDt = dt / subdivisions;
    for (int i = 0; i < subdivisions; i++) {
        StepPhysics(subDt);
    }
}

// Add this method to your Scene class to check for potential tunneling
bool Scene::WouldTunnel(const RigidBody& body, float dt) {
    if (body.isStatic) return false;

    float speed = glm::length(body.velocity);
    float minDimension = std::min({body.size.x, body.size.y, body.size.z});
    float distanceThisFrame = speed * dt;

    // If object moves more than half its size in one frame, it might tunnel
    return distanceThisFrame > (minDimension * 0.5f);
}

void Scene::RenderContactPoints(Renderer& renderer, const glm::mat4& viewProj) {
    // Store contact points from the last physics step
    for (const ContactManifold& manifold : lastFrameManifolds) {
        for (const ContactPoint& cp : manifold.contacts) {
            // Draw a small red sphere at each contact point
            glm::mat4 contactTransform = glm::translate(glm::mat4(1.0f), cp.point);
            contactTransform = glm::scale(contactTransform, glm::vec3(0.1f)); // Small sphere

            // You can implement this as a small cube or sphere
            // renderer.DrawDebugPoint(cp.point, glm::vec3(1.0f, 0.0f, 0.0f), viewProj);

            // Also draw the normal as a line
            glm::vec3 normalEnd = cp.point + cp.normal * 0.5f;
            // renderer.DrawDebugLine(cp.point, normalEnd, glm::vec3(0.0f, 1.0f, 0.0f), viewProj);
        }
    }
}