// ===== ContactManifold.h =====
#pragma once

#include <glm/glm.hpp>
#include <vector>

struct ContactPoint {
    glm::vec3 point;
    glm::vec3 normal;
    float penetration;
};

struct ContactManifold {
    bool hasCollision = false;
    float penetration = 0.0f;
    glm::vec3 normal;
    std::vector<ContactPoint> contacts;

    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
};
