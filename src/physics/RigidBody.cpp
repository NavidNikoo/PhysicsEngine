#include "RigidBody.h"



RigidBody::RigidBody(float m, const glm::vec3& pos)
    : mass(m), position(pos), velocity(0.0f), forces(0.0f) {}

void RigidBody::ApplyForce(const glm::vec3& force) {
    forces += force;
}

void RigidBody::Integrate(float deltaTime) {
    if (mass <= 0.0f) return; // Avoid division by zero

    glm::vec3 acceleration = forces / mass;
    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;

    // Clear forces after integration
    forces = glm::vec3(0.0f);
}

bool CheckAABBCollision(const AABB& a, const AABB& b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
           (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
           (a.min.z <= b.max.z && a.max.z >= b.min.z);
}