#include "RigidBody.h"
#include "physics/shapes/BoxShape.h"
#include <cstdlib>
#include <iostream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <cfloat>

// === Damping & Limits (easy to tune) ===
constexpr float LINEAR_DAMPING = 0.99f;
constexpr float ANGULAR_DAMPING = 0.95f;
constexpr float MAX_LINEAR_VELOCITY = 8.0f;
constexpr float MAX_ANGULAR_VELOCITY = 5.0f;
constexpr float SLEEP_THRESHOLD = 0.01f;
constexpr float ANGULAR_SLEEP_THRESHOLD = 0.01f;

RigidBody::RigidBody()
    : mass(1.0f), position(0.0f), velocity(0.0f), forces(0.0f), isStatic(false) {
    size = glm::vec3(1.0f);
    shape = std::make_shared<BoxShape>(size * 0.5f);
    color = glm::vec3(
        (rand() % 100) / 100.0f,
        (rand() % 100) / 100.0f,
        (rand() % 100) / 100.0f
    );
    ComputeInertia();
}

RigidBody::RigidBody(float m, const glm::vec3& pos)
    : mass(m), position(pos), velocity(0.0f), forces(0.0f), isStatic(false) {
    size = glm::vec3(1.0f);
    shape = std::make_shared<BoxShape>(size * 0.5f);
    color = glm::vec3(
        (rand() % 100) / 100.0f,
        (rand() % 100) / 100.0f,
        (rand() % 100) / 100.0f
    );
    ComputeInertia();
}

RigidBody::RigidBody(float m, const glm::vec3& pos, const glm::vec3& sz)
    : mass(m), position(pos), size(sz), velocity(0.0f), forces(0.0f), isStatic(false) {
    shape = std::make_shared<BoxShape>(size * 0.5f);
    color = glm::vec3(
        (rand() % 100) / 100.0f,
        (rand() % 100) / 100.0f,
        (rand() % 100) / 100.0f
    );
    ComputeInertia();
}

void RigidBody::ComputeInertia() {
    if (isStatic || mass <= 0.0f) {
        inverseInertiaTensor = glm::mat3(0.0f);
        return;
    }

    glm::vec3 dims = size;
    float ix = (1.0f / 12.0f) * mass * (dims.y * dims.y + dims.z * dims.z);
    float iy = (1.0f / 12.0f) * mass * (dims.x * dims.x + dims.z * dims.z);
    float iz = (1.0f / 12.0f) * mass * (dims.x * dims.x + dims.y * dims.y);
    inertiaTensor = glm::mat3(
        ix, 0, 0,
        0, iy, 0,
        0, 0, iz
    );
    inverseInertiaTensor = glm::inverse(inertiaTensor);
}

void RigidBody::ApplyForce(const glm::vec3& force) {
    forces += force;
    isSleeping = false;
}

void RigidBody::ApplyTorque(const glm::vec3& t) {
    if (!isStatic) torque += t;
}

void RigidBody::IntegrateVelocity(float dt) {
    if (isStatic) return;

    // Linear acceleration and integration
    glm::vec3 acceleration = forces / mass;
    velocity += acceleration * dt;

    // Exponential linear damping
    velocity *= std::pow(LINEAR_DAMPING, dt);

    // Cap max linear velocity
    const float maxVelSq = MAX_LINEAR_VELOCITY * MAX_LINEAR_VELOCITY;
    if (glm::length2(velocity) > maxVelSq)
        velocity = glm::normalize(velocity) * MAX_LINEAR_VELOCITY;

    // Clamp tiny velocities to zero for stability
    if (glm::length2(velocity) < SLEEP_THRESHOLD * SLEEP_THRESHOLD)
        velocity = glm::vec3(0.0f);

    // NaN check
    if (glm::any(glm::isnan(velocity))) {
        std::cout << "⚠️ NaN linear velocity! Resetting...\n";
        velocity = glm::vec3(0.0f);
    }

    // Reset forces
    forces = glm::vec3(0.0f);
}

void RigidBody::IntegrateAngularVelocity(float dt) {
    if (isStatic) return;

    // Convert local inverse inertia tensor to world space
    glm::mat3 R = glm::toMat3(orientation);
    glm::mat3 worldInvInertia = R * inverseInertiaTensor * glm::transpose(R);

    glm::vec3 angAccel = worldInvInertia * torque;
    angularVelocity += angAccel * dt;

    // ✅ Exponential angular damping
    angularVelocity *= std::exp(-ANGULAR_DAMPING * dt);

    // Clamp angular velocity to prevent explosion
    if (glm::length2(angularVelocity) > MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY)
        angularVelocity = glm::normalize(angularVelocity) * MAX_ANGULAR_VELOCITY;

    if (glm::length2(angularVelocity) < ANGULAR_SLEEP_THRESHOLD * ANGULAR_SLEEP_THRESHOLD)
        angularVelocity = glm::vec3(0.0f);

    if (glm::any(glm::isnan(angularVelocity))) {
        std::cout << "NaN angular velocity! Resetting...\n";
        angularVelocity = glm::vec3(0.0f);
    }

    torque = glm::vec3(0.0f);
}

void RigidBody::IntegratePosition(float dt) {
    if (isStatic) return;

    position += velocity * dt;

    if (position.y < -100.0f || glm::any(glm::isnan(position))) {
        std::cout << "NaN or out-of-bounds position! Resetting...\n";
        velocity = glm::vec3(0.0f);
        angularVelocity = glm::vec3(0.0f);
        position = glm::vec3(0.0f, 5.0f, 0.0f);
    }
}

void RigidBody::IntegrateOrientation(float dt) {
    if (isStatic) return;

    glm::quat deltaRot = glm::quat(0.0f, angularVelocity * dt) * orientation;
    orientation += 0.5f * deltaRot;
    orientation = glm::normalize(orientation);
}

AABB RigidBody::GetAABB() const {
    if (!shape) return AABB(position, position);

    glm::vec3 halfExtents = shape->halfExtents;
    glm::mat3 rot = glm::toMat3(orientation);

    glm::vec3 absRot[3] = {
        glm::abs(rot[0]),
        glm::abs(rot[1]),
        glm::abs(rot[2])
    };

    glm::vec3 worldHalfExtents =
        absRot[0] * halfExtents.x +
        absRot[1] * halfExtents.y +
        absRot[2] * halfExtents.z;

    glm::vec3 aabbMin = position - worldHalfExtents;
    glm::vec3 aabbMax = position + worldHalfExtents;

    return AABB(aabbMin, aabbMax);
}


void RigidBody::SetShapeAndSize(const glm::vec3& fullSize) {
    size = fullSize;
    shape = std::make_shared<BoxShape>(size * 0.5f);
    ComputeInertia();
}