#include "ContactSolver.h"
#include <glm/glm.hpp>
#include <iostream>
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

void ContactSolver::Resolve(RigidBody& a, RigidBody& b, const glm::vec3& contactPoint, const glm::vec3& normalInput, float penetration, float deltaTime) {
    if (penetration <= 0.0f) return;
    if (a.isStatic && b.isStatic) return;

    // Tuned parameters
    const float restitution = 0.05f;
    const float baumgarte = 0.8f;
    const float slop = 0.0001f;
    const float maxCorrection = 0.5f;

    float invMassA = a.isStatic ? 0.0f : 1.0f / a.mass;
    float invMassB = b.isStatic ? 0.0f : 1.0f / b.mass;

    glm::vec3 normal = glm::normalize(normalInput);
    glm::vec3 ra = contactPoint - a.position;
    glm::vec3 rb = contactPoint - b.position;

    glm::vec3 va = a.velocity + glm::cross(a.angularVelocity, ra);
    glm::vec3 vb = b.velocity + glm::cross(b.angularVelocity, rb);
    glm::vec3 relVel = vb - va;

    float velAlongNormal = glm::dot(relVel, normal);

    glm::vec3 raCrossN = glm::cross(ra, normal);
    glm::vec3 rbCrossN = glm::cross(rb, normal);

    float angularTerm = glm::dot(raCrossN, a.inverseInertiaTensor * raCrossN) +
                        glm::dot(rbCrossN, b.inverseInertiaTensor * rbCrossN);
    float invMassSum = invMassA + invMassB + angularTerm;
    if (invMassSum == 0.0f) return;

    // --- Bias and Impulse ---
    float bias = baumgarte * std::max(penetration - slop, 0.0f) / deltaTime;
    float j = -(1.0f + restitution) * velAlongNormal + bias;
    j /= invMassSum;
    glm::vec3 impulse = j * normal;

    std::cout << "⚡ Contact | Pen=" << penetration
              << " j=" << j
              << " Normal=" << glm::to_string(normal)
              << " ContactPoint=" << glm::to_string(contactPoint) << "\n";

    if (!a.isStatic) {
        a.velocity -= impulse * invMassA;
        a.angularVelocity -= a.inverseInertiaTensor * glm::cross(ra, impulse);
    }
    if (!b.isStatic) {
        b.velocity += impulse * invMassB;
        b.angularVelocity += b.inverseInertiaTensor * glm::cross(rb, impulse);
    }

    // --- Friction ---
    float staticFriction = std::sqrt(a.staticFriction * b.staticFriction);
    float dynamicFriction = std::sqrt(a.dynamicFriction * b.dynamicFriction);

    glm::vec3 newRelVel = (b.velocity + glm::cross(b.angularVelocity, rb)) -
                          (a.velocity + glm::cross(a.angularVelocity, ra));
    glm::vec3 tangent = newRelVel - glm::dot(newRelVel, normal) * normal;

    if (glm::length2(tangent) > 1e-6f) {
        tangent = glm::normalize(tangent);
        float jt = -glm::dot(newRelVel, tangent);
        jt /= invMassSum;

        glm::vec3 frictionImpulse;
        float maxFriction = j * staticFriction;

        if (std::abs(jt) < maxFriction) {
            frictionImpulse = jt * tangent;
        } else {
            frictionImpulse = -dynamicFriction * j * tangent;
        }

        if (!a.isStatic) {
            a.velocity -= frictionImpulse * invMassA;
            a.angularVelocity -= a.inverseInertiaTensor * glm::cross(ra, frictionImpulse);
        }
        if (!b.isStatic) {
            b.velocity += frictionImpulse * invMassB;
            b.angularVelocity += b.inverseInertiaTensor * glm::cross(rb, frictionImpulse);
        }
    }

    // --- Position Correction ---
    float correctionMag = baumgarte * std::max(penetration - slop, 0.0f) / invMassSum;
    correctionMag = glm::clamp(correctionMag, 0.0001f, maxCorrection);  // Minimum clamp
    glm::vec3 correction = correctionMag * normal;

    std::cout << "🛠️ Correction | Mag=" << correctionMag
              << " Correction=" << glm::to_string(correction) << "\n\n";

    if (!a.isStatic) a.position -= correction * invMassA;
    if (!b.isStatic) b.position += correction * invMassB;

    // --- Wake ---
    if (a.isSleeping) {
        a.isSleeping = false;
        a.sleepCounter = 0;
    }
    if (b.isSleeping) {
        b.isSleeping = false;
        b.sleepCounter = 0;
    }
}

bool ContactSolver::AABBOverlap(const AABB& a, const AABB& b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
           (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
           (a.min.z <= b.max.z && a.max.z >= b.min.z);
}
