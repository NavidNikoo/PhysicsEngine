#ifndef CONTACT_SOLVER_H
#define CONTACT_SOLVER_H

#include "physics/bodies/RigidBody.h"
#include <glm/glm.hpp>
#include "AABB.h"

class ContactSolver {
public:
    void Resolve(RigidBody& a, RigidBody& b, const glm::vec3& contactPoint, const glm::vec3& normalInput, float penetration, float deltaTime);
    static bool AABBOverlap(const AABB& a, const AABB& b);
};

#endif
