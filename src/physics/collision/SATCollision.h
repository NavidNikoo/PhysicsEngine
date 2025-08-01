#pragma once

#include "physics/bodies/RigidBody.h"
#include "physics/collision/ContactManifold.h"

class SATCollision {
public:
    static ContactManifold DetectCollision(const RigidBody& a, const RigidBody& b);
    void ProjectBoxOntoAxis(const RigidBody& body, const glm::vec3& axis, float& min, float& max);

private:
    static int Clip(const glm::vec3& n, float c, glm::vec3* faceIn, glm::vec3* faceOut);
    static void ComputeIncidentFace(const glm::vec3& normal, const RigidBody& incBody, glm::vec3* incidentVerts);
};
