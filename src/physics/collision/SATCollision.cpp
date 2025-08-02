#include "SATCollision.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <limits>
#include <vector>
#include <iostream>
#include <string>

namespace {
    void ProjectBoxOntoAxis(const RigidBody& body, const glm::vec3& axis, float& min, float& max) {
        if (!body.shape) {
            std::cerr << "🚨 ERROR: ProjectBoxOntoAxis() — shape is null!\n";
            min = max = 0.0f;
            return;
        }

        glm::mat3 rot = glm::toMat3(body.orientation);
        glm::vec3 center = body.position;
        glm::vec3 halfExtents = body.shape->halfExtents;

        std::vector<glm::vec3> corners = {
            {+halfExtents.x, +halfExtents.y, +halfExtents.z},
            {-halfExtents.x, +halfExtents.y, +halfExtents.z},
            {-halfExtents.x, -halfExtents.y, +halfExtents.z},
            {+halfExtents.x, -halfExtents.y, +halfExtents.z},
            {+halfExtents.x, +halfExtents.y, -halfExtents.z},
            {-halfExtents.x, +halfExtents.y, -halfExtents.z},
            {-halfExtents.x, -halfExtents.y, -halfExtents.z},
            {+halfExtents.x, -halfExtents.y, -halfExtents.z}
        };

        min = std::numeric_limits<float>::infinity();
        max = -std::numeric_limits<float>::infinity();

        for (const auto& corner : corners) {
            glm::vec3 worldCorner = center + rot * corner;
            float projection = glm::dot(worldCorner, axis);
            min = std::min(min, projection);
            max = std::max(max, projection);
        }
    }

    float GetOverlapOnAxis(const RigidBody& a, const RigidBody& b, const glm::vec3& axis) {
        if (glm::length2(axis) < 1e-6f) return -1.0f;
        glm::vec3 normAxis = glm::normalize(axis);
        float minA, maxA, minB, maxB;
        ProjectBoxOntoAxis(a, normAxis, minA, maxA);
        ProjectBoxOntoAxis(b, normAxis, minB, maxB);
        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        return (overlap > 0) ? overlap : -1.0f;
    }

    std::vector<glm::vec3> GetBoxAxes(const glm::quat& orientation) {
        glm::mat3 rot = glm::toMat3(orientation);
        return { rot[0], rot[1], rot[2] };
    }
}

ContactManifold SATCollision::DetectCollision(const RigidBody& a, const RigidBody& b) {
    ContactManifold manifold;
    manifold.a = (RigidBody*)&a;
    manifold.b = (RigidBody*)&b;

    float minOverlap = std::numeric_limits<float>::infinity();
    glm::vec3 smallestAxis;
    bool shouldFlip = false;

    auto aAxes = GetBoxAxes(a.orientation);
    auto bAxes = GetBoxAxes(b.orientation);

    std::vector<glm::vec3> axes = aAxes;
    axes.insert(axes.end(), bAxes.begin(), bAxes.end());

    for (const glm::vec3& aAxis : aAxes) {
        for (const glm::vec3& bAxis : bAxes) {
            glm::vec3 cross = glm::cross(aAxis, bAxis);
            if (glm::length2(cross) > 1e-6f)
                axes.push_back(glm::normalize(cross));
        }
    }

    // SAT test: find axis of least penetration
    for (const glm::vec3& axis : axes) {
        float overlap = GetOverlapOnAxis(a, b, axis);
        if (overlap < 0.0f) {
            manifold.hasCollision = false;
            return manifold;  // Separating axis found
        }
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = glm::normalize(axis);
            shouldFlip = glm::dot(b.position - a.position, axis) < 0.0f;
        }
    }

    if (minOverlap <= 0.0f || glm::length2(smallestAxis) < 1e-5f) {
        manifold.hasCollision = false;
        return manifold;
    }

    glm::vec3 collisionNormal = shouldFlip ? -smallestAxis : smallestAxis;
    manifold.penetration = minOverlap;
    manifold.normal = collisionNormal;

    // === Generate multiple contact points using corners of B ===
    glm::mat3 rotB = glm::toMat3(b.orientation);
    glm::vec3 halfB = b.shape->halfExtents;

    std::vector<glm::vec3> corners;
    for (int x = -1; x <= 1; x += 2) {
        for (int y = -1; y <= 1; y += 2) {
            for (int z = -1; z <= 1; z += 2) {
                glm::vec3 localCorner = glm::vec3(x, y, z) * halfB;
                glm::vec3 worldCorner = b.position + rotB * localCorner;
                corners.push_back(worldCorner);
            }
        }
    }

    float contactThreshold = 0.05f;

    // Surface point of A (top surface along normal)
    glm::vec3 planePoint = a.position + collisionNormal * glm::dot(a.shape->halfExtents, collisionNormal);

    for (const auto& corner : corners) {
        float dist = glm::dot(corner - planePoint, collisionNormal);
        float penetration = -dist;

        if (penetration >= -contactThreshold) {
            ContactPoint cp;
            cp.point = corner;
            cp.normal = collisionNormal;
            cp.penetration = penetration;
            manifold.contacts.push_back(cp);
        }
    }

    if (!manifold.contacts.empty()) {
        manifold.hasCollision = true;
        std::cout << "📌 Generated " << manifold.contacts.size()
                  << " contacts from box corners.\n";
    } else {
        manifold.hasCollision = false;
    }

    std::cout << "🎯 SAT Collision detected!\n";
    std::cout << "   Bodies: A=" << glm::to_string(a.position) << " B=" << glm::to_string(b.position) << std::endl;
    std::cout << "   Penetration: " << manifold.penetration << std::endl;
    std::cout << "   Normal: " << glm::to_string(manifold.normal) << std::endl;
    std::cout << "   Contact count: " << manifold.contacts.size() << std::endl;

    return manifold;
}


int SATCollision::Clip(const glm::vec3& n, float c, glm::vec3* input, glm::vec3* output) {
    int count = 0;
    glm::vec3 prev = input[3];
    float prevDist = glm::dot(n, prev) - c;

    for (int i = 0; i < 4; ++i) {
        glm::vec3 curr = input[i];
        float currDist = glm::dot(n, curr) - c;

        if (currDist >= 0.0f) {
            if (prevDist < 0.0f) {
                float t = prevDist / (prevDist - currDist);
                output[count++] = prev + t * (curr - prev);
            }
            output[count++] = curr;
        } else if (prevDist >= 0.0f) {
            float t = prevDist / (prevDist - currDist);
            output[count++] = prev + t * (curr - prev);
        }

        prev = curr;
        prevDist = currDist;
    }

    return count;
}

void SATCollision::ComputeIncidentFace(const glm::vec3& normalWorld, const RigidBody& incBody, glm::vec3* outVerts) {
    glm::mat3 rot = glm::toMat3(incBody.orientation);
    glm::vec3 localNormal = glm::transpose(rot) * -normalWorld;
    glm::vec3 absNormal = glm::abs(localNormal);

    int axis = 0;
    if (absNormal.y > absNormal.x) axis = 1;
    if (absNormal.z > absNormal[axis]) axis = 2;

    glm::vec3 half = incBody.shape->halfExtents;
    float sign = localNormal[axis] < 0 ? 1.0f : -1.0f;

    glm::vec3 faceCenter = glm::vec3(0.0f);
    faceCenter[axis] = sign * half[axis];

    glm::vec3 u(0.0f), v(0.0f);
    u[(axis + 1) % 3] = half[(axis + 1) % 3];
    v[(axis + 2) % 3] = half[(axis + 2) % 3];

    glm::vec3 corners[4] = {
        faceCenter + u + v,
        faceCenter - u + v,
        faceCenter - u - v,
        faceCenter + u - v
    };

    for (int i = 0; i < 4; ++i) {
        outVerts[i] = incBody.position + rot * corners[i];
    }
}