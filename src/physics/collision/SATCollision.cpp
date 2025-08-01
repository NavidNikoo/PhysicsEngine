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

    for (const glm::vec3& axis : axes) {
        float overlap = GetOverlapOnAxis(a, b, axis);
        if (overlap < 0.0f) {
            manifold.hasCollision = false;
            return manifold;
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

    const RigidBody* ref = &a;
    const RigidBody* inc = &b;
    auto refAxes = aAxes;
    auto incAxes = bAxes;
    float maxRefDot = -1.0f;
    int refFaceIdx = 0;
    for (int i = 0; i < 3; ++i) {
        float dotVal = glm::abs(glm::dot(refAxes[i], collisionNormal));
        if (dotVal > maxRefDot) {
            maxRefDot = dotVal;
            refFaceIdx = i;
        }
    }

    glm::vec3 refNormal = refAxes[refFaceIdx] * ((glm::dot(refAxes[refFaceIdx], collisionNormal) > 0) ? 1.0f : -1.0f);


    glm::vec3 incidentVerts[4];
    ComputeIncidentFace(collisionNormal, *inc, incidentVerts);

    glm::mat3 refRot = glm::toMat3(ref->orientation);
    glm::vec3 refHalf = ref->shape->halfExtents;
    glm::vec3 refCenter = ref->position;

    glm::vec3 localNormal = glm::transpose(refRot) * collisionNormal;
    int faceIdx = 0;
    glm::vec3 absNormal = glm::abs(localNormal);
    if (absNormal.y > absNormal.x) faceIdx = 1;
    if (absNormal.z > absNormal[faceIdx]) faceIdx = 2;

    glm::vec3 refPos = refCenter + refNormal * refHalf[faceIdx];

    glm::vec3 refU = refAxes[(faceIdx + 1) % 3];
    glm::vec3 refV = refAxes[(faceIdx + 2) % 3];
    float halfU = refHalf[(faceIdx + 1) % 3];
    float halfV = refHalf[(faceIdx + 2) % 3];

    glm::vec3 clipA = refU;
    float clipAOffset = glm::dot(clipA, refPos + refU * halfU);

    glm::vec3 clipB = -refU;
    float clipBOffset = glm::dot(clipB, refPos - refU * halfU);

    glm::vec3 clipC = refV;
    float clipCOffset = glm::dot(clipC, refPos + refV * halfV);

    glm::vec3 clipD = -refV;
    float clipDOffset = glm::dot(clipD, refPos - refV * halfV);

    glm::vec3 clipped1[4], clipped2[4], clipped3[4], clipped4[4];
    int count = Clip(clipA, clipAOffset, incidentVerts, clipped1);
    count = Clip(clipB, clipBOffset, clipped1, clipped2);
    count = Clip(clipC, clipCOffset, clipped2, clipped3);
    count = Clip(clipD, clipDOffset, clipped3, clipped4);

    if (count == 0) {
        manifold.hasCollision = false;
        return manifold;
    }

    for (int i = 0; i < count; ++i) {
        float depth = glm::dot(refNormal, refPos - clipped4[i]);
        if (!std::isfinite(depth) || depth < 0.0f || depth > minOverlap + 0.1f)
            continue; // skip invalid points
        float paddedDepth = depth + 0.01f;
        if (paddedDepth >= 0.001f && paddedDepth <= minOverlap + 0.1f) {
            ContactPoint cp;
            cp.point = clipped4[i];
            cp.normal = refNormal;
            cp.penetration = paddedDepth;
            manifold.contacts.push_back(cp);
        }
    }

    manifold.normal = refNormal;
    manifold.hasCollision = !manifold.contacts.empty();
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