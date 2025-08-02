// SphereShape.h
#pragma once
#include "Shape.h"

class SphereShape : public Shape {
public:
    float radius;

    explicit SphereShape(float r) : radius(r) {}

    glm::vec3 GetSize() const {
        return glm::vec3(radius * 2.0f);
    }

    AABB ComputeAABB(const glm::vec3& position) const override {
        glm::vec3 rVec(radius);
        return AABB(position - rVec, position + rVec);
    }
};
