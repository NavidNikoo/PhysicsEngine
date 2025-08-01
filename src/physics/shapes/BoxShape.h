// src/physics/shapes/BoxShape.h
#pragma once
#include <glm/glm.hpp>
#include "Shape.h"
#include "../collision/AABB.h"

class BoxShape : public Shape {
public:
    glm::vec3 halfExtents;

    explicit BoxShape(const glm::vec3& halfExtents)
        : halfExtents(halfExtents) {}

    glm::vec3 GetSize() const {
        return halfExtents * 2.0f;
    }

    AABB ComputeAABB(const glm::vec3& position) const override {
        return AABB(position - halfExtents, position + halfExtents);
    }
};
