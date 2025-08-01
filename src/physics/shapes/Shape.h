// src/physics/shapes/Shape.h
#pragma once
#include <glm/glm.hpp>
#include "../collision/AABB.h"

class Shape {
public:
    virtual ~Shape() = default;

    // 👇 This line is the fix
    virtual AABB ComputeAABB(const glm::vec3& position) const = 0;
};