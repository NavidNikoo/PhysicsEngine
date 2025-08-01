#pragma once
#include <glm/glm.hpp>
#include "AABB.h"

class Collider {
public:
    glm::vec3 size;

    Collider(const glm::vec3& size = glm::vec3(1.0f)) : size(size) {}

    AABB GetAABB(const glm::vec3& position) const {
        glm::vec3 halfSize = size * 0.5f;
        return AABB{position - halfSize, position + halfSize};
    }

    AABB ComputeAABB(const glm::vec3& position, const glm::vec3& size) const {
        glm::vec3 halfSize = size * 0.5f;
        AABB box;
        box.min = position - halfSize;
        box.max = position + halfSize;
        return box;
    }
};
