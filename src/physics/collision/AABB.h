#pragma once
#include <glm/glm.hpp>

class AABB {
public:
    glm::vec3 min;
    glm::vec3 max;

    AABB() = default;
    AABB(const glm::vec3& min, const glm::vec3& max)
        : min(min), max(max) {}

    bool Overlaps(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
               (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }

    bool Contains(const glm::vec3& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    glm::vec3 GetOverlap(const AABB& other) const {
        glm::vec3 overlap;
        overlap.x = std::max(0.0f, std::min(max.x, other.max.x) - std::max(min.x, other.min.x));
        overlap.y = std::max(0.0f, std::min(max.y, other.max.y) - std::max(min.y, other.min.y));
        overlap.z = std::max(0.0f, std::min(max.z, other.max.z) - std::max(min.z, other.min.z));
        return overlap;
    }

};
