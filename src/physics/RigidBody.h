#pragma once
#include <glm/glm.hpp>

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

class RigidBody {
public:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 forces;
    float mass;

    RigidBody(float m, const glm::vec3& pos);
    void ApplyForce(const glm::vec3& force);
    void Integrate(float deltaTime);

    static constexpr float size = 1.0f;

    AABB GetAABB() const {
        glm::vec3 halfExtents(size / 2.0f);
        return { position - halfExtents, position + halfExtents };
    }
};

// Free function declaration (outside of class)
bool CheckAABBCollision(const AABB& a, const AABB& b);
