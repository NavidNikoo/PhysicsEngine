#pragma once
#include <glm/glm.hpp>
#include "physics/collision/Collider.h"
#include "physics/shapes/BoxShape.h"
#include <glm/gtc/quaternion.hpp>
#include <memory>


class RigidBody {
public:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 forces;
    float mass;
    float staticFriction = 0.5f;
    float dynamicFriction = 0.3f;
    glm::vec3 color;
    glm::vec3 size = glm::vec3(1.0f);
    bool isStatic = false;   // New flag
    bool isSleeping = false;
    int sleepCounter = 0;
    const int sleepCounterThreshold = 30;  // You can tweak this
    bool hasAwakened = false; // Add this to your RigidBody class
    glm::vec3 angularVelocity = glm::vec3(0.0f);
    glm::vec3 torque = glm::vec3(0.0f);
    glm::mat3 inertiaTensor = glm::mat3(1.0f);  // Identity for now (will customize per shape)
    glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // For rotation
    glm::mat3 inverseInertiaTensor;



    Collider collider; // Each RigidBody has a collider now
    std::shared_ptr<BoxShape> shape;

    RigidBody(); // Default constructor
    RigidBody(float m, const glm::vec3& pos); // Constructor with mass and position (declaration only here!)

    RigidBody(float m, const glm::vec3& pos, const glm::vec3& sz);

    void ApplyForce(const glm::vec3& force);
    void ApplyPhysics(float dt);
    void IntegrateVelocity(float dt);
    void IntegratePosition(float dt);
    void ApplyTorque(const glm::vec3& t);
    void IntegrateAngularVelocity(float dt);
    void IntegrateOrientation(float dt);
    void ComputeInertia();
    void SetShapeAndSize(const glm::vec3& fullSize);

    AABB GetAABB() const;

};

// Free function declaration (outside of class)
bool CheckAABBCollision(const AABB& a, const AABB& b);
