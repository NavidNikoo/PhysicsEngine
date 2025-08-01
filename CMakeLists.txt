cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)

set(DEPENDENCY_DIR "/Users/navidnikoo/Dependencies")
set(GLAD_DIR "${DEPENDENCY_DIR}/glad")
set(GLM_DIR "${DEPENDENCY_DIR}/glm-master")

# Include paths
include_directories(
        ${GLAD_DIR}/include
        ${GLM_DIR}
        /opt/homebrew/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

link_directories(/opt/homebrew/lib)

# Add core (physics and scene)
add_library(core STATIC
        ${GLAD_DIR}/src/glad.c
        src/core/Scene.cpp
        src/core/Camera.cpp
        src/physics/bodies/RigidBody.cpp
        src/physics/collision/ContactSolver.cpp
        src/physics/collision/Collision.h
        src/physics/collision/Collider.h
        src/physics/collision/AABB.h
        src/physics/shapes/BoxShape.h
        src/physics/shapes/Shape.h
        src/physics/collision/ContactManifold.h
        src/physics/collision/SATCollision.h
        src/physics/collision/SATCollision.cpp
)


# Add graphics
add_library(graphics STATIC
        src/graphics/Renderer.cpp
        src/graphics/Shader.cpp
)

# Add executable
add_executable(PhysicsEngine src/main.cpp)

# Link libraries
target_link_libraries(PhysicsEngine
        graphics
        core
        glfw
        OpenGL::GL
)

add_definitions(-DSHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
