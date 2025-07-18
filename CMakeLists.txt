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
        /opt/homebrew/include    # For GLFW via Homebrew
)

# Link paths
link_directories(/opt/homebrew/lib)

# Add GLAD library
add_library(glad STATIC ${GLAD_DIR}/src/glad.c
        src/physics/RigidBody.cpp
        src/physics/RigidBody.h
        src/core/Scene.cpp
        src/core/Scene.h)

# Add graphics library (Renderer and Shader)
add_library(graphics STATIC
        src/graphics/Renderer.cpp
        src/graphics/Shader.cpp
)

# Add executable
add_executable(PhysicsEngine src/main.cpp)

# Link libraries to PhysicsEngine executable
target_link_libraries(PhysicsEngine
        glad
        graphics
        glfw
        OpenGL::GL
)

add_definitions(-DSHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
