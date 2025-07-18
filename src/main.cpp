#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>

#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "physics/RigidBody.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(800, 600, "Physics Engine", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, 800, 600);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    Shader shader(SHADER_DIR "/basic.vert", SHADER_DIR "/basic.frag");
    Renderer renderer;

    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f),
        800.0f / 600.0f,
        0.1f,
        100.0f
    );

    // Create rigid body
    std::vector<RigidBody> bodies;
    for (int i = 0; i < 5; ++i) {
        bodies.emplace_back(1.0f, glm::vec3(0.0f, i * 1.1f, 0.0f));
    }

    float lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        float currentTime = glfwGetTime();
        float deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader.use();
        glm::mat4 view = glm::lookAt(
            glm::vec3(0.0f, 0.0f, 8.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f)
        );
        shader.setMat4("view", &view[0][0]);
        shader.setMat4("projection", &projection[0][0]);

        for (RigidBody& body : bodies) {
            body.ApplyForce(glm::vec3(0.0f, -9.81f, 0.0f) * body.mass);
            body.Integrate(deltaTime);

            float sideForce = ((rand() % 200) - 100) / 5000.0f; // Small random [-0.02, 0.02]
            body.ApplyForce(glm::vec3(sideForce, 0.0f, 0.0f));

            if (body.position.y < -1.0f) {
                body.position.y = -1.0f;
                body.velocity.y *= -0.5f;
            }

            if (body.position.y <= -1.0f) {
                body.velocity.x *= 0.8f;  // Simulate horizontal ground friction
            }

            glm::mat4 model = glm::translate(glm::mat4(1.0f), body.position);
            renderer.DrawCube(model, shader);
        }

        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                AABB a = bodies[i].GetAABB();
                AABB b = bodies[j].GetAABB();

                if (CheckAABBCollision(a, b)) {
                    glm::vec3 overlap;
                    overlap.x = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
                    overlap.y = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
                    overlap.z = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);

                    // Find the smallest axis of penetration
                    if (overlap.y < overlap.x) {
                        // Resolve in Y (vertical stacking)
                        if (bodies[i].position.y > bodies[j].position.y) {
                            bodies[i].position.y += overlap.y / 2.0f;
                            bodies[j].position.y -= overlap.y / 2.0f;
                        } else {
                            bodies[i].position.y -= overlap.y / 2.0f;
                            bodies[j].position.y += overlap.y / 2.0f;
                        }

                        // Bounce in Y
                        bodies[i].velocity.y *= -0.5f;
                        bodies[j].velocity.y *= -0.5f;
                    } else {
                        // Resolve in X (horizontal separation)
                        if (bodies[i].position.x > bodies[j].position.x) {
                            bodies[i].position.x += overlap.x / 2.0f;
                            bodies[j].position.x -= overlap.x / 2.0f;
                        } else {
                            bodies[i].position.x -= overlap.x / 2.0f;
                            bodies[j].position.x += overlap.x / 2.0f;
                        }

                        // Friction effect: dampen X velocity
                        bodies[i].velocity.x *= -0.3f;
                        bodies[j].velocity.x *= -0.3f;
                    }

                }
            }
        }


        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    // ⬇️ Properly terminate AFTER the loop
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;

}
