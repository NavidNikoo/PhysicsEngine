#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "core/Scene.h"
#include "core/Camera.h"

// Window size
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// Mouse input
bool firstMouse = true;
double lastX = SCR_WIDTH / 2.0f;
double lastY = SCR_HEIGHT / 2.0f;

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    Camera* cam = static_cast<Camera*>(glfwGetWindowUserPointer(window));

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed y

    lastX = xpos;
    lastY = ypos;

    cam->ProcessMouseMovement(xoffset, yoffset);
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

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Physics Engine", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    Shader shader(SHADER_DIR "/basic.vert", SHADER_DIR "/basic.frag");
    Renderer renderer;
    Scene scene;
    Camera camera;

    // Mouse input setup
    glfwSetWindowUserPointer(window, &camera);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Projection (orthographic or perspective can be swapped easily here)
    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f),
        static_cast<float>(SCR_WIDTH) / SCR_HEIGHT,
        0.1f,
        100.0f
    );

    const float fixedDeltaTime = 0.016f; // 60 FPS physics
    float accumulator = 0.0f;
    float currentTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        float newTime = glfwGetTime();
        float frameTime = newTime - currentTime;
        currentTime = newTime;

        accumulator += frameTime;

        // Handle input once per frame (not per physics step)
        scene.HandleInput(window);
        camera.HandleInput(window, frameTime);

        // Physics updates at fixed time step
        while (accumulator >= fixedDeltaTime) {
            scene.StepPhysicsWithSubdivision(fixedDeltaTime);
            accumulator -= fixedDeltaTime;
        }

        // Render
        glClearColor(0.6f, 0.8f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader.use();

        glm::mat4 view = camera.GetViewMatrix();
        shader.setMat4("view", &view[0][0]);
        shader.setMat4("projection", &projection[0][0]);
        shader.setVec3("lightPos", glm::vec3(5.0f, 10.0f, 5.0f));
        shader.setVec3("viewPos", camera.position);

        scene.Render(renderer, shader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
