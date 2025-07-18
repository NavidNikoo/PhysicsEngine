#include "Renderer.h"

float cubeVertices[] = {
 // positions
 -0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f,  0.5f, -0.5f,
  0.5f,  0.5f, -0.5f,
 -0.5f,  0.5f, -0.5f,
 -0.5f, -0.5f, -0.5f,

 -0.5f, -0.5f,  0.5f,
  0.5f, -0.5f,  0.5f,
  0.5f,  0.5f,  0.5f,
  0.5f,  0.5f,  0.5f,
 -0.5f,  0.5f,  0.5f,
 -0.5f, -0.5f,  0.5f,

 -0.5f,  0.5f,  0.5f,
 -0.5f,  0.5f, -0.5f,
 -0.5f, -0.5f, -0.5f,
 -0.5f, -0.5f, -0.5f,
 -0.5f, -0.5f,  0.5f,
 -0.5f,  0.5f,  0.5f,

  0.5f,  0.5f,  0.5f,
  0.5f,  0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f,  0.5f,
  0.5f,  0.5f,  0.5f,

 -0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f,  0.5f,
  0.5f, -0.5f,  0.5f,
 -0.5f, -0.5f,  0.5f,
 -0.5f, -0.5f, -0.5f,

 -0.5f,  0.5f, -0.5f,
  0.5f,  0.5f, -0.5f,
  0.5f,  0.5f,  0.5f,
  0.5f,  0.5f,  0.5f,
 -0.5f,  0.5f,  0.5f,
 -0.5f,  0.5f, -0.5f
};

Renderer::Renderer() {
 glGenVertexArrays(1, &VAO);
 glGenBuffers(1, &VBO);
 glBindVertexArray(VAO);
 glBindBuffer(GL_ARRAY_BUFFER, VBO);
 glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);
 glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
 glEnableVertexAttribArray(0);
}

void Renderer::DrawCube(const glm::mat4& transform, Shader& shader) {
 shader.setMat4("model", &transform[0][0]);
 glBindVertexArray(VAO);
 glDrawArrays(GL_TRIANGLES, 0, 36);

}
