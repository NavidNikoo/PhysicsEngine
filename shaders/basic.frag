#version 330 core

out vec4 FragColor;

uniform vec3 objectColor;
uniform vec3 lightPos;
uniform vec3 viewPos;

void main()
{
    // Simple shading (ambient only for now)
    FragColor = vec4(objectColor, 1.0);
}
