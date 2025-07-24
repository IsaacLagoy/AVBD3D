#version 330 core
layout (location = 0) in vec3 aPos; // Position
layout (location = 1) in vec3 aNormal; // Normal vector

out vec3 FragPos; // To pass to fragment shader
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0)); // World space position
    Normal = mat3(transpose(inverse(model))) * aNormal; // Transform normal to world space
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
