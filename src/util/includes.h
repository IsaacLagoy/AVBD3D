#ifndef INCLUDES_H
#define INCLUDES_H

#include <iostream>
#include <set>
#include <memory>
#include <unordered_map>
#include <vector>
#include <array>
#include <optional>

// import glad and glfw
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// import glm
#include <glm/glm.hpp>

// look for future changes to glm experimental
#define GLM_ENABLE_EXPERIMENTAL 
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

// import assimp Jonah Stuff
#include <assimp/scene.h>
#include <stb/stb_image.h>

#define DEBUG_PRINT false
#define DEBUG_LINEAR_PRINT false

// shorthand names
using vec2 = glm::vec2;
using vec3 = glm::vec3;
using vec4 = glm::vec4;
using mat3x3 = glm::mat3x3;
using mat4x4 = glm::mat4x4;
using quat = glm::quat;

#endif