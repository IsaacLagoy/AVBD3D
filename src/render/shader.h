#ifndef SHADER_H
#define SHADER_H

#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <glad/glad.h>

class Shader {
public:
    unsigned int ID;
    Shader(const char* vertexPath, const char* fragmentPath);
    void use();
    void setMat4(const std::string &name, const glm::mat4 &mat) const;
    void setVec4(const std::string &name, const glm::vec4 &vec) const;
    void setVec3(const std::string &name, const glm::vec3 &vec) const;
};

#endif