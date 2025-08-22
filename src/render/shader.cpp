#include "shader.h"

Shader::Shader(const char* vertexPath, const char* fragmentPath) {
    std::ifstream vFile(vertexPath), fFile(fragmentPath);
    std::stringstream vStream, fStream;
    vStream << vFile.rdbuf(); fStream << fFile.rdbuf();
    std::string vCode = vStream.str(), fCode = fStream.str();
    const char* vShaderCode = vCode.c_str();
    const char* fShaderCode = fCode.c_str();

    unsigned int vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, NULL);
    glCompileShader(vertex);

    int success;
    char infoLog[512];
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertex, 512, NULL, infoLog);
        std::cerr << "Vertex Shader compilation failed:\n" << infoLog << std::endl;
    }

    unsigned int fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, NULL);
    glCompileShader(fragment);
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragment, 512, NULL, infoLog);
        std::cerr << "Fragment Shader compilation failed:\n" << infoLog << std::endl;
    }

    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);

    glGetProgramiv(ID, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(ID, 512, NULL, infoLog);
        std::cerr << "Shader Program linking failed:\n" << infoLog << std::endl;
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);
}


void Shader::use() {
    glUseProgram(ID);
}

void Shader::setMat4(const std::string &name, const glm::mat4 &mat) const {
    GLint uniformLocation = glGetUniformLocation(ID, name.c_str());
    if (uniformLocation == -1) {
        std::cerr << "Warning: Uniform '" << name << "' not found in shader program." << std::endl;
    }
    glUniformMatrix4fv(uniformLocation, 1, GL_FALSE, glm::value_ptr(mat));
}

void Shader::setVec4(const std::string &name, const glm::vec4 &vec) const {
    GLint uniformLocation = glGetUniformLocation(ID, name.c_str());
    if (uniformLocation == -1) {
        std::cerr << "Warning: Uniform '" << name << "' not found in shader program." << std::endl;
    }
    glUniform4fv(uniformLocation, 1, glm::value_ptr(vec));
}

void Shader::setVec3(const std::string &name, const glm::vec3 &vec) const {
    GLint uniformLocation = glGetUniformLocation(ID, name.c_str());
    if (uniformLocation == -1) {
        std::cerr << "Warning: Uniform '" << name << "' not found in shader program." << std::endl;
    }
    glUniform3fv(uniformLocation, 1, glm::value_ptr(vec));
}