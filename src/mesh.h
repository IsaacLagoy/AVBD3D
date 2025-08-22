#ifndef MESH_H
#define MESH_H

#include "util/includes.h"

struct Mesh {
    // Cube vertices (position only)
    inline static const float verts[72] = {
        // Positions          // Normals
        // Front face (+Z)
        -0.5f, -0.5f,  0.5f,  
        0.5f, -0.5f,  0.5f,  
        0.5f,  0.5f,  0.5f,  
        -0.5f,  0.5f,  0.5f,  

        // Back face (-Z)
        -0.5f, -0.5f, -0.5f,  
        -0.5f,  0.5f, -0.5f,  
        0.5f,  0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,  

        // Left face (-X)
        -0.5f, -0.5f, -0.5f, 
        -0.5f, -0.5f,  0.5f, 
        -0.5f,  0.5f,  0.5f, 
        -0.5f,  0.5f, -0.5f, 

        // Right face (+X)
        0.5f, -0.5f, -0.5f,  
        0.5f,  0.5f, -0.5f,
        0.5f,  0.5f,  0.5f,
        0.5f, -0.5f,  0.5f,

        // Bottom face (-Y)
        -0.5f, -0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,
        0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,

        // Top face (+Y)
        -0.5f,  0.5f, -0.5f, 
        -0.5f,  0.5f,  0.5f, 
        0.5f,  0.5f,  0.5f, 
        0.5f,  0.5f, -0.5f,
    };

    inline static const float norms[72] {
        0, 0, 1,
        0, 0, 1,
        0, 0, 1,
        0, 0, 1,

        0, 0, -1,
        0, 0, -1,
        0, 0, -1,
        0, 0, -1,

        -1, 0, 0,
        -1, 0, 0,
        -1, 0, 0,
        -1, 0, 0,

        1, 0, 0,
        1, 0, 0,
        1, 0, 0,
        1, 0, 0,

        0, -1, 0,
        0, -1, 0,
        0, -1, 0,
        0, -1, 0,

        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
    };

    // Cube indices for drawing with glDrawElements
    inline static const unsigned int inds[36] = {
        // Front face
        0, 1, 2,
        2, 3, 0,

        // Back face
        4, 5, 6,
        6, 7, 4,

        // Left face
        8, 9, 10,
        10, 11, 8,

        // Right face
        12, 13, 14,
        14, 15, 12,

        // Bottom face
        16, 17, 18,
        18, 19, 16,

        // Top face
        20, 21, 22,
        22, 23, 20
    };

    inline static const int numUniqueVerts = 8;
    inline static const vec3 uniqueVerts[8] = {
        vec3(-0.5, -0.5, -0.5),
        vec3(-0.5, -0.5, 0.5),
        vec3(-0.5, 0.5, -0.5),
        vec3(-0.5, 0.5, 0.5),
        vec3(0.5, -0.5, -0.5),
        vec3(0.5, -0.5, 0.5),
        vec3(0.5, 0.5, -0.5),
        vec3(0.5, 0.5, 0.5)
    };

    static int bestDot(vec3 dir);
};

#endif