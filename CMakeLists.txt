cmake_minimum_required(VERSION 3.14)
project(Basilisk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add glad
add_library(glad STATIC include/glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# get other includes glm, glfw
include(FetchContent)
add_subdirectory(include)

add_executable(render
    src/helper.cpp
    src/engine.cpp
    src/main.cpp
    src/shader.cpp
    src/solver.cpp
    src/rigid.cpp
    src/camera.cpp
    src/includes.cpp
    src/manifold.cpp
    src/force.cpp
    src/collision.cpp
    src/mesh.cpp
    src/contact.cpp
)

target_include_directories(render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(render glad glfw glm)

# ADD THESE LINES TO COPY SHADERS
# Ensure the 'shaders' directory exists in the build output
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/shaders)
# Copy the contents of your source shaders directory to the build shaders directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})