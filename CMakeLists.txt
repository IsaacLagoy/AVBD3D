cmake_minimum_required(VERSION 3.14)
project(Basilisk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------
# Dependencies
# -----------------------

# glad
add_library(glad STATIC include/glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# stb
add_library(stb INTERFACE include/stb/stb_image.h)
target_include_directories(stb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# glm, glfw, assimp, etc.
include(FetchContent)
add_subdirectory(include)

# -----------------------
# Build options
# -----------------------
option(WIREFRAME_RIGIDS "Turns rigid bodies from full polygons to wire frames." OFF)
option(SHOW_NORMALS "Displays normals as green pegs." ON)
option(SHOW_EPA_VERTICES "Displays the vertices used to construct the polytope near face as small red/blue boxes." OFF)
option(SHOW_CONTACT_POINTS "Displays the contact points between rigid bodies as large red/blue boxes." OFF)
option(SHOW_CONSTRAINTS "Displays the error in contacts as a pink line." ON)

# helper macro to cut down on repetition
macro(add_option_define target optname)
    if(${optname})
        target_compile_definitions(${target} PRIVATE ${optname})
    endif()
endmacro()

# -----------------------
# Executable
# -----------------------

# gather sources
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# define target
add_executable(render ${SRC_FILES})
target_include_directories(render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(render glad glfw glm assimp stb)

# attach compile options only to "render"
add_option_define(render WIREFRAME_RIGIDS)
add_option_define(render SHOW_NORMALS)
add_option_define(render SHOW_EPA_VERTICES)
add_option_define(render SHOW_CONTACT_POINTS)
add_option_define(render SHOW_CONSTRAINTS)

# -----------------------
# Resources
# -----------------------

# shaders
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/shaders)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# TODO: add same logic for textures
