cmake_minimum_required(VERSION 3.14)
project(Basilisk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add glad
add_library(glad STATIC include/glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add stb
add_library(stb INTERFACE include/stb/stb_image.h)
target_include_directories(stb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# get other includes glm, glfw
include(FetchContent)
add_subdirectory(include)

# Search recursively for all .cpp files in src/
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Then use that list to build the executable
add_executable(render ${SRC_FILES})

target_include_directories(render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(render glad glfw glm assimp stb)

# ADD THESE LINES TO COPY SHADERS
# Ensure the 'shaders' directory exists in the build output
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/shaders)
# Copy the contents of your source shaders directory to the build shaders directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})