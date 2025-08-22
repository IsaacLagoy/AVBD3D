# include glfw
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(GLFW_BUILD_WAYLAND ON CACHE BOOL "")
    set(GLFW_BUILD_X11    OFF CACHE BOOL "")
endif()

include(FetchContent)
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(glfw)

# include glm
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/icaven/glm.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(glm)

# include assimp
FetchContent_Declare(
  assimp
  GIT_REPOSITORY https://github.com/assimp/assimp.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(assimp)