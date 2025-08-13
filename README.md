# Augmented Vertex Block Descent 3D

This is a 3D implementation of Augmented Vertex Block Descent designed by Roblox and the University of Utah. It is designed using their public 2D demo and extended into 3D. 

### Note

This project is still work in progress. I am going through the stages of leaning how AVBD works and debugging my code. I would love feedback if you have any tips on what may be wrong!

## Building
To clone the repo:

```bash
git clone https://github.com/IsaacLagoy/AVBD3D.git
```

Make sure that you have cmake and a C++ compiler installed.

To build:

```bash
mkdir build
cd build
cmake -DUSE_WAYLAND=OFF ..
cmake --build .
```

To run:
```
./render
```

## About This Version

To be compatible with the future C++ version of [Baslisk Engine](https://github.com/BasiliskGroup/BasiliskEngine), this project uses the following packages for rendering with OpenGL and linear algebra:

- GLFW
- GLAD
- GLM

There are a few major difference between this engine and the one seen in the avbd_demo2d provided by the University of Utah, primarily in the collision pipeline. 

GJK and EPA are used for collision detection and contact generation. Specifically, the nearest polytope face from EPA generates a single contact point per frame using barycentric coordinates. This will soon be adapted to persistent manifolds to support warmstarting contacts.

Despite the inaccuracy in definition, `orientation` has been changed to `rotation` to reflect the standard in many game engines. This was done to make the code more accessable.

`size` has been changed to `scale` to stay consistent with [Baslisk Engine](https://github.com/BasiliskGroup/BasiliskEngine) terminology.

There are many type and variable changes during the conversion from 2D to 3D, some of the major differences are listed below. Many of the redundant changes like those for the inertial and initial positions are excluded since they have the same types as the base position.

### Rigid

| 2D | 3D |
| -- | -- |
| `float3 position` | `vec3 position` and `quat rotation` |
| `float3 velocity` | `vec6 velocity` |
| `float2 size` | `vec3 scale` |
| `float moment` | `mat3x3 inertiaTensor` |

### Force

| 2D | 3D |
| -- | -- |
| `float3 jacobian` | `vec6 jacobian` |
| `float6 hessian` | `mat6x6 hessian` |


### Manifold

| 2D | 3D |
| -- | -- |
| `float2 rX` | `vec3 rX` |
| `float2 normal` | `vec3 normal` |
| `float2 C0` | `vec3 C0` |
| `NUM_CONTACTS = 2` | `NUM_CONTACTS = 4` |

## Relevant Links
- University of Utah Project Page - https://graphics.cs.utah.edu/research/projects/avbd/
- AVBD Paper - https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf
- 2D Demo - https://github.com/savant117/avbd-demo2d