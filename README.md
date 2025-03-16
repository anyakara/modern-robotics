# Robotics Math Library

## Overview
**Robotics Math Library** is a high-performance C++ library for fundamental mathematical computations in robotics. It provides efficient tools for:
- **Linear Algebra** (Vectors, Matrices, Transformations)
- **Kinematics** (Forward/Inverse Kinematics, Jacobians)
- **Rigid Body Dynamics** (Newton-Euler, Lagrangian Mechanics)
- **Motion Planning** (A*, RRT, PRM)
- **Optimization** (Nonlinear solvers, Kalman Filters)

## Features
- **Optimized Computation:** SIMD support (AVX, SSE)
- **Multi-threading:** Parallel computation support
- **Python API:** Pybind11-based Python wrapper
- **Modular Design:** Easy to extend and integrate

## Installation

### **Build from Source**
1. Clone the repository:
```git clone https://github.com/yourname/robotics-math-lib.git
cd robotics-math-lib```

2. Create a build directory and compile:
```mkdir build && cd build
cmake ..
make -j$(nproc)```

## Python Bindings
To install the Python bindings:
```pip install .```


## Usage

1. Forward Kinematics
```#include "kinematics/ForwardKinematics.hpp"
using namespace robotics;

FKSolver fk_solver;
Matrix4d end_effector_pose = fk_solver.compute(joint_angles);```

2. Motion Planning with RRT
```#include "planning/PathPlanning.hpp"

RRTPlanner planner;
Path path = planner.plan(start, goal, obstacles);```

3. Python Example
```import robotics_math
fk_solver = robotics_math.FK()
pose = fk_solver.compute([0.1, 0.5, -0.3])```

## License
MIT License