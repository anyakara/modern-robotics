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
```bash
git clone https://github.com/yourname/robotics-math-lib.git
cd robotics-math-lib```

2. Create a build directory and compile:
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)```

## Python Bindings
To install the Python bindings:
```pip install .```


## Architecture

robotics-math-lib/
│── include/                      # Header files for core library
│   ├── linalg/                   # Linear algebra module
│   │   ├── Matrix.hpp             # Matrix operations
│   │   ├── Vector.hpp             # Vector operations
│   │   ├── Transform.hpp          # Transformation matrices
│   ├── kinematics/                # Kinematics module
│   │   ├── ForwardKinematics.hpp  # FK solvers
│   │   ├── InverseKinematics.hpp  # IK solvers
│   │   ├── Jacobian.hpp           # Jacobian computation
│   ├── dynamics/                  # Rigid body dynamics
│   │   ├── NewtonEuler.hpp        # Newton-Euler solver
│   │   ├── Lagrangian.hpp         # Lagrangian dynamics
│   ├── planning/                  # Motion planning
│   │   ├── PathPlanning.hpp       # A*, RRT, PRM
│   │   ├── Trajectory.hpp         # Splines & motion profiles
│   ├── optimization/              # Numerical solvers
│   │   ├── NonlinearSolver.hpp    # Levenberg-Marquardt, QP
│   │   ├── KalmanFilter.hpp       # EKF, UKF for sensor fusion
│   ├── utils/                     # Helper functions
│   │   ├── MathUtils.hpp          # General math utilities
│   │   ├── Timer.hpp              # Performance measurement
│── src/                           # Implementation files
│   ├── linalg/                    # Linear algebra implementation
│   ├── kinematics/                 # Kinematics implementation
│   ├── dynamics/                   # Dynamics implementation
│   ├── planning/                   # Motion planning implementation
│   ├── optimization/               # Optimization methods
│   ├── utils/                      # Helper function implementations
│── tests/                          # Unit tests
│   ├── test_matrix.cpp             # Tests for matrix operations
│   ├── test_kinematics.cpp         # Tests for FK, IK
│── examples/                       # Example applications
│   ├── basic_fk.cpp                # FK example
│   ├── rrt_planner.cpp             # RRT motion planning
│── python_bindings/                # Python wrapper
│   ├── robotics_math_pybind.cpp    # Pybind11 integration
│── benchmarks/                     # Performance benchmarks
│   ├── bench_matrix.cpp            # Benchmarking matrix ops
│── docs/                           # Documentation
│── CMakeLists.txt                   # CMake build script
│── README.md                        # Project overview
│── .gitignore                        # Ignore unnecessary files



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