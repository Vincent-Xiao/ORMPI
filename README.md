# [ORMPI](https://github.com/Vincent-Xiao/ORMPI) | [教程](https://iostream.io/2019/09/13/github-ormpi-%e5%bc%80%e6%ba%90%e6%9c%ba%e5%99%a8%e4%ba%ba%e8%bf%90%e5%8a%a8%e8%a7%84%e5%88%92%e5%99%a8/)

ORMPI is the **Open Robotic Motion Planning Implementation** for efficiently developing or evaluating  the motion planning algorithm for high degree-of-freedom robots. It supports the complete real-time motion planning procedure which implements robot modeling, planner, kinematics, and collision detection. 

ORMPI is designed  to be an **Open Robotic Motion Planning Compiler Stack** for  CPUs, GPUs, and specialized accelerators. It aims to close the gap between the productivity-focused motion planning frameworks, and the performance- and efficiency-focused hardware backends.

# Features

ORMPI implements the complete real-time motion planning algorithm for high degree-of-freedom robots.

- Representation of Environments: The **octree** is a spatial partitioning technique providing broad-phase processing by dividing a space into regions and testing if objects overlap the same region. We represent the environments via Octree in terms of its relative simplicity and decent accuracy.
- Robot Modeling: Oriented Bounding Boxes Tree (**OBBTree**)  is used to model 3D geometry and joints relationship (link chains) of a robot, which produces better bounds and better culling than AABBs or spheres.
- Kinematics: We refer to the source code of [Moveit](https://moveit.ros.org/) and implement the fast and accurate forward kinematics of robot arms.
- Planner: Rapidly-exploring Random Trees (**RRT**)  avoids the necessity to construct a roadmap a prior via incremental sampling methods which can efficiently boost motion planning for dynamic environments. And RRT is suitable and flexible for hardware backends meeting the demand of motion planning for online robots.
- Collision Detection: ORMPI proposed an **approximate OBB-Octree** node algorithm to boosts the performance of collision detection. It reduces 82.9% computation time than the original OBB-OBB Intersection.

Meanwhile, ORMPI supports **Half**-precision floating-point format to boosts motion planning, which is more appropriate for specialized accelerators.

# Installation

```
git clone https://github.com/Vincent-Xiao/ORMPI.git
mkdir build && cd build
cmake ..
make 
```
# Usage

The "**./src/ORMIP.cpp**" is a demo code for how to construct a motion planning alogrithm.

1. Define the motion planning environment: this demo uses [ROS](https://www.ros.org/) and [Moveit](https://moveit.ros.org/) to construct a pick-place task and exports the environment data to ORMPI
2. Define the center coordinate of environment and build an Octree.
3. Define a start config (*the rotating angles of the robot arm in configuration space*) and the goal config
4. Define the planner (RRT in demo) to start motion planning
5. Print the success count of iteration number and the average planning time

![The Pick-Place task](https://iostream.io/wp-content/uploads/2019/09/PICK-PLACE-e1568366602215.png)

The Pick-Place task

# Configuration

The parameters of motion planning can be configured in the file "**./config/config.h**".

- **DataType** Config for float or half
- **loopNum** for iteration of motion planning
- **TESTMode** for whether printing debug information
- **OctoTree** for the max depth and environment bounds to construct the OctoTree
- **RRT** for the bias, step size and max iteration of the search algorithm
- **Robot** for the degree-of-freedom (DOF), transform matrix and center coordinate of one robot.

# ToDo Work

- Supporting more real-time planner
- Supporting GPU
- Supporting compiler for FPGA
- Improving code packaging and riching the API

# License
-------
© Contributors Licensed under an [Apache-2.0](https://github.com/Vincent-Xiao/ORMPI/blob/master/LICENSE) license.
