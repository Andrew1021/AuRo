# Autonome Roboter

Repository for exercises in MSI-AS Lecture 'Autonome Roboter' SoSe 21

[![GitHub Action Status](https://github.com/Andrew1021/AuRo/workflows/Setup%20ROS%20environment/badge.svg)](https://github.com/Andrew1021/AuRo)
<!--- [![Dependabot Status](https://api.dependabot.com/badges/status?host=github&repo=ros-tooling/action-ros-ci)](https://dependabot.com) --->
<!--- [![codecov](https://codecov.io/gh/ros-tooling/action-ros-ci/branch/master/graph/badge.svg)](https://codecov.io/gh/ros-tooling/action-ros-ci) --->

# Installation

Clone this repository into the `src` directory.

Make sure that this repository is your current directory.
This must be the given for the next steps.

## Automatic installation

To install all dependencies run `./install.sh`.

## Running Husky Simulation

Make sure that the file `setup.bash` is sourced either by:

```bash
source /opt/ros/noetic/setup.bash
```

or by calling the alias which was created with the code above:

```bash
auro
```

### Exercise 1

Build the catkin workspace:

```bash
cd to/your/catkin_ws
catkin build
# re-source your catkin workspace with:
auro
# or:
source ~/catkin_ws/devel/setup.bash
```

when the build is finished and the catkin_workspace is re-sourced the simulation can be started via:

```bash
roslaunch exercise_1 exercise_1.launch
```

## Repository Structure

```bash
.
├── github                          # Metapackage contains: ROS-Package for exercise 1
|   ├── workflows                   # Folder for launch files
|   └── setup-ros.yml               # launch file for exercise 1
├── exercise_1                      # Metapackage contains: ROS-Package for exercise 1
│   ├── images                      # Images for exercise 2
|   ├── launch                      # Folder for launch files
│   │   └── exercise_1.launch       # launch file for exercise 1
│   ├── CMakeLists.txt              # CMakeList.txt (don't touch)
│   └── package.xml                 # Package.xml (don't touch)
├── husky_highlevel_controller                  # Metapackage contains: ROS-Package for exercise 2
|   ├── config                                  # Folder for configuration files
|   ├── include                                 # Folder for include files
│   │   └── husky_highlevel_controller          # Folder for include files for exercise 2
│   │       └── husky_highlevel_controller.hpp  # header file for exercise 2
|   ├── launch                                  # Folder for launch files
│   │   └── husky_highlevel_controller.launch   # launch file for exercise 2
|   ├── src                                     # Folder for source files
│   │   └── husky_highlevel_controller.cpp      # source file for exercise 2
|   ├── test                                    # Folder for tests
│   │   └── unit                                # Folder for unit tests for exercise 2
│   ├── images                                  # Images for exercise 2
│   ├── CMakeLists.txt                          # CMakeList.txt (don't touch)
│   └── package.xml                             # Package.xml (don't touch)
├── gitignore                       # ignore list for commit files
├── AuRo.repos                      # file with required repositories
├── LICENSE                         # license for code in repository
├── README.md                       # file with repository decription
└── install.sh                      # install.sh for ros1 environment
```
