# Autonome Roboter

Repository for exercises in MSI-AS Lecture 'Autonome Roboter' SoSe 21

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
├── AuRo.repos                      # file with required repositories
├── AuRo                            # Metapackage contains:
│   ├── CMakeLists.txt              # CMakeList.txt (don't touch)
│   ├── images                      # Images for every exercise
│   │   ├── ...
│   └── package.xml                 # Package.xml (don't touch)
├── exercise_1                      # ROS-Package for exercise 1
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── exercise_one.launch
│   └── package.xml
├── exercise_2                       # ROS-Package for exercise 2
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── exercise_2.launch
│   └── package.xml
└── README.md
```
