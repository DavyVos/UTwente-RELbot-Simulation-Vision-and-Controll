# Welcome to the RELbot vision and controll project

## Install

1. Move into your ros2 workspace src folder.
```bash
cd ros2_ws/src
```
2. Pull the project

```bash
git pull git@github.com:DavyVos/UTwente-RELbot-Simulation-Vision-and-Controll.git
```
3. Build the package, use cmake-args to allow cmake-tools to find workspace packages.
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-ignore turtlesim
```

## Use the launch files
Use the launch to run the tests for assignments 1.2.

Start the videoserver.py

Start the image node.
```bash
cd ~/ros2_ws/launch/image_lineup.yaml
```

Start the object recognition and the simulator.
```bash
cd ~/ros2_ws/launch/relbot_simulation.yaml
```

## DetectBrightness node
**Description**

The DetectBrightness node subscribes to a camera image topic and determines whether the scene is dark or light based on the average brightness of the image.

**Running the node**
```bash
cd ~/ros2_ws/
. ./install/setup.bash
ros2 run assignment1 brightness
```
Run with threshold parameter
```bash
ros2 run assignment1 brightness --ros-args -p threshold:=75.0
```

**The node:**
* Subscribes to the image topic (sensor_msgs/msg/Image).
* Converts the incoming RGB image to grayscale using OpenCV.
* Computes the average pixel intensity over the entire image.
* *ompares the average brightness to a configurable threshold.
* Publishes the result on the `/brightness` topic using the 

```bash
ros2 topic info /brightness
```

```bash
ros2 interfaces show assignment1/msg/ObjectBrightness
```

## FindBrightObject Node
**Description**
Find an object, the node support two detection methods:
1. basic → Grayscale thresholding
1. hsvcircle → HSV color filtering + circular contour detection

**Running the node**

```bash
ros2 run assignment1 findobject
```

**Running the node**

This is the list of different parameter options:
```bash
ros2 run assignment1 findobject --ros-args -p method:="basic"
ros2 run assignment1 findobject --ros-args -p method:="basic" -p threshold:=50.0
ros2 run assignment1 findobject --ros-args -p method:="hsvcircle"
ros2 run assignment1 findobject --ros-args -p method:="hsvcircle" -p hsv_lower:="[90,150,140]" -p hsv_upper:="[102,255,240]"
```

## Sequencecontroller
**Description**

Controll the RELbot with support for preset routes, open and closed loop controll.

The SequenceController node controls the RELbot motors.
It publishes motor commands to:

`/input/motor_cmd`

The controller supports multiple control strategies:

1. Setpoint sequence (open loop)
1. Object following (open loop)
1. Closed-loop object/light following
1. Dynamic control loop (currently active)

⚠ Some strategies are still WIP (Work In Progress) and not fully integrated via parameters yet.

**Running the node**
```bash
ros2 run assignment1 sequencecontroller
```
Set mode (⚠ currently not switching behavior):
```bash
ros2 run assignment1 sequencecontroller --ros-args -p mode:="follow"
```