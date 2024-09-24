# ROS PTU Controller

[![PTU Controller Image](image/ptu_setup.png)](image/ptu_setup.png)

## Overview

This repository contains a ROS (Robot Operating System) package for controlling a Pan-Tilt Unit (PTU) using ROS nodes. The goal of this project is to provide a flexible and easy-to-use interface for controlling PTU hardware through ROS, enabling the integration of pan-tilt control in various robotic applications.

---

## Features

- **PTU Motion Control**: Smooth and precise control of the PTU for both panning and tilting.
- **ROS Integration**: Built with ROS, making it easy to integrate into existing robotic systems.
- **Visualization**: Visualize the current state of the PTU through ROS topics.
- **Real-time Feedback**: Receive real-time feedback from the PTU regarding its current position and status.

---

## Installation

### Prerequisites

Before using this package, make sure you have the following installed:

- **ROS (Noetic, Melodic, etc.)**
- **PTU hardware and drivers**
- **Python 3.x** (if the controller is written in Python)

### Instructions

1. Clone the repository into your ROS workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/L4ptr/ros_ptu_controller.git
    ```

2. Build the package:

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. Install any additional dependencies:

    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

---

## Usage

1. Launch the PTU controller:

    ```bash
    roslaunch ptu_controller ptu_control.launch
    ```

2. To control the PTU, you can publish to the corresponding ROS topics for panning and tilting:

    - **/ptu/pan**: Controls the pan movement.
    - **/ptu/tilt**: Controls the tilt movement.

3. To visualize the PTU position in real-time, subscribe to the feedback topics:

    - **/ptu/position_feedback**: Provides the current pan and tilt angles.

---

## Example

Here's an example of how to command the PTU to pan and tilt:

```bash
rostopic pub /ptu/pan std_msgs/Float64 "data: 0.5"
rostopic pub /ptu/tilt std_msgs/Float64 "data: -0.2"
