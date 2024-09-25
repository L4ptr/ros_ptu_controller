# ROS PTU Controller

[![PTU Controller Image](image/IMG-20240921-WA0002.jpg)](image/IMG-20240921-WA0002.jpg)

## Overview

This repository contains a ROS package for controlling a Pan-Tilt Unit (PTU) and different components using ROS nodes. It enable the use of kartesian koordiantes und was designed for the participation on the FRE 2024. It contains the CAD files used for mounting on a Directed Perception/Flir D47 PTU.

### Prerequisites

Before using this package, make sure you have the following installed:

- **ROS (Noetic)**
- **PTU hardware with serial connection**
- ROS <a href="https://wiki.ros.org/asr_flir_ptu_driver">ASR driver</a>
- **Python 3.x**

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
    rosrun ros_ptu_controller controller.py
    rosrun ros_ptu_controller trigger_listener.py
    rosrun ros_ptu_controller xyz_talker.py
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
