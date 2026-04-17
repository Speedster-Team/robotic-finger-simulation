# drake-finger-sim
* RDS Speedster team
* Spring 2026

## Description
This project allows for the simulation of a robotic finger using Drake.

## Video
https://github.com/user-attachments/assets/51fc3995-e88c-482a-a0bd-d43471b91480

## Drake ROS Installation
Drake must be installed on system as well as on drake_ros built locally in a overlay workspace.

1. Install drake on system [drake installation tutorial](https://drake.mit.edu/apt.html)

2. Source drake, add this to bashrc

        export PATH="/opt/drake/bin${PATH:+:${PATH}}"
        export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

3. Create overlay workspace with drakeros

        mkdir -p ~/drake_ws/src
        cd ~/drake_ws/src
        git clone https://github.com/RobotLocomotion/drake-ros.git

4. Build the drake overlay workspace. Build using gcc in ~/drake_ws.

        source /opt/ros/kilted/setup.bash
        export CC=gcc-13
        export CXX=g++-13
        colcon build --packages-select drake_ros \
        --cmake-args \
            -DCMAKE_PREFIX_PATH=/opt/drake \
            -Dpybind11_DIR=/opt/drake/lib/cmake/pybind11 \
            -DCMAKE_C_COMPILER=gcc-13 \
            -DCMAKE_CXX_COMPILER=g++-13 \
            -DBUILD_TESTING=OFF

5. Source overlay workspace, add this to bashrc

        source ~/drake_ws/install/setup.bash


## Usage Instructions
1. Clone this repository into the src of a new ros2 workspace
2. Build using `colcon build`
3. Make sure you source the drake workspace, see the *finger_simulation README* for more information
3. Launch the drake simulation: `ros2 launch finger_simulation 4barsim.launch.xml`

## AI usage
AI was used to get the 'basic_' version up and runnning, provided with examples from the official drake_ros github repo.
