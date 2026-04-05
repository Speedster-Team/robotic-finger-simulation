# drake-finger-sim
* RDS Speedster team
* Spring 2026

## Drake ROS Installation
1. Install drake on system [drake installation tutorial](https://drake.mit.edu/apt.html)

2. Source drake, add this to bashrc

        export PATH="/opt/drake/bin${PATH:+:${PATH}}"
        export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

3. Create overlay workspace with drakeros

        mkdir -p ~/drake_ws/src
        cd ~/drake_ws/src
        git clone https://github.com/RobotLocomotion/drake-ros.git

4. Build the drake overlay workspace. Build using gcc in a 

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

        source ~/ws_drake_ros/install/setup.bash


## Usage Instructions
1. Clone this repository into the src of a new ros2 workspace
2. Colcon build
3. Launch the drake simulation: `ros2 launch finger_simulation finger.launch.xml`
3. Launch the urdf model in rviz: `ros2 launch finger_description finger.launch.xml`
