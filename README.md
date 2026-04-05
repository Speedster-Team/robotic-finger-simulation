# drake-finger-sim
* RDS Speedster team
* Spring 2026

## Drake ROS Installation
1. Install drake on system [drake installation tutorial](https://drake.mit.edu/apt.html)

2. Source drake, add this to bashrc

        export PATH="/opt/drake/bin${PATH:+:${PATH}}"
        export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

    verify success:
        
        python3 -c "import pydrake; print(pydrake.__file__)"

3. Create overlay workspace with drakeros

        mkdir -p ~/drake_ws/src
        cd ~/drake_ws/src
        git clone https://github.com/RobotLocomotion/drake-ros.git

4. Build the overlay workspace without building testing

        colcon build --packages-select drake_ros --cmake-args -DCMAKE_PREFIX_PATH=/opt/drake -DBUILD_TESTING=OFF

5. Source overlay workspace, add this to bashrc

        source ~/ws_drake_ros/install/setup.bash


## Usage Instructions
1. Clone this repository into the src of a new ros2 workspace
2. Colcon build
3. Launch the simulation in rviz: `ros2 launch finger_description finger.launch.xml`
