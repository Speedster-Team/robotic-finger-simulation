# finger_bridge package
* RDS Speedster team
* Spring 2026

## Description
This package serves as middleman shuffling commands between the simulation/harware and ros as well as feedback. 

## Launch files

## Design

### Topics
all versions:
- subscribes to '/commands' of custom type for finger commands

simulation_bridge:
- publishes to '/motor_torques' at 100Hz like the 
 ** there needs to be some sort of position to torque control going on here
hardware_bridge:
- sends complete trajectories over to teensy
- recieves feedback