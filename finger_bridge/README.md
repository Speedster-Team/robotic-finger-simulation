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

### Laptop - Teensy communication standard

#### Data command from laptop to teensy
This command is used to send a position trajectory for the teensy to send to motors.

    <command_type = D> <data_length: int, 1-1000> <repeat = 1 | 0>

    <mcp_splay_position: float> <mcp_flex_position: float> <pip_flex_position: float>

    ...

    <mcp_splay_position: float> <mcp_flex_position: float> <pip_flex_position: float>
            
    <data_crc8: uint_8> 
    end

#### Stop command from laptop to teensy
This command will stop the teensy safely.

    <command_type = S>
    end

#### General feedback from teensy to laptop
The teensy is constantly sending these commands back to the laptop.

    <mcp_splay_position: float> <mcp_flex_position: float> <pip_flex_position: float>

#### Message recieved feedback from teensy to laptop
This message is response for a message recieved from the laptop.

    <success_bool >


