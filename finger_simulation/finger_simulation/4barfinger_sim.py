# inspired by https://github.com/RobotLocomotion/drake-ros/blob/main/drake_ros_examples/examples/multirobot/multirobot.py

import argparse
import numpy as np

from pydrake.geometry import DrakeVisualizer
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.geometry import Meshcat
from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams
import drake_ros.core
from drake_ros.core import ClockSystem, RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams

from ament_index_python.packages import get_package_share_directory
import os

def main():
    builder = DiagramBuilder()
    drake_ros.core.init()

    sys_ros_interface = builder.AddSystem(RosInterfaceSystem("fingersim"))
    ClockSystem.AddToBuilder(builder, sys_ros_interface.get_ros_interface())

    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001),
        builder,
    )

    viz_dt = 1 / 100.0
    scene_tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            sys_ros_interface.get_ros_interface(),
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_dt,
            ),
        )
    )

    ### FOR MESHCAT

    # meshcat = Meshcat()

    # visualizer = MeshcatVisualizer.AddToBuilder(
    #     builder,
    #     scene_graph,
    #     meshcat,
    #     MeshcatVisualizerParams()
    # )

    ### FOR RVIZ

    scene_visualizer = builder.AddSystem(
        RvizVisualizer(
            sys_ros_interface.get_ros_interface(),
            params=RvizVisualizerParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_dt,
            ),
        )
    )

    builder.Connect(
        scene_graph.get_query_output_port(),
        scene_visualizer.get_graph_query_input_port(),
    )

    builder.Connect(
        scene_graph.get_query_output_port(),
        scene_tf_broadcaster.get_graph_query_input_port(),
    )

    # Load finger SDF
    parser = Parser(plant)
    pm = parser.package_map()

    # Derive workspace install dir from any known package
    share_dir = get_package_share_directory("finger_simulation")
    install_dir = os.path.join(share_dir, "..", "..", "..")  # share/pkg -> install/
    pm.PopulateFromFolder(os.path.realpath(install_dir))

    # Weld the grass to the world so that it's fixed during the simulation.
    finger, = parser.AddModels(url="package://finger_description/sdf/finger.sdf")
    plant.RenameModelInstance(model_instance=finger, name="speedster_finger")
    parser.AddModels(
        url="package://finger_simulation/models/grasspatch/model.sdf")
    parser.AddModels(
        url="package://finger_simulation/models/Standard_Toilet/model.sdf")

    base_frame = plant.GetFrameByName("base_link", finger)
    plant.WeldFrames(plant.world_frame(), base_frame, RigidTransform(np.array([0,0,.05])))

    # close loop for four bar
    middle_phalanx2 = plant.GetBodyByName("middle_phalanx2", finger)
    distal_phalanx  = plant.GetBodyByName("distal_phalanx", finger)

    p_AP = np.array([0.0001,  0.0348, -0.0147])  # dip_flex2 in middle_phalanx2 frame
    p_BQ = np.array([-0.0054, 0.0022, -0.0087])  # dip_flex2 in distal_phalanx frame

    plant.AddBallConstraint(
        body_A=middle_phalanx2,
        p_AP=p_AP,
        body_B=distal_phalanx,
        p_BQ=p_BQ,
    )
    plant.Finalize()

    ### Set up scene

    grasspatch_frame = plant.GetFrameByName("grasspatch_frame")
    plant_context = plant.CreateDefaultContext()
    tf_world_grasspatch = grasspatch_frame.CalcPoseInWorld(plant_context)

    standard_toilet_body = plant.GetBodyByName("toilet_base_link")
    tf_grasspatch_toilet = RigidTransform(RollPitchYaw(np.asarray([45, 30, 0]) * np.pi / 180), p=[1.0,0,0.8])
    tf_world_toilet = tf_world_grasspatch.multiply(tf_grasspatch_toilet)
    plant.SetDefaultFloatingBaseBodyPose(standard_toilet_body, tf_world_toilet)


    # Zero torque input
    nu = plant.num_actuated_dofs(finger)
    u0 = np.ones(nu) * 10
    constant = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        constant.get_output_port(0),
        plant.get_actuation_input_port(finger),
    )

    _viz = DrakeVisualizer.AddToBuilder(builder, scene_graph)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    plant_context = diagram.GetMutableSubsystemContext(plant, simulator_context)

    joint = plant.GetJointByName("mcp_flexion")
    print("mcp_flexion axis:", joint.revolute_axis())  # should be [-1, 0, 0] in world at rest
    print("num_positions:", plant.num_positions())
    print("num_velocities:", plant.num_velocities())
    print("num_actuated_dofs:", plant.num_actuated_dofs(finger))
    print("gravity:", plant.gravity_field().gravity_vector())

    step = 0.01
    sim_time = float("inf")
    while simulator_context.get_time() < sim_time:
        next_time = min(
            simulator_context.get_time() + step,
            sim_time,
        )
        simulator.AdvanceTo(next_time)

if __name__ == "__main__":
    main()