################################## citation [0] begin ################################
import numpy as np

from pydrake.geometry import DrakeVisualizer
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.controllers import InverseDynamicsController

import drake_ros.core
from drake_ros.core import ClockSystem, RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams

def main():
    sim_time = float("inf")

    builder = DiagramBuilder()
    drake_ros.core.init()

    sys_ros_interface = builder.AddSystem(RosInterfaceSystem("fingersim"))
    ClockSystem.AddToBuilder(builder, sys_ros_interface.get_ros_interface())

    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001),
        builder,
    )

    viz_dt = 1 / 32.0
    scene_tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            sys_ros_interface.get_ros_interface(),
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_dt,
            ),
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(),
        scene_tf_broadcaster.get_graph_query_input_port(),
    )

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

    # Load finger model
    model_file_url = "package://finger_description/urdf/finger.urdf"
    urdf_parser = Parser(plant)
    urdf_parser.package_map().PopulateFromRosPackagePath()
    urdf_parser.package_map().Add(
        "finger_description",
        "/home/michael-jenz/rds_ws/finger_vizualization/install/finger_description/share/finger_description"
    )
    (finger,) = urdf_parser.AddModels(url=model_file_url)
    plant.RenameModelInstance(model_instance=finger, name="speedster_finger")

    base_frame = plant.GetFrameByName("base_link", finger)
    plant.WeldFrames(plant.world_frame(), base_frame, RigidTransform())

    plant.Finalize()

    # Set the control input to zero torque
    nu = plant.num_actuated_dofs(finger)
    u0 = np.ones(nu)
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

    step = 0.1
    while simulator_context.get_time() < sim_time:
        next_time = min(
            simulator_context.get_time() + step,
            sim_time,
        )
        simulator.AdvanceTo(next_time)

if __name__ == "__main__":
    main()


################################## citation [0] end ################################