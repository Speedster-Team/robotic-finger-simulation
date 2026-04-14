"""Runs drake simulation of robotic finger."""

import os
import time

from ament_index_python.packages import get_package_share_directory

import drake_ros.core
from drake_ros.core import ClockSystem, RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams

from finger_simulation.systems.finger_pulley_system import FingerPulleySystem
from finger_simulation.systems.motor_input_system import MotorSystem
from finger_simulation.systems.motor_radius_system import (
    MotorTorqueToForceSystem
)

import numpy as np

import pydot

from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.geometry import MeshcatVisualizerParams
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import ConstantVectorSource


class FingerSimulation():
    """Class runs drake simulation of the finger."""

    def __init__(self):
        """Create instance of FingerSimulation."""
        self.builder = DiagramBuilder()
        drake_ros.core.init()

        self.sys_ros_interface = self.builder.AddSystem(
            RosInterfaceSystem('fingersim'))
        ClockSystem.AddToBuilder(self.builder,
                                 self.sys_ros_interface.get_ros_interface())

        self.plant, self.scene_graph = AddMultibodyPlant(
            MultibodyPlantConfig(time_step=0.001),
            self.builder,
        )

    def init_viz(self, enable_rviz):
        """Initialize all visualization aspects of drake simulation."""
        # start tf broadcaster
        viz_dt = 1 / 100.0
        scene_tf_broadcaster = self.builder.AddSystem(
            SceneTfBroadcasterSystem(
                self.sys_ros_interface.get_ros_interface(),
                params=SceneTfBroadcasterParams(
                    publish_triggers={TriggerType.kPeriodic},
                    publish_period=viz_dt,
                ),
            )
        )

        self.builder.Connect(
            self.scene_graph.get_query_output_port(),
            scene_tf_broadcaster.get_graph_query_input_port(),
        )

        if (enable_rviz):
            # FOR RVIZ
            scene_visualizer = self.builder.AddSystem(
                RvizVisualizer(
                    self.sys_ros_interface.get_ros_interface(),
                    params=RvizVisualizerParams(
                        publish_triggers={TriggerType.kPeriodic},
                        publish_period=viz_dt,
                    ),
                )
            )

            self.builder.Connect(
                self.scene_graph.get_query_output_port(),
                scene_visualizer.get_graph_query_input_port(),
            )
        else:
            # FOR MESHCAT
            meshcat = Meshcat()

            MeshcatVisualizer.AddToBuilder(
                self.builder,
                self.scene_graph,
                meshcat,
                MeshcatVisualizerParams()
            )

    def load_scene(self):
        """Load all models into scene and fix in space."""
        # Load finger SDF
        parser = Parser(self.plant)
        pm = parser.package_map()

        # Derive workspace install dir from any known package
        share_dir = get_package_share_directory('finger_simulation')
        install_dir = os.path.join(share_dir, '..', '..', '..')  # share/pkg -> install/
        pm.PopulateFromFolder(os.path.realpath(install_dir))

        # Weld the grass to the world so that it's fixed during the simulation.
        self.finger, = parser.AddModels(
            url='package://finger_description/sdf/finger.sdf')
        self.plant.RenameModelInstance(
            model_instance=self.finger, name='speedster_finger')
        parser.AddModels(
            url='package://finger_simulation/models/grasspatch/model.sdf')
        parser.AddModels(
            url='package://finger_simulation/models/Standard_Toilet/model.sdf')
        self.box, = parser.AddModels(
            url='package://finger_simulation/models/box/model.sdf')

        base_frame = self.plant.GetFrameByName('base_link', self.finger)
        self.plant.WeldFrames(self.plant.world_frame(),
                              base_frame,
                              RigidTransform(RollPitchYaw([0, 0, 0]),
                                             np.array([0, 0, .1])))

        box_frame = self.plant.GetFrameByName('box_frame', self.box)
        self.plant.WeldFrames(self.plant.world_frame(),
                              box_frame,
                              RigidTransform(RollPitchYaw([0, 0, 0]),
                                             np.array([0, 0.2, .05])))

        # close loop for four bar
        middle_phalanx2 = self.plant.GetBodyByName(
            'middle_phalanx2', self.finger)
        distal_phalanx = self.plant.GetBodyByName(
            'distal_phalanx', self.finger)

        # dip_flex2 in middle_phalanx2 frame
        p_AP = np.array([0.0001,  0.0348, -0.0147])

        # dip_flex2 in distal_phalanx frame
        p_BQ = np.array([-0.0054, 0.0022, -0.0087])

        self.plant.AddBallConstraint(
            body_A=middle_phalanx2,
            p_AP=p_AP,
            body_B=distal_phalanx,
            p_BQ=p_BQ,
        )
        self.plant.Finalize()

        # set up other object locations
        grasspatch_frame = self.plant.GetFrameByName('grasspatch_frame')
        plant_context = self.plant.CreateDefaultContext()
        tf_world_grasspatch = grasspatch_frame.CalcPoseInWorld(plant_context)

        standard_toilet_body = self.plant.GetBodyByName('toilet_base_link')
        tf_grasspatch_toilet = RigidTransform(
            RollPitchYaw(np.asarray([45, 30, 0]) * np.pi / 180),
            p=[1.0, 0, 0.8])
        tf_world_toilet = tf_world_grasspatch.multiply(tf_grasspatch_toilet)
        self.plant.SetDefaultFloatingBaseBodyPose(
            standard_toilet_body, tf_world_toilet)

    def torques(self):
        """Initialize joint torque inputs."""
        # constant torques
        nu = self.plant.num_actuated_dofs(self.finger)
        u0 = np.ones(nu) * 10
        constant = self.builder.AddSystem(ConstantVectorSource(u0))
        self.builder.Connect(
            constant.get_output_port(0),
            self.plant.get_actuation_input_port(self.finger),
        )

        # # ros topic torque commands
        # nu = plant.num_actuated_dofs(finger)

        # # Subscriber reads Float64MultiArray from /finger/torque_command
        # torque_sub = builder.AddSystem(
        #     RosSubscriberSystem(
        #         SerializerInterface(Float64MultiArray),
        #         "/cmd_torque",
        #         QoSProfile(depth=1),
        #         sys_ros_interface.get_ros_interface(),
        #     )
        # )

        # ZeroOrderHold bridges the async ROS message into the Drake time-stepped system.
        # It holds the last received value until a new one arrives.
        # zoh = builder.AddSystem(ZeroOrderHold(period_sec=0.001, vector_size=nu))

        # builder.Connect(torque_sub.get_output_port(0), zoh.get_input_port(0))
        # builder.Connect(zoh.get_output_port(0), plant.get_actuation_input_port(finger))

    def build_diagram(self):
        """Build the diagram."""
        self.diagram = self.builder.Build()

    def save_diagram(self):
        """Save the diagram as a png."""
        dot_string = self.diagram.GetGraphvizString()
        graphs = pydot.graph_from_dot_data(dot_string)
        graphs[0].write_png('diagram.png')

    def run(self):
        """Start the simulation."""
        simulator = Simulator(self.diagram)
        simulator.Initialize()
        # time.sleep(5.0)  # delay 3 seconds
        simulator_context = simulator.get_mutable_context()
        simulator.set_target_realtime_rate(0)

        self.diagram.GetMutableSubsystemContext(self.plant, simulator_context)

        plant_context = self.diagram.GetMutableSubsystemContext(
            self.plant, simulator_context)
        
        step = 0.001
        sim_time = float('inf')
        while simulator_context.get_time() < sim_time:
            next_time = min(
                simulator_context.get_time() + step,
                sim_time,
            )
            simulator.AdvanceTo(next_time)

            # get contact forces on box
            contact_results = self.plant.get_contact_results_output_port()\
                .Eval(plant_context)
            for i in range(contact_results.num_point_pair_contacts()):
                info = contact_results.point_pair_contact_info(i)
                body_A = self.plant.get_body(info.bodyA_index()).name()
                body_B = self.plant.get_body(info.bodyB_index()).name()
                if 'box' in body_A or 'box' in body_B:
                    print(f't={simulator_context.get_time():.2f} '
                        f'{body_A}<->{body_B} '
                        f'force={info.contact_force()}')


def main():
    """Set up and start simulation."""
    # initialize finger simulation class
    fingersim = FingerSimulation()
    fingersim.init_viz(enable_rviz=True)
    fingersim.load_scene()

    # add external systems
    nu = fingersim.plant.num_actuated_dofs(fingersim.finger)

    torque_system = fingersim.builder.AddSystem(MotorSystem(nu, '/cmd_torque'))

    motor_torque_to_force_system = fingersim.builder.AddSystem(
        MotorTorqueToForceSystem())

    motor_tension_to_joint_torque_system = fingersim.builder.AddSystem(
        FingerPulleySystem())

    fingersim.builder.Connect(
        torque_system.GetOutputPort('motor_torque'),
        motor_torque_to_force_system.GetInputPort('motor_torque'),
    )
    fingersim.builder.Connect(
        motor_torque_to_force_system.GetOutputPort('tendon_tension'),
        motor_tension_to_joint_torque_system.GetInputPort('tendon_tension'),
    )
    fingersim.builder.Connect(
        torque_system.GetOutputPort('motor_splay_torque'),
        motor_tension_to_joint_torque_system.GetInputPort(
            'motor_splay_torque'),
    )
    fingersim.builder.Connect(
        motor_tension_to_joint_torque_system.GetOutputPort('joint_torque'),
        fingersim.plant.get_actuation_input_port(fingersim.finger),
    )

    fingersim.build_diagram()
    # fingersim.save_diagram()
    fingersim.run()


if __name__ == '__main__':
    main()
