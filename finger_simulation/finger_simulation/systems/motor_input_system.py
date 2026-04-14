"""Leafsystem class connecting ROS topic and drake simulation."""

import numpy as np

from pydrake.systems.framework import LeafSystem

import rclpy

from std_msgs.msg import Float64MultiArray


class MotorSystem(LeafSystem):
    """Leaf system connecting torque commands to drake simulation."""

    def __init__(self, nu, topic='/cmd_torque'):
        """Create instance of MotorSystem."""
        super().__init__()
        self.nu = nu

        # define splay gear ratio
        self.gear_ratio = 3.5

        # Internal state: holds the last received torque command
        # This is the ZOH behavior — persists until next message
        self.state_index = self.DeclareDiscreteState(nu)
        self.DeclarePerStepDiscreteUpdateEvent(
            self._update_torque)

        # Output port for flex motors (all but last)
        self.DeclareVectorOutputPort(
            'motor_torque', nu, self._calc_flex_torque
        )
        # Output port for splay motor (last element)
        self.DeclareVectorOutputPort(
            'motor_splay_torque', 1, self._calc_splay_torque
        )

        # Set up a plain rclpy subscription (no drake_ros needed)
        rclpy.init(args=None)
        self._node = rclpy.create_node('torque_command_listener')
        self._latest_torque = np.zeros(nu)
        self._sub = self._node.create_subscription(
            Float64MultiArray,
            topic,
            self._ros_callback,
            1,
        )

    def _calc_flex_torque(self, context, output):
        """Filter input torques for only flexion motor torques."""
        state = context.get_discrete_state(self.state_index).get_value()
        output.SetFromVector(state)

    def _calc_splay_torque(self, context, output):
        """Filter input torques for only splay motor torques."""
        state = context.get_discrete_state(self.state_index).get_value()
        output.SetFromVector(state[-1:] * self.gear_ratio)

    def _ros_callback(self, msg):
        """Save new torque topic messages."""
        data = list(msg.data)
        self._latest_torque = np.array((data + [0.0] * self.nu)[:self.nu])

    def _update_torque(self, context, discrete_state):
        """Spin ROS node every time there is a simulation update for msgs."""
        rclpy.spin_once(self._node, timeout_sec=0)
        discrete_state.set_value(self._latest_torque)
