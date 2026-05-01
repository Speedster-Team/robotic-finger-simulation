"""Leafsystem class connecting ROS topic and drake simulation."""

import numpy as np

from pydrake.systems.framework import LeafSystem

import rclpy

from std_msgs.msg import Float64MultiArray


class Ros2Drake(LeafSystem):
    """Leaf system connecting ros inputs to drake."""

    def __init__(self, node):
        """Create instance of MotorSystem."""
        super().__init__()
        self.nu = 3

        # save node
        self._node = node

        # Internal state: holds the last received torque command
        # This is the ZOH behavior — persists until next message
        self.state_index = self.DeclareDiscreteState(self.nu)
        self.DeclarePerStepDiscreteUpdateEvent(
            self._update_torque)

        # Output port for flex motors (all but last)
        self.DeclareVectorOutputPort(
            'motor_torque', self.nu, self._calc_torque
        )

        # Set subscriber
        self._latest_torque = np.zeros(self.nu)
        self._sub = self._node.create_subscription(
            Float64MultiArray,
            '/cmd_torque',
            self._ros_callback,
            10,
        )

    def _calc_torque(self, context, output):
        """Filter input torques for only flexion motor torques."""
        state = context.get_discrete_state(self.state_index).get_value()
        output.SetFromVector(state)

    def _ros_callback(self, msg):
        """Save new torque topic messages."""
        data = list(msg.data)
        if len(data) == 3:
            self._latest_torque = np.array((data + [0.0] * self.nu)[:self.nu])

    def _update_torque(self, context, discrete_state):
        """Spin ROS node every time there is a simulation update for msgs."""
        rclpy.spin_once(self._node, timeout_sec=0)
        discrete_state.set_value(self._latest_torque)
