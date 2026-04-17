"""Leafsystem class connecting drake simulation outputs to ROS."""

import numpy as np

from pydrake.systems.framework import LeafSystem

import rclpy

from std_msgs.msg import Float64MultiArray


class Drake2Ros(LeafSystem):
    """Leaf system connecting drake outputs to ros."""

    def __init__(self, node):
        """Create instance of Drake2Ros."""
        super().__init__()
        self.nu = 3

        # save node
        self._node = node

        # Input port for motor velocity
        self.vel_input_port = self.DeclareVectorInputPort(
            'motor_velocity', self.nu
        )

        # Input port for motor position
        self.pos_input_port = self.DeclareVectorInputPort(
            'motor_position', self.nu
        )

        # init publisher
        self._vel_pub = self._node.create_publisher(
            Float64MultiArray,
            '/motor_velocity',
            10
        )

        # init publisher
        self._pos_pub = self._node.create_publisher(
            Float64MultiArray,
            '/motor_position',
            10
        )

        # create event to publish at specified freq
        self.DeclarePeriodicPublishEvent(
            period_sec=1.0/100.0,   # 100 Hz
            offset_sec=0.0,
            publish=self._publish_motor_velocity,
        )

        # create event to publish at specified freq
        self.DeclarePeriodicPublishEvent(
            period_sec=1.0/100.0,   # 100 Hz
            offset_sec=0.0,
            publish=self._publish_motor_position,
        )

    def _calc_flex_torque(self, context, output):
        """Filter input torques for only flexion motor torques."""
        state = context.get_discrete_state(self.state_index).get_value()
        output.SetFromVector(state)

    def _calc_splay_torque(self, context, output):
        """Filter input torques for only splay motor torques."""
        state = context.get_discrete_state(self.state_index).get_value()
        output.SetFromVector(state[:1] * self.gear_ratio)

    def _ros_callback(self, msg):
        """Save new torque topic messages."""
        data = list(msg.data)
        self._latest_torque = np.array((data + [0.0] * self.nu)[:self.nu])

    def _update_torque(self, context, discrete_state):
        """Spin ROS node every time there is a simulation update for msgs."""
        rclpy.spin_once(self._node, timeout_sec=0)
        discrete_state.set_value(self._latest_torque)

    def _publish_motor_velocity(self, context):
        """Publish the motor velocity on a ROS topic."""
        vel = self.vel_input_port.Eval(context)
        msg = Float64MultiArray()
        msg.data = vel.tolist()
        self._vel_pub.publish(msg)

    def _publish_motor_position(self, context):
        """Publish the motor position on a ROS topic."""
        pos = self.pos_input_port.Eval(context)
        msg = Float64MultiArray()
        msg.data = pos.tolist()
        self._pos_pub.publish(msg)
