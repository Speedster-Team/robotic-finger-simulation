from pydrake.systems.framework import LeafSystem
from pydrake.common.value import Value
from std_msgs.msg import Float64MultiArray
import rclpy
import numpy as np

class MotorSystem(LeafSystem):
    """Leaf system connecting torque commands to drake simulation."""
    def __init__(self, nu, topic="/cmd_torque"):
        super().__init__()
        self.nu = nu

        # Internal state: holds the last received torque command
        # This is the ZOH behavior — persists until next message
        state_index = self.DeclareDiscreteState(nu)  # initialized to zeros

        # Output port reads from discrete state
        self.DeclareStateOutputPort("torque", state_index)

        # Periodic update: poll the ROS subscription every sim step
        self.DeclarePerStepDiscreteUpdateEvent(self._update_torque)

        # Set up a plain rclpy subscription (no drake_ros needed)
        rclpy.init(args=None)
        self._node = rclpy.create_node("torque_command_listener")
        self._latest_torque = np.zeros(nu)
        self._sub = self._node.create_subscription(
            Float64MultiArray,
            topic,
            self._ros_callback,
            1,
        )

    def _ros_callback(self, msg):
        data = list(msg.data)
        # Clamp to nu
        self._latest_torque = np.array((data + [0.0] * self.nu)[:self.nu])

    def _update_torque(self, context, discrete_state):
        # Spin rclpy to process any pending messages
        rclpy.spin_once(self._node, timeout_sec=0)
        discrete_state.set_value(self._latest_torque)