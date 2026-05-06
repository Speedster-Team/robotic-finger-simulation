from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py

from finger_interfaces.msg import MotorFeedback


class FingerRecorder(Node):
    """Class to record finger data."""

    def __init__(self):
        """Init instance of finger recorder."""
        super().__init__('finger_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri = f'src/robotic-finger-simulation/finger_recorder/bags/finger_bag_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
            storage_id='mcap')

        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            id=0,
            name='motor_pos_action_feedback',
            type='finger_interfaces/msg/MotorFeedback',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            MotorFeedback,
            'motor_pos_action_feedback',
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        """Subscription callback."""
        self.writer.write(
            'motor_pos_action_feedback',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    """Main function."""
    try:
        with rclpy.init(args=args):
            sbr = FingerRecorder()
            rclpy.spin(sbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()