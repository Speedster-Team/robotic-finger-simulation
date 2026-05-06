import rosbag2_py
from rclpy.serialization import deserialize_message
from finger_interfaces.msg import MotorFeedback
import matplotlib.pyplot as plt
import numpy as np
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        # place relative path here from src/
        uri='src/robotic-finger-simulation/finger_recorder/finger_bag_20260506_010247',
        storage_id='mcap'),
    rosbag2_py.ConverterOptions('', ''))

print(reader.get_all_topics_and_types())  # sanity check

data = []
while reader.has_next():
    topic, raw, t = reader.read_next()
    if topic in ('motor_pos_action_feedback', '/motor_pos_action_feedback'):
        msg = deserialize_message(raw, MotorFeedback)
        data.append((t, msg))

print(f'{len(data)} messages read')


timestamps = [t / 1e9 for t, msg in data]  # nanoseconds -> seconds
values = [msg.motor_positions for t, msg in data]  # replace with your actual field name
active_values = [msg.active for t, msg in data]  # replace with your actual field name
values = np.array(values)
print(values)
plt.plot(timestamps, values[:, 0])
plt.plot(timestamps, values[:, 1])
plt.plot(timestamps, values[:, 2])
# plt.plot(timestamps, active_values)
# plt.plot(timestamps, values[:, 2])
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Motor Feedback')
plt.show()