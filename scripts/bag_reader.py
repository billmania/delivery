#!/usr/bin/env python3

import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG_PATH = '/mnt/logs/delivery_tests/'
MISSION = '2023_04_03_illahee/'
BAGS_DIR = 'ros_bags/'
BAG = 'rosbag2_2023_04_03-12_14_21'


def sequential_reader():
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=BAG_PATH + MISSION + BAGS_DIR + BAG,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=['/x150_status'])
    reader.set_filter(storage_filter)

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        timestamp = msg.header.stamp

        yaw = msg.compass_mag
        pitch = msg. pitch_deg
        roll = msg.roll_deg

        print(f"{round(roll, 3):0.3f},"
              f"{round(pitch, 3):0.3f},"
              f"{round(yaw, 3):0.3f}")

    reader.reset_filter()

sequential_reader()
