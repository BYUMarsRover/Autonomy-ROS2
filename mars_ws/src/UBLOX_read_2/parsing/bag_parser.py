# from IPython.core.debugger import set_trace

import numpy as np
import sqlite3

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class BagParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of, name_of, type_of in topics_data}
    #

    def __del__(self):
        self.conn.close()
    #

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}").fetchall()
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]
    #

    def get_msg_data(self, msg_list):

        msg_time = []
        header_time = []

        topic_data = {}

        msg_keys = list(msg_list[0][1].get_fields_and_field_types().keys())
        msg_keys.remove('header')

        for mk in msg_keys:
            topic_data[mk] = []
        #

        for msg in msg_list:
            msg_time.append(msg[0])

            h_sec = msg[1].header.stamp.sec
            h_nano = msg[1].header.stamp.nanosec
            header_time.append([h_sec, h_nano])

            for mk in msg_keys:
                tmp = getattr(msg[1], mk)
                topic_data[mk].append(tmp)
            #
        #

        header_time = np.array(header_time).T

        for mk in msg_keys:
            topic_data[mk] = np.array(topic_data[mk]).T
        #
        topic_data['header_time'] = header_time

        return topic_data
    #
#
