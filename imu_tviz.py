#!/usr/bin/env python

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from topic_handler import TopicHandler
import tviz
import angles

from sensor_msgs.msg import Imu as MSG_TYPE

def msg_to_dict(msg):
    rx,ry,rz = angles.quat_to_rpy(msg.orientation)
    d = {}
    d['acc_x'] = msg.linear_acceleration.x
    d['acc_y'] = msg.linear_acceleration.y
    d['acc_z'] = msg.linear_acceleration.z
    d['vel_roll'] = msg.angular_velocity.x
    d['vel_pitch'] = msg.angular_velocity.y
    d['vel_yaw'] = msg.angular_velocity.z
    d['roll'] = rx
    d['pitch'] = ry
    d['yaw'] = rz
    return d

def topic_handler_to_row(topic_handler):
    msgs = topic_handler.get()
    dicts = map(msg_to_dict, msgs) if len(msgs) > 0 else [EMPTY_DICT]
    df = pd.DataFrame.from_dict(dicts)
    row = {}
    row['acc_x'] = df['acc_x'].mean()
    row['acc_y'] = df['acc_y'].mean()
    row['acc_z'] = df['acc_z'].mean()
    row['vel_roll'] = df['vel_roll'].mean()
    row['vel_pitch'] = df['vel_pitch'].mean()
    row['vel_yaw'] = df['vel_yaw'].mean()
    row['roll'] = angles.deg_angle_mean(df['roll'])
    row['pitch'] = angles.deg_angle_mean(df['pitch'])
    row['yaw'] = angles.deg_angle_mean(df['yaw'])
    return row

NODE_NAME = 'imu_tviz'
NAMES = ['microstrain','vectornav']
TOPICS = ['/imu/data','/vectornav/IMU']
RATE = 4
MAX_MSGS = 10
FIELDS = ['acc_x','acc_y','acc_z','vel_roll','vel_pitch','vel_yaw','roll','pitch','yaw']

if len(NAMES) == 0:
    NAMES = TOPICS
assert len(NAMES) == len(TOPICS)

TOPIC_HANDLERS = [TopicHandler(t, MSG_TYPE, max_msgs=MAX_MSGS, name=n) for n,t in zip(NAMES,TOPICS)]
EMPTY_DICT = dict([(FIELD,np.nan) for FIELD in FIELDS])

if __name__ == '__main__':
    tviz.run(NODE_NAME, RATE, TOPIC_HANDLERS, topic_handler_to_row, FIELDS)
