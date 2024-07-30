#!/usr/bin/env python

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from topic_handler import TopicHandler
import tviz
import angles

from geometry_msgs.msg import PoseStamped as MSG_TYPE

def msg_to_dict(msg):
    rx,ry,rz = angles.quat_to_rpy(msg.pose.orientation)
    d = {}
    d['x'] = msg.pose.position.x
    d['y'] = msg.pose.position.y
    d['z'] = msg.pose.position.z
    d['roll'] = rx
    d['pitch'] = ry
    d['yaw'] = rz
    return d

def topic_handler_to_row(topic_handler):
    msgs = topic_handler.get()
    dicts = map(msg_to_dict, msgs) if len(msgs) > 0 else [EMPTY_DICT]
    df = pd.DataFrame.from_dict(dicts)
    row = {}
    row['x'] = df['x'].mean()
    row['y'] = df['y'].mean()
    row['z'] = df['z'].mean()
    row['roll'] = angles.deg_angle_mean(df['roll'])
    row['pitch'] = angles.deg_angle_mean(df['pitch'])
    row['yaw'] = angles.deg_angle_mean(df['yaw'])
    return row

NODE_NAME = 'mocap_tviz'
NAMES = ['agent1','agent2']
TOPICS = ['fishberg_test/world','fishberg_test2/world']
RATE = 4
MAX_MSGS = 10
FIELDS = ['x','y','z','roll','pitch','yaw']

if len(NAMES) == 0:
    NAMES = TOPICS
assert len(NAMES) == len(TOPICS)

TOPIC_HANDLERS = [TopicHandler(t, MSG_TYPE, max_msgs=MAX_MSGS, name=n) for n,t in zip(NAMES,TOPICS)]
EMPTY_DICT = dict([(FIELD,np.nan) for FIELD in FIELDS])

if __name__ == '__main__':
    tviz.run(NODE_NAME, RATE, TOPIC_HANDLERS, topic_handler_to_row, FIELDS)
