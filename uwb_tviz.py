#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
import os
from tabulate import tabulate

from nlink_parser.msg import LinktrackNodeframe2 as MSG_TYPE

def node_id_to_name(node_id):
    return f'n{node_id}'

class UWBTopicHandler:
    def __init__(self, node_id):
        self.node_id = node_id
        self.bins = {}

        self.name = node_id_to_name(node_id)
        self.topic = f'/n{node_id}/range'
        self.sub = rospy.Subscriber(self.topic, MSG_TYPE, self.callback)

    def callback(self,msg):
        for node in msg.nodes:
            if node.id not in self.bins:
                self.bins[node.id] = []
            self.bins[node.id].append(float(node.dis))

    def get(self):
        bins = self.bins
        self.bins = {}
        return bins

    def received_ids(self):
        return list(self.bins.keys()) + [self.node_id]

def all_received_ids(topic_handlers):
    received_ids = set([])
    for topic_handler in topic_handlers:
        received_ids.update(topic_handler.received_ids())
    return sorted(list(received_ids))

def topic_handlers_to_dataframe(topic_handlers):
    COL_IDS = all_received_ids(topic_handlers)
    COL_NAMES = [f'n{col_id}' for col_id in COL_IDS]

    matrix = {}
    for topic_handler in topic_handlers:
        row_name = topic_handler.name
        cols = []
        bins = topic_handler.get()
        for col_id in COL_IDS:
            if col_id == topic_handler.node_id:
                cols.append('n/a')
            elif col_id not in bins:
                cols.append('no data')
            else:
                data = np.array(bins[col_id])
                cols.append(f'{data.mean():0.2f} ± {2*data.std():0.2f} [{len(data)}]')
        matrix[row_name] = cols
    
    df = pd.DataFrame.from_dict(matrix,orient='index',columns=COL_NAMES)
    return df

ROS_NODE_NAME = 'uwb_tviz'
A_IDS = [0,1,2,3,4,5]
B_IDS = [6,7,8,9,10,11]
SHOW_ONLY_SPECIFIED = False
SHOW_SYMMETRIC = True
RATE = 1

# SHOW_ONLY_SPECIFIED
# :: True: cols will only be shown for IDs specified in A_IDS + B_IDS
# :: False: cols will be shown for any ID heard from

# SHOW_SYMMETRIC
# :: True: shows A <=> B range measurements
# :: False: shows only A => B range measurments

NODE_IDS = A_IDS + B_IDS
A_NAMES = list(map(node_id_to_name,A_IDS))
B_NAMES = list(map(node_id_to_name,B_IDS))
NODE_NAMES = A_NAMES + B_NAMES

LISTENER_IDS = NODE_IDS if SHOW_SYMMETRIC else A_IDS
SHOW_COLS = NODE_NAMES if SHOW_SYMMETRIC else B_NAMES

TOPIC_HANDLERS = [UWBTopicHandler(node_id) for node_id in LISTENER_IDS]

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME)
    rate = rospy.Rate(RATE)
    rate.sleep()

    while not rospy.is_shutdown():
        matrix = topic_handlers_to_dataframe(TOPIC_HANDLERS)

        if SHOW_ONLY_SPECIFIED:
            matrix.drop(columns=matrix.columns.difference(SHOW_COLS), inplace=True)

        output = tabulate(matrix,headers='keys',tablefmt='psql')

        os.system('clear')
        print(output,flush=True)

        rate.sleep()
