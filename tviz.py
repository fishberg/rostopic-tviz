#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
import os
from tabulate import tabulate

class DataFrameHandler:
    def __init__(self, topic_handlers, topic_handler_to_row, col_names):
        self.topic_handlers = topic_handlers
        self.topic_handler_to_row = topic_handler_to_row
        self.col_names = col_names

    def to_dataframe(self):
        matrix = dict([(th.name, self.topic_handler_to_row(th)) for th in self.topic_handlers])
        df = pd.DataFrame.from_dict(matrix, orient='index', columns=self.col_names)
        return df

def run(node_name, hz, topic_handlers, topic_handler_to_row, col_names):
    rospy.init_node(node_name)
    rate = rospy.Rate(hz)
    rate.sleep()

    dfh = DataFrameHandler(topic_handlers, topic_handler_to_row, col_names)

    while not rospy.is_shutdown():
        df = dfh.to_dataframe()
        output = tabulate(df, headers='keys', tablefmt='psql', floatfmt='.2f')
        os.system('clear')
        print(output,flush=True)
        rate.sleep()
