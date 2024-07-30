#!/usr/bin/env python

import rospy

class TopicHandler:
    def __init__(self, topic, msg_type, max_msgs=1, name=None):
        self.name = topic if name == None else name
        self.topic = topic
        self.msg_type = msg_type
        self.max_msgs = max_msgs
        assert max_msgs >= 1

        self.sub = rospy.Subscriber(self.topic, msg_type, self.callback)
        self.msgs = []

    def callback(self,msg):
        if len(self.msgs) >= self.max_msgs:
            self.msgs = self.msgs[1:]
        self.msgs.append(msg)
        assert len(self.msgs) <= self.max_msgs

    def get(self):
        msgs = self.msgs
        self.msgs = []
        return msgs
