#!/usr/bin/env python

import rospy
import sys

from rosbag import Bag
with Bag(sys.argv[3], 'w') as Y:
    for topic, msg, t in Bag(sys.argv[1]):
        Y.write(sys.argv[4] if topic == sys.argv[2] else topic, msg, t)