#!/usr/bin/env python

import roslib
import rospy

from iai_control_msgs.msg import MultiJointVelocityImpedanceCommand

from optparse import OptionParser

parser = OptionParser()
parser.add_option("-v", "--velocity", dest="velocity",
                  help="Velocity commanded to each joint (VELOCITY) Default=%default (rads/s)", metavar="VELOCITY", default= 0.0)
parser.add_option("-s", "--stiffness", dest="stiffness",
                  help="Stiffness commanded to each joint (STIFFNESS) Default=%default (Nm/rad)", metavar="STIFFNESS", default= 80.0)
parser.add_option("-d", "--damping", dest="damping",
                  help="Damping commanded to each joint (DAMPING) Default=%default", metavar="DAMPING", default= 0.7)
parser.add_option("-t", "--topic", dest="topic",
                  help="Command topic Default=%default", metavar="TOPIC", default="/right_arm_vel_controller/command")


(options, args) = parser.parse_args()

def main():
    rospy.init_node("cmd_arm_vel_imp", anonymous=True)

    pub = rospy.Publisher(options.topic, MultiJointVelocityImpedanceCommand, queue_size=1, latch=True)

    cmd = MultiJointVelocityImpedanceCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.velocity = [ options.velocity ] * 7
    cmd.stiffness = [ options.stiffness ] * 7
    cmd.damping = [ options.damping ] * 7

    rospy.loginfo("Publishing to:")
    print options.topic
    rospy.loginfo("Publishing the following command:")
    print cmd
    pub.publish(cmd)
    rospy.loginfo("Done publishing")

    rospy.sleep(2.0)

if __name__ == "__main__":
    main()
