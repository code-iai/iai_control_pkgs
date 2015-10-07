#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
    rospy.init_node('move_arm', anonymous=True)
    pub = rospy.Publisher('/l_arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)
    traj = JointTrajectory()
    traj.joint_names = ['l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint',
        'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
    point = JointTrajectoryPoint()
    point.positions = [0.593, 1.265, 0.964, 0.524, -2.1, -0.067, 4.419]
    point.time_from_start = rospy.Time(4)
    traj.points = [point]
    pub.publish(traj)
    rospy.loginfo('Moving arm')
