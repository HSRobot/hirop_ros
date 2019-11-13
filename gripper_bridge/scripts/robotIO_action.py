#! /usr/bin/env python

import roslib
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from hirop_msgs.srv import *
from sensor_msgs.msg import JointState
import time
import thread


close_client = None
open_client = None
joint_names = ['pickup_gripper_joint']
last_joint_state = [0]
open_server = 'openGripper'
close_server = 'closeGripper'
action_server = "gripper_controller/follow_joint_trajectory"

def on_goal(goal_handle):

    global last_joint_state
    print("in on_goal")
    goal_handle.set_accepted()
    traj = goal_handle.get_goal().trajectory
    print(traj)
    open_gripper =  False
    last_joint_state = traj.points[1].positions

    if traj.points[1].positions[3] == 0.0:
            open_gripper = True

    if open_gripper:
        print("open gripper")
        open_client()
    else:
        print("close gripper")
        close_client()

    time.sleep(1)
    goal_handle.set_succeeded()

def on_cancel():
	print("in on_cache")

def publish_joint_state(tname):
    print("start publish_joint_state")
    pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        msg = JointState()
        msg.header.stamp = now
        msg.header.frame_id = "From real-time state data"
        msg.name = joint_names
        msg.position = last_joint_state
        pub_joint_states.publish(msg)
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('gripper_action_server')

    server = actionlib.ActionServer(action_server, FollowJointTrajectoryAction,
                                             on_goal, on_cancel, auto_start=False)

    rospy.wait_for_service(open_server)
    rospy.wait_for_service(close_server)


    print("server is ok")

    open_client = rospy.ServiceProxy(open_server, openGripper)
    close_client = rospy.ServiceProxy(close_server, closeGripper)
    server.start()
    try:
        thread.start_new_thread(publish_joint_state,("publish_joint_state",))
    except:
        print "Error: unable to start thread"
    rospy.spin()
