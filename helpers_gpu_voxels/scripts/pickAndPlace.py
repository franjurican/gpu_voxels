#!/usr/bin/env python

# python modules
import numpy as np
import sys

# ROS
import rospy
import actionlib

# ROS msgs
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from gpu_collision_check.srv import TrajectoryForCollision, TrajectoryForCollisionRequest
from std_msgs.msg import Header

class MoveKUKA():
    def __init__(self, actionServerName, useService, serviceName = ""):
        # action client
        self.clientAction = actionlib.SimpleActionClient(actionServerName, FollowJointTrajectoryAction)

        # ROS service for collision check
        self.service = rospy.ServiceProxy(serviceName, TrajectoryForCollision)

        # try to connect to action server for 5 seconds
        if self.clientAction.wait_for_server(rospy.Duration(5)):
            rospy.loginfo('Connected to action server: ' + actionServerName)
        else:
            rospy.loginfo('Couldn\'t connect to action server: ' + actionServerName)
            sys.exit('Exiting ...')

        # try to connect to ROS service for 5 seconds
        self.useService = useService
        if self.useService:
            try:
                self.service.wait_for_service(5)
                rospy.loginfo('Connected to ROS service: ' + serviceName)
            except rospy.exceptions.ROSException, e:
                rospy.loginfo('Couldn\'t connect to ROS service: ' + serviceName)
                rospy.loginfo("%s", e)
                sys.exit('Exiting ...')

    def moveRobot(self, goalPosition, goalDuration, goalName = 'right'):
        # empty messages
        goalMsg = FollowJointTrajectoryGoal()
        position1 = JointTrajectoryPoint()
        h = Header()
 
        # set goal position and end time
        position1.positions = goalPosition
        position1.time_from_start.secs = goalDuration

        # current ROS time for headers
        h.stamp = rospy.Time.now()

        # fill goal message 
        goalMsg.trajectory.header = h
        goalMsg.trajectory.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        goalMsg.trajectory.points.append(position1)

        # send trajectory for collision check
        if self.useService:
            collisionCheck = TrajectoryForCollisionRequest()
            collisionCheck.waypoints = goalMsg.trajectory
            response = self.service.call(collisionCheck)
            print response.error.data

        # move robot
        rospy.loginfo('Started position ' + '"' + goalName + ': ' + str(goalPosition) + '"')
        self.clientAction.send_goal(goalMsg)
        self.clientAction.wait_for_result()
        time=(rospy.Time.now().to_nsec() - h.stamp.to_nsec())*1e-6
        rospy.loginfo('Done executing position ' + '"' + goalName + '"' + " in " + str(time) + '[ms]')

    def start(self, trajExeTime):
        home = np.array([0, -np.pi/2, np.pi/2, 0, 0, 0])
        right = np.array([np.pi/4, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        left = np.array([-np.pi/4, -np.pi/2, np.pi/2, 0, np.pi/2, 0])

        # first move to home position
        self.moveRobot(home, trajExeTime, 'home')
        side = 'right'

        # start moving left-right
        while not rospy.is_shutdown():
            if side == 'right':
                self.moveRobot(left, trajExeTime, side)
                side = 'left'
            else:
                self.moveRobot(right, trajExeTime, side)
                side = 'right'
            rospy.sleep(2)

if __name__ == '__main__':
    # init ros
    rospy.init_node('kukaMove')

    # init node
    node = MoveKUKA("/joint_trajectory_action", False, "/testRobotVoxels/add_trajectory_for_collision_check")
    # node = MoveKUKA("/position_trajectory_controller/follow_joint_trajectory", False, "/testRobotVoxels/add_trajectory_for_collision_check")

    # mode selection
    rospy.loginfo("Mode selection. For auto mode enter 'a', for manual mode enter 'm'.")
    mode = raw_input("Enter value (a or m): ")
    
    # start moving kuka
    if mode == "a":
        rospy.loginfo("Starting auto mode ...") 
        node.start(2)

    elif mode == "m":
        # definition of available positions
        right = np.array([np.pi/4, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        left = np.array([-np.pi/4, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        home = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        pose = np.array([right, left, home])
        poseName = ["right", "left", "home"]

        # first move robot to right: k=0 -> right, k=1 -> left, k=2 -> home
        k = 0
        rospy.loginfo("Starting manual mode ...")
        while True:
            # info about next pose
            if poseName[k] == "right":
                rospy.loginfo("Move to pose 'right'?")
            elif poseName[k] == "left":
                rospy.loginfo("Move to pose 'left'?")
            else:
                rospy.loginfo("Move to pose 'home'?")

            # get confirmation 
            start = raw_input("Enter value (y or n): ")

            # move robot?
            if start == "y":
                node.moveRobot(pose[k], 5, poseName[k])
                k = (k + 1) % 3
            else:
                rospy.loginfo("Exiting ...")
                break
    else:
        rospy.loginfo("Wrong entry for mode selection!") 