#!/usr/bin/env python

# python modules
import numpy as np
import sys
import copy

# ROS
import rospy
import actionlib
import moveit_commander
from tf.transformations import quaternion_from_euler

# ROS msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from gpu_collision_check.srv import TrajectoryForCollision, TrajectoryForCollisionRequest
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped, Pose

class ExperimentTraj():
    def __init__(self, collisionCheck = False, useActionlib = True):
        ####################################################################################################
        # UPISI IME ACTION SERVERA, ROS SERVICE-a ZA KOLIZIJU, ROS SERVICE-a ZA TOPP_RA I CMD PUBLISHER-a! #
        ####################################################################################################
        self.actionServerName = "/position_trajectory_controller/follow_joint_trajectory"
        self.collisionServiceName = "/testRobotVoxels/add_trajectory_for_collision_check"
        self.toppServiceName = "/generate_toppra_trajectory"
        self.cmdPublisherName = "/position_trajectory_controller/command" # cmd publisher ako se NE koristi actionlib!!
        self.groupName = "kuka_kr10r1100sixx" # ime grupe za moveit
        self.ikServiceName = "/compute_ik" # inverzna kinematika 

        # max brzine i akceleracije za topp ra
        self.maxVel = np.array([3.83, 3.66, 4.71, 6.65, 5.43, 8.59])*0.02
        self.maxAcc = np.array([10.0, 3.0, 10.0, 10.0, 10.0, 10.0])*0.05
        ###############################################################################################################

        # za simulaciju "cekaj" vrijeme
        while rospy.Time().now().is_zero():
            continue
        rospy.loginfo("Vrijeme je spremno!")

        # spremi parametre
        self.collisionCheckON = collisionCheck
        self.useActionlib = useActionlib

        # action client, ROS service za koliziju, ROS service za topp_ra i cmd publisher
        self.actionClient = actionlib.SimpleActionClient(self.actionServerName, FollowJointTrajectoryAction)
        self.collisionService = rospy.ServiceProxy(self.collisionServiceName, TrajectoryForCollision)
        self.toppService = rospy.ServiceProxy(self.toppServiceName, GenerateTrajectory)
        self.cmdPublisher = rospy.Publisher(self.cmdPublisherName, JointTrajectory, queue_size=10)

        self.scan_waypoints = []

        ################################################
        # kreiranje objekta za pokretanje trajektorije #
        ################################################
        if self.useActionlib:
            rospy.loginfo('Koristim action server!')
            if self.actionClient.wait_for_server(rospy.Duration(5)):
                rospy.loginfo('Spojen na action server: ' + self.actionServerName)
            else:
                rospy.logerr('Nisam se uspio spojiti na action server: ' + self.actionServerName)
                rospy.logerr('Izlazim ...')
                sys.exit(1)
        else:
            rospy.loginfo('Koristim cmd publisher: ' + self.cmdPublisherName)
        
        #################################################
        # spajanje na ROS service za detekciju kolizije #
        #################################################
        if self.collisionCheckON:
            rospy.loginfo("Provjeravam koliziju!")
            try:
                self.collisionService.wait_for_service(5)
                rospy.loginfo('Spojen na ROS service za detekciju kolizije: ' + self.collisionServiceName)
            except rospy.exceptions.ROSException, e:
                rospy.logerr('Ne mogu se spojiti na ROS service za detekciju kolizije: ' + self.collisionServiceName)
                rospy.logerr("%s", e)
                rospy.logerr('Izlazim ...')
                sys.exit(2)
        
        ######################################
        # spajanje na ROS service za TOPP RA #
        ######################################
        rospy.loginfo("Koristim TOPP RA za generiranje trajektorije!")
        try:
            self.toppService.wait_for_service(5)
            rospy.loginfo('Spojen na ROS service za TOPP RA: ' + self.toppServiceName)
        except rospy.exceptions.ROSException, e:
            rospy.logerr('Ne mogu se spojiti na ROS service za TOPP RA: ' + self.toppServiceName)
            rospy.logerr("%s", e)
            rospy.logerr('Izlazim ...')
            sys.exit(3)
        
        ############################
        # inicijalizacija moveit-a #
        ############################
        group = moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(self.groupName)
        self.ikService = rospy.ServiceProxy(self.ikServiceName, GetPositionIK)

        # spajanje na service za inverznu kinematiku
        try:
            self.ikService.wait_for_service(5)
            rospy.loginfo('Spojen na ROS service za IK: ' + self.ikServiceName)
        except rospy.exceptions.ROSException, e:
            rospy.logerr('Ne mogu se spojiti na ROS service za IK: ' + self.ikServiceName)
            rospy.logerr("%s", e)
            rospy.logerr('Izlazim ...')
            sys.exit(4)

    ################################################################
    # metoda za zadavanje trajektorije u koordinatnom sustavu baze #
    ################################################################
    def cartesianRun(self, start, end):
        test = self.inverseKinematics(self.group.get_current_pose(), "test")
        self.jointSpaceRun(start, test)
    
    ###########################################
    # metoda za racunanje inverzne kinematike #
    ###########################################
    def inverseKinematics(self, pose, name = ""):
        # zahtjev za IK
        req = PositionIKRequest()
        req.group_name = self.groupName
        req.ik_link_name = self.group.get_end_effector_link()
        req.pose_stamped = pose
        req.timeout.secs = 0.5
        req.avoid_collisions = False
        req.attempts = 5

        # izracunaj IK
        resp = self.ikService(req)
        jointPositions = np.array(resp.solution.joint_state.position)

        if jointPositions.size == 0:
            rospy.logerr("Za poziciju '" + name + "' IK nije uspijela!")
            sys.exit(5)
        else:
            return jointPositions

    ####################################################
    # metoda za zadavanje trajektorije u joint space-u #
    ####################################################

    def planPath(self, waypoints):
        group = self.group

        (plan, fraction) = group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.02,  # eef_step
            0.0)  # jump_threshold

        print fraction
        #print plan
        if fraction > 0.95:
            print "Planning OK!"
        else:
            print "Planning failed!"
        
        return plan  # the service Response class, in this case MyCustomServiceMessageResponse

    def cartesianSpaceRun(self, sampling_hz = 250):
        # trenutni joint states
        plan = self.planPath(self.scan_waypoints)

        # popuni trajektoriju s pozicijama i ogranicenjima za topp ra
        traj = JointTrajectory()
        traj.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']

        for temp in plan.joint_trajectory.points:
            point = JointTrajectoryPoint()
            point.positions = temp.positions
            point.velocities = self.maxVel
            point.accelerations = self.maxAcc
            traj.points.append(point)

        # generiranje trajektorije - topp ra
        req = GenerateTrajectoryRequest()
        req.waypoints = traj
        req.sampling_frequency = sampling_hz
        
        res = self.toppService(req)
        res.trajectory.header.stamp = rospy.Time().now()
        res.trajectory.joint_names = traj.joint_names

        # posalji trajektoriju algoritmu detekcije kolizije
        if self.collisionCheckON:
            rospy.loginfo("Saljem trajektoriju algoritmu detekcije kolizije pomocu service-a: " + self.collisionServiceName)
            collisionCheck = TrajectoryForCollisionRequest()
            collisionCheck.waypoints = res.trajectory
            response = self.collisionService(collisionCheck)
            rospy.loginfo(response.error.data)

        # pokreni trajektoriju
        rospy.logwarn("Pokrecem izvrsavanje trajektorije na robotu!")
        if self.useActionlib:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = res.trajectory
            self.actionClient.send_goal_and_wait(goal)
        else:
            self.cmdPublisher.publish(res.trajectory)
            rospy.spin()

    def jointSpaceRun(self, start, end, sampling_hz = 70):
        # trenutni joint states
        current = np.array(self.group.get_current_joint_values())

        # poslozi tocke putanje
        path = [current, start, end]

        # popuni trajektoriju s pozicijama i ogranicenjima za topp ra
        traj = JointTrajectory()
        traj.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']

        for temp in path:
            point = JointTrajectoryPoint()
            point.positions = temp
            point.velocities = self.maxVel
            point.accelerations = self.maxAcc
            traj.points.append(point)

        # generiranje trajektorije - topp ra
        req = GenerateTrajectoryRequest()
        req.waypoints = traj
        req.sampling_frequency = sampling_hz
        
        res = self.toppService(req)
        res.trajectory.header.stamp = rospy.Time().now()
        res.trajectory.joint_names = traj.joint_names

        # posalji trajektoriju algoritmu detekcije kolizije
        if self.collisionCheckON:
            rospy.loginfo("Saljem trajektoriju algoritmu detekcije kolizije pomocu service-a: " + self.collisionServiceName)
            collisionCheck = TrajectoryForCollisionRequest()
            collisionCheck.waypoints = res.trajectory
            response = self.collisionService(collisionCheck)
            rospy.loginfo(response.error.data)

        # pokreni trajektoriju
        rospy.logwarn("Pokrecem izvrsavanje trajektorije na robotu!")
        if self.useActionlib:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = res.trajectory
            self.actionClient.send_goal_and_wait(goal)
        else:
            self.cmdPublisher.publish(res.trajectory)
            rospy.spin()
    
    #####################################################
    # skaliraj brzine i akceleracije joint-a za topp ra #
    #####################################################
    def scaleVelAcc(self, scaleVel, scaleAcc):
        self.maxVel = self.maxVel * scaleVel
        self.maxAcc = self.maxAcc * scaleAcc

    #####################################################
    # Kreiranje tocaka za experiment#
    #####################################################
    def generatePointsLeftRight(self):     
        self.scan_waypoints = []  

        # left-right
        #Point 1
        waypoint = Pose()
        waypoint.position.x = 0.7
        waypoint.position.y = -0.4
        waypoint.position.z = 0.4
        waypoint.orientation.x = 0.0
        waypoint.orientation.y = 0.707
        waypoint.orientation.z = 0.0
        waypoint.orientation.w = 0.707
        waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
        self.scan_waypoints.append(waypoint)

        #Point 1
        waypoint = Pose()
        waypoint.position.x = 0.7
        waypoint.position.y = 0.4
        waypoint.position.z = 0.4
        waypoint.orientation.x = 0.0
        waypoint.orientation.y = 0.707
        waypoint.orientation.z = 0.0
        waypoint.orientation.w = 0.707
        waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
        self.scan_waypoints.append(waypoint)

        #Point 1
        waypoint = Pose()
        waypoint.position.x = 0.7
        waypoint.position.y = -0.4
        waypoint.position.z = 0.4
        waypoint.orientation.x = 0.0
        waypoint.orientation.y = 0.707
        waypoint.orientation.z = 0.0
        waypoint.orientation.w = 0.707
        waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
        self.scan_waypoints.append(waypoint)

    def createLawnMoverPath(self, start_x, start_y, start_z, step_x, step_y, step_z, N):
        self.scan_waypoints = []

        for i in range(0, N):

            # point 1
            waypoint = Pose()
            waypoint.position.x = start_x 
            waypoint.position.y = start_y + i*step_y
            waypoint.position.z = start_z + i*step_z
            waypoint.orientation.x = 0.0
            waypoint.orientation.y = 0.707
            waypoint.orientation.z = 0.0
            waypoint.orientation.w = 0.707
            waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
            self.scan_waypoints.append(waypoint)

            # point 2
            waypoint = Pose()
            waypoint.position.x = start_x + step_x
            waypoint.position.y = start_y + i*step_y
            waypoint.position.z = start_z + i*step_z
            waypoint.orientation.x = 0.0
            waypoint.orientation.y = 0.707
            waypoint.orientation.z = 0.0
            waypoint.orientation.w = 0.707
            waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
            self.scan_waypoints.append(waypoint)

            # point 3
            waypoint = Pose()
            waypoint.position.x = start_x + step_x
            waypoint.position.y = start_y + i*step_y + step_y/2
            waypoint.position.z = start_z + i*step_z + step_z/2
            waypoint.orientation.x = 0.0
            waypoint.orientation.y = 0.707
            waypoint.orientation.z = 0.0
            waypoint.orientation.w = 0.707
            waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
            self.scan_waypoints.append(waypoint)

            # point 4
            waypoint = Pose()
            waypoint.position.x = start_x 
            waypoint.position.y = start_y + i*step_y + step_y/2
            waypoint.position.z = start_z + i*step_z + step_z/2
            waypoint.orientation.x = 0.0
            waypoint.orientation.y = 0.707
            waypoint.orientation.z = 0.0
            waypoint.orientation.w = 0.707
            waypoint.orientation = self.normalizeQuaternion(waypoint.orientation)
            self.scan_waypoints.append(waypoint)
    def normalizeQuaternion(self, q):
        q_n = copy.deepcopy(q)

        n = copy.deepcopy((q.x**2 + q.y**2 + q.z**2 + q.w**2 )**0.5)
        q_n.x = q.x / n
        q_n.y = q.y / n
        q_n.z = q.z / n
        q_n.w = q.w / n

        return q_n


##################
# start ROS node #
##################
if __name__ == "__main__":
    rospy.init_node("experiment_RAL")
    obj = ExperimentTraj(True, True)

    start = np.array([-0.8, -0.55, 1.1, 0.0, 1.0, 1.0])
    end = np.array([0, -0.55, 1.1, 0.0, 1.0, 1.0])
    

    #obj.generatePointsLeftRight()
    obj.createLawnMoverPath(0.5, -0.4, 0.4, 0.3, 0.1, 0.0, 8)
    
    scale = 0.5
    obj.scaleVelAcc(5*scale, 2*scale)
    obj.cartesianSpaceRun(10)
    #obj.jointSpaceRun(start, end, 250)

    '''
    # postavi pocetnu i zavrsnu tocku trajektorije 
    start = np.array([-0.8, -0.55, 1.1, 0.0, 1.0, 1.0])
    end = np.array([0, -0.55, 1.1, 0.0, 1.0, 1.0])

    # pokreni trajektoriju
    obj.scaleVelAcc(5, 2)
    obj.jointSpaceRun(start, end, 250)
    '''