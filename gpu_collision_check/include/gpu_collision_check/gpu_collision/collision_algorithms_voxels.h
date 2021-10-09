#ifndef COLLISION_ALGORITHMS_VOXELS_H
#define COLLISION_ALGORITHMS_VOXELS_H

// Gpu-Voxels urdf support in VS code (automatically added with Gpu-Voxels definitions in cmake)
#ifndef _BUILD_GVL_WITH_URDF_SUPPORT_   
#define _BUILD_GVL_WITH_URDF_SUPPORT_ 
#endif

// C++ stl
#include <iostream>
#include <string>

// headers for CPU and GPU based voxelization
#include <gpu_collision_check/gpu_collision/insert_robot_voxels.h>
#include <gpu_collision_check/gpu_collision/insert_scene_voxels.h>
#include <helpers_gpu_voxels/timer/timer_wrapper.h>

// Gpu-Voxels
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class CollisionAlgorithmsVoxels
{
    public:
        /* 
            Creates object for voxels based collision algorithms.
            \param gvl - pointer on Gpu-Voxels base class
            \param robot - pointer on robot
            \param scene - pointer on scene 
        */
        CollisionAlgorithmsVoxels(gpu_voxels::GpuVoxels *gvl, InsertRobotVoxels *robot, InsertSceneVoxels *scene);

        /* Empty destructor */
        ~CollisionAlgorithmsVoxels();

        /* Print info for CollisionAlorithms class */
        friend std::ostream &operator<<(std::ostream &output, const CollisionAlgorithmsVoxels &obj);

        /* 
            Connect to action server.
            \param name - name of action server
        */
        void connect2ActionServer(std::string name);

        /* 
            Start loop with ONLY robot map. This method is used for testing.
            \param hz - loop rate
            \param visualize - if true send map to visualizer
         */
        void startLoopRobot(float hz, bool visualize = true);

        /* 
            Start loop with ONLY scene map. This method is used for testing.
            \param hz - loop rate (only for visualization)
            \param visualize - if true send map to visualizer
            \param removeRobot - if true cut robot out of scene
            \param filter - if true filter scene
         */
        void startLoopScene(float hz, bool removeRobot = true, bool filter = true, bool visualize = true);

        /*
            Start algorithm for collision check, based on prediction horizon.
            \param filter - filter scene after removing robot
            \param collThreshold - number of voxels in collision after which algorithm detects collision
            \param jumpThreshold - number of consecutive collision detections (eliminates sudden collision "jumps")
        */
        void collisionCheckPredictionHorizon(bool filter = false, int collThreshold = 0, int jumpThreshold = 0);

        /* 
            Start algorithm for collision check, based on safe zones.
         */
        void collisionCheckSafeZones();

    private:
        /* Robot action when collision is detected. Used by \method collisionCheckPredictionHorizon(...) */
        virtual void collisionRobotAction(int numColl, float scalingV, float scalingA);

        /* Robot actions in different zones. Used by \method collisionCheckSafeZones(...) for zones around BASE LINK */
        virtual void safeZonesBaseLinkRobotAction(float distanceToBaseLink);

        /* Global and private node handle */
        ros::NodeHandle n, np;

        /* ROS service for TOPP-RA */
        ros::ServiceClient toppRA;

        /* Publisher for cmd */
        ros::Publisher pubCmd;

        /* Robot object */
        InsertRobotVoxels *robot;

        /*Scene object*/
        InsertSceneVoxels *scene;

        /* Timer uses wall time*/
        TimerWrapper timer; 

        /* Pointer on gpu_voxels base object */
        gpu_voxels::GpuVoxels *gvl;

        /* Mutex for multithread synchronization */
        boost::mutex mtx;

        /* Strings */
        std::string targetFrame, oneKinectMapName, twoKinectsMapName, firstKinectMapName, secondKinectMapName, actionServerName;

        /* Action server */
        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> actionServer;

        /* Connected to AS */
        bool actionServerConnected;

        /* enum for detection of nullptr */
        enum RobotScene {RobotNULL, SceneNULL, RobotSceneNULL, RobotSceneOK};
        RobotScene checkNULL;
};
#endif //COLLISION_ALGORITHMS_VOXELS_H