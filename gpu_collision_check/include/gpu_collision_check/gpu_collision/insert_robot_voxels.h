#ifndef INSERT_ROBOT_VOXELS_H
#define INSERT_ROBOT_VOXELS_H

// Gpu-Voxels urdf support in VS code (automatically added with Gpu-Voxels definitions in cmake)
#ifndef _BUILD_GVL_WITH_URDF_SUPPORT_   
#define _BUILD_GVL_WITH_URDF_SUPPORT_ 
#endif

// C++ stl
#include <iostream>
#include <string>

// Gpu-Voxels
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>

// Gpu-Voxels maps and lists
#include <gpu_voxels/voxelmap/BitVoxelMap.h>
#include <gpu_voxels/voxelmap/DistanceVoxelMap.h>
#include <gpu_voxels/voxellist/BitVoxelList.h>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>

// ROS msgs
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

// ROS service
#include <gpu_collision_check/TrajectoryForCollision.h>

class InsertRobotVoxels
{
    public:
        /* Struct for prediction horizon */
        typedef struct StructPredHor 
        {
            bool status;
            float minAngle;
            float maxAngle;
            float stepSize;
            int sweptNum;    
        } PredictionHorizonStruct;

        /* enum for swept */
        enum SweptType {ST_EMPTY, ST_LIST, ST_MAP};

        /* Boost voxel maps and list ptrs */
        typedef boost::shared_ptr<gpu_voxels::voxelmap::BitVectorVoxelMap> BitVectorVoxelMapPtr;
        typedef boost::shared_ptr<const gpu_voxels::voxelmap::BitVectorVoxelMap> BitVectorVoxelMapConstPtr;
        typedef boost::shared_ptr<gpu_voxels::voxelmap::DistanceVoxelMap> DistanceVoxelMapPtr;
        typedef boost::shared_ptr<const gpu_voxels::voxelmap::DistanceVoxelMap> DistanceVoxelMapConstPtr;
        typedef boost::shared_ptr<gpu_voxels::voxellist::BitVectorVoxelList> BitVectorVoxelListPtr;
        typedef boost::shared_ptr<const gpu_voxels::voxellist::BitVectorVoxelList> BitVectorVoxelListConstPtr;

        /* 
            Creates robot object for Gpu-Voxels.
            Initializes voxel map and gets parameters for prediction horizon from ROS parameter server, 
            if parameter dosen't exist on ROS parameter server returns default value.
            \param gvl - pointer on Gpu-Voxels lib
            \param mapSize - map size in VOXELS
            \param voxelSize - voxel size
        */
        InsertRobotVoxels(gpu_voxels::GpuVoxels *gvl, Eigen::Vector3i mapSize, float voxelSize);

        /* 
            If using exit() and object has automatic storage duration, destructor MUST be explicitly called! 
            Gpu-Voxels main part MUST finish with exit() call!
        */
        ~InsertRobotVoxels();

        /* 
            Initializes robot - CALL THIS METHOD FIRST!!
            Creates robot with \param robotName, and creates voxel map for robot and voxel list for swept. 
            \param robotURDF - path to URDF file (URDF must be in folder with .binvox files) 
            \param modelPath - if true use relative path from $GPU_VOXELS_MODEL_PATH, otherwise use absolute path! 
            \param moveBase - if true move base to position set in \param baseLinkPose 
            \param moveBase uses dummy joint names (loaded in constructor) from ROS parameter server!!
            Base can be manually moved with /method moveRobotBase(...)
        */
        void initializeRobot(std::string robotName, std::string robotURDF, bool moveBase = false, Eigen::Vector3f baseLinkPose = Eigen::Vector3f(0, 0, 0), bool modelPath = true);

        /* 
            This method NEEDS to be called if using swept! Call AFTER \method initializeRobot(...)!!
            Initialize swept. Select swept type: ST_MAP or ST_LIST!
            \param sweptType - swept type (map or list) 
        */
        void initializeSwept(SweptType ST);

        /*
            Initialize bounding box robot model. BB model is used for cuting robot out of scene. 
            Call AFTER \method initializeRobot(...)!!
            \param robotURDF - path to URDF
            \param modelPath - if true use relative path from $GPU_VOXELS_MODEL_PATH, otherwise use absolute path! 
        */
        void initializeRobotBB(std::string robotURDF, bool modelPath = true);

        /* 
            This method starts communication with ROS.
            Start subscribing to joint states topic. Topic type: sensor_msgs/JointState. Subscribes only on first call.
            Start ROS service server. Service type: gpu_collision_check/TrajectoryForCollision
            \param jointStatesTopicName - topic name
        */
        void startSubscribing(std::string jointStatesTopicName = "joint_states");

        /* Manually set joint values. Call updateRobotMap() after this method. This method is used for testing. */
        void setJointValuesManually(gpu_voxels::robot::JointValueMap jointValues);
        
        /* \return - true if robot is initialized. */
        bool isInitialized();

        /* Lock thread */
        void lockThread();

        /* Unlock thread */
        void unlockThread();

        /* 
            Get base link position 
            \return - base link position, if robots not initialized returns NaN.
        */
        Eigen::Vector3f getBaseLinkPose();

        /* 
            Get swept type (list or map).
            \return - swept type
        */
        SweptType getSweptType();

        /* 
            Get robots name.
            \return - robot name, if robots not initialized returns empty string.
        */
        std::string getRobotName();

        /* 
            Get name of robots voxelmap. Voxelmap type: MT_BITVECTOR_VOXELMAP.
            \return - map name, if robots not initialized returns empty string.  
        */
        std::string getRobotMapName();

        /* 
            Get name of robots voxelmap for visualization. Voxelmap type: MT_DISTANCE_VOXELMAP. 
            \return - map name, if robots not initialized returns empty string. 
        */
        std::string getRobotVisualizationMapName();

        /* 
            Get swept voxellist name. Voxellist type: MT_BITVECTOR_VOXELLIST. Use with swept type = ST_LIST.
            \return - if swept type = ST_LIST returns list name, otherwise empty string. 
        */
        std::string getSweptListName();

        /* 
            Get swept voxelmap name. Voxelmap type: MT_BITVECTOR_VOXELMAP. Use with swept type = ST_MAP.
            \return - if swept type = ST_MAP returns map name, otherwise empty string.  
        */
        std::string getSweptMapName();

        /* 
            Get name of swept voxelmap for visualization. Voxelmap type: MT_DISTANCE_VOXELMAP. Use with swept type = ST_MAP.
            \return - if swept type = ST_MAP returns map name, otherwise empty string.  
        */
        std::string getSweptVisualizationMapName();

        /* 
            Get ROBOT voxelmap. Map type: MT_BITVECTOR_VOXELMAP.
            \return - shared pointer to map, if robots not initialized returns nullptr.
        */
        BitVectorVoxelMapPtr getRobotVoxelMap();

        /* 
            Get ROBOT voxelmap for VISUALIZATION. Map type: MT_DISTANCE_VOXELMAP.
            \return - shared pointer to map, if robots not initialized returns nullptr. 
        */
        DistanceVoxelMapPtr getRobotVisualizationVoxelMap();

        /* 
            Get SWEPT voxellist. List type - MT_BITVECTOR_VOXELLIST. Use with swept type = ST_LIST.
            \return - if swept type = ST_LIST returns shared ptr to voxellist, otherwise nullptr.  
        */
        BitVectorVoxelListPtr getSweptVoxelList();

        /* 
            Get SWEPT voxelmap. Map type - MT_BITVECTOR_VOXELMAP. Use with swept type = ST_MAP.
            \return - if swept type = ST_MAP returns shared ptr to voxelmap, otherwise nullptr. 
        */
        BitVectorVoxelMapPtr getSweptVoxelMap();

        /* 
            Get SWEPT voxelmap for VISUALIZATION. Map type: MT_DISTANCE_VOXELMAP. Use with swept type = ST_MAP.
            \return - if swept type = ST_MAP returns shared ptr to voxelmap, otherwise nullptr. 
        */
        DistanceVoxelMapPtr getSweptVisualizationVoxelMap();

        /* 
            Get joint values. Thread safe!
            \return - joint values, if robots not initialized returns empty JointValueMap.
        */
        gpu_voxels::robot::JointValueMap getJointValues();

        /* 
            Get prediction horizon info. 
            \return - prediction horizon
        */
        PredictionHorizonStruct getPredictionHorizonInfo();

        /* Enables prediction horizon. */
        void enablePredictionHorizon();

        /* Disables prediction horizon. */
        void disablePredictionHorizon();

        /* 
            Manually set prediction horizon parameters. 
            Overrides parameters from constructor (parameters from ROS parameter server).
            \param minAngle - minimal joints angle
            \param maxAngle - maximal joints angle
            \param stepSize - step size of joints movement
            \param sweptNum - number of swepts
        */
        void configPredictionHorizon(float minAngle, float maxAngle, float stepSize, int sweptNum);

        /* Send map to visualizer. NOT thread safe! */
        void visualizeRobot();

         /* 
            Send list/map to visualizer. NOT thread safe!
            INFO: voxellist/map for swept volumes is EMPTY until call to \method updateSweptMap().
        */
        void visualizeSwept();
        
        /*
            Robot must have three dummy joints for movement in GPU-Voxels visualizer!! NOT thread safe!
            Default dummy joints names: base_x, base_y and base_z!! Dummy joints are set ONLY in 
            URDF for GPU-voxels (use GUI for URDF generation and mesh voxelization). 
            \param basePose - new base position
            \param visualize - automatic visualization
            \param dummyX - name of dummy joint for base movement in x-axis 
            \param dummyY - name of dummy joint for base movement in y-axis
            \param dummyY - name of dummy joint for base movement in z-axis
        */      
        void moveRobotBase(Eigen::Vector3f basePose, bool visualize, std::string dummyX = "base_x", std::string dummyY = "base_y", std::string dummyZ = "base_z");

        /* 
            Linear interpolation in joint space.
            \param start - joints start position
            \param end - joints end position
            \param ratio - interpolation ratio [0, 1]
            \return - JointValueMap with interpolated joints
        */
        static gpu_voxels::robot::JointValueMap jointsLinearInterpolation(gpu_voxels::robot::JointValueMap start, gpu_voxels::robot::JointValueMap end, float ratio);

        /* Updates robot map with current pose (pose WILL be updated). Thread safe! */
        void updateRobotMap();

        /* 
            Update swept map. It WON'T update robot pose (call after \method updateRobotMap() 
            or ros::SpinOnce())!! Thread safe! 
        */
        void updateSweptMap();

        /* ROS service callback. Can be used for manually adding trajectory for collision check! */
        bool trajectoryServiceCallback(gpu_collision_check::TrajectoryForCollision::Request &req, 
                                       gpu_collision_check::TrajectoryForCollision::Response &res);

        /* Return current trajectory */
        trajectory_msgs::JointTrajectory getCurrentTrajectory();

        /* Return current point on trajectory */
        int getCurrentPointOnTrajectory();

    private:
        /*
            Loads robot into map. NOT thread safe!
            \param joints - joint value map
            \param swept - if true use map for robot with SWEPT volumes, if false use map with ONLY robot
            \param ID - voxels group ID (1 for robot position, 5+ for swept volumes)
         */
        void loadRobotIntoMap(gpu_voxels::robot::JointValueMap joints, bool swept = false, gpu_voxels::BitVoxelMeaning ID = eBVM_OCCUPIED);

        /*
            Creates swept volume and loads swept into voxellist/map.
            \param start - joints start position
            \param end - joints end position
            \param sweptNum - number of swept levels
            \param ratio - prediction horizon ratio
         */
        void createSweptVolume(gpu_voxels::robot::JointValueMap start, gpu_voxels::robot::JointValueMap end, int sweptNum, float ratio);

        /* 
            Calculates ratio r = [0, 1] for prediction horizon. Works only for KUKA kr10 and other robots with same joint names!
            \param start - joints start position
            \param end - joints end position
            \param stepSize - step size for prediction horizon
            \param minAngle - minimal angle for prediction horizon
            \param maxAngle - maximal angle for prediction horizon
            \return - ratio r
        */
        virtual float calculateRatio(gpu_voxels::robot::JointValueMap start, gpu_voxels::robot::JointValueMap end, float stepSize ,float minAngle, float maxAngle);

        /* 
            Select next point in trajectory.
            \param currentValue - current joint state
            \param distanceToPoint - distance in joint space to next point
            \return - next point in joint space 
        */
        gpu_voxels::robot::JointValueMap selectNextPoint(gpu_voxels::robot::JointValueMap currentValue, float distanceToPoint);

        /* Joint states callback. Thread safe! */
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

        /* Global and private node handle */
        ros::NodeHandle n, np;

        /* Joint states subscriber (global queue) */
        ros::Subscriber sub;

        /* ROS service server for adding trajectory */
        ros::ServiceServer service;

        /* Current joint values */
        gpu_voxels::robot::JointValueMap jointValues;

        /* Pointer on gpu_voxels base object */
        gpu_voxels::GpuVoxels *gvl;

        /* Mutex for multithread synchronization */
        boost::mutex mtx;

        /* Maps for robot */
        BitVectorVoxelMapPtr robotVoxelMap, sweptVoxelMap;
        DistanceVoxelMapPtr robotVisVoxelMap, sweptVisVoxelMap;

        /* Swept list */
        BitVectorVoxelListPtr sweptVoxelList;

        /* Struct with info of prediction horizon */
        PredictionHorizonStruct predictionHorizon;

        /* Strings */
        std::string robotName, robotBBName, robotMapName, robotVisMapName, dummyX, dummyY, dummyZ, topicName; 
        std::string sweptMapName, sweptVisMapName, sweptListName, serviceName, robotBB10Name;

        /* Init robot only ONCE */
        bool alreadyInit, alreadyInitBB, alreadySub, trajectoryLoaded;

        /* Swept type */
        SweptType sweptType;

        /* Base link position */
        Eigen::Vector3f baseLinkPose;

        /* Current trajectory for swept */
        trajectory_msgs::JointTrajectory currentTraj;

        /* Trajectory points counter */
        int currentTrajPoint;
};
#endif //INSERT_ROBOT_VOXELS_H