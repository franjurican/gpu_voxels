#ifndef INSERT_SCENE_VOXELS_H
#define INSERT_SCENE_VOXELS_H

// Gpu-Voxels urdf support in VS code (automatically added with Gpu-Voxels definitions in cmake)
#ifndef _BUILD_GVL_WITH_URDF_SUPPORT_   
#define _BUILD_GVL_WITH_URDF_SUPPORT_ 
#endif

// C++ stl
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

// headers for robot and timer class
#include <gpu_collision_check/gpu_collision/insert_robot_voxels.h>
#include <helpers_gpu_voxels/timer/timer_wrapper.h>

// Gpu-Voxels
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>

// Gpu-Voxels maps
#include <gpu_voxels/GpuVoxelsMap.h>
#include <gpu_voxels/voxelmap/ProbVoxelMap.h>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>

// ROS msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

class InsertSceneVoxels
{
    public:

        /* Boost shared ptr for probab map */ 
        typedef boost::shared_ptr<voxelmap::ProbVoxelMap> ProbVoxelMapPtr;
        typedef boost::shared_ptr<const voxelmap::ProbVoxelMap> ProbVoxelMapConstPtr;

        /* 
            Creates scene object for Gpu-Voxels.
            \param gvl - pointer to Gpu-Voxels base class
            \param mapSize - map size in voxels
            \param voxelSize - voxel size
            \param originOffset - origin Offset in Gpu-Voxels
            \param kinectNum - number of kinects  
        */
        InsertSceneVoxels(gpu_voxels::GpuVoxels *gvl, Eigen::Vector3i mapSize, float voxelSize, Eigen::Vector3f originOffset = Eigen::Vector3f(0, 0, 0));

        /* 
            If using exit() and object has automatic storage duration, destructor MUST be explicitly called! 
            Gpu-Voxels main part MUST finish with exit() call!
        */
        ~InsertSceneVoxels();

        /*
            Voxelizes PointCloud from kinect and transforms PointCloud into targetFrame. Type of scene map is: MT_PROBAB_VOXELMAP!
            \param kinectName1 - name of first kinect
            \param kinectTopicName1 - topic name for depth part of first kinect
            \param targetFrame - name of target frame for transformation (transformation should be broadcasted on tf!)
        */
        void registerKinect(std::string kinectName1, std::string kinectTopicName1, std::string targetFrame = "world");

        /*
            Voxelizes two PointClouds from kinects and transforms PointClouds into targetFrame. Type of scene map is: MT_PROBAB_VOXELMAP!
            \param kinectName1 - name of first kinect
            \param kinectTopicName1 - topic name for depth part of first kinect
            \param kinectName2 - name of second kinect
            \param kinectTopicName2 - topic name for depth part of second kinect
            \param targetFrame - name of target frame for transformation (transformation should be broadcasted on tf!)
        */
        void registerKinects(std::string kinectName1, std::string kinectTopicName1, std::string kinectName2, std::string kinectTopicName2 , std::string targetFrame = "world");

        /* 
            Remove robot from scene. This method removes robot from scene, only if robot model is initialized. 
            \param robot - pointer to robots voxel model 
            \param filter - filter scene after removing robot
            \param - occupancyThreshold - remove all voxels below threshold (occupancyThreshold -> [0, 1])
            \param fillThreshold - remove all voxels that have less neighbours than this threshold (fillThreshold -> [0, 1])
        */
        void removeRobotFromScene(InsertRobotVoxels *robot, bool filter, float occupancyThreshold = 0.75, float fillThreshold = 0.25);

        /* 
            Start subscribing to callbacks (start spinners). Every callback is in its own thread! Non blocking! 
            \return - true if spinners started
        */
        bool startSceneSpinners();

        /* Stop spinners. */
        void stopSceneSpinners();

        /* Returns name of voxel map */
        std::string getSceneVoxelMapName();

        /* Returns shared ptr to scene voxel map */
        ProbVoxelMapPtr getSceneVoxelMap();

        /* Lock thread */
        void lockThread();

        /* Unlock thread */
        void unlockThread();

        /* Visualize scene map. Thread safe! */
        void visualizeScene();

        /*
            Creates transformation matrix from tf buffer. If transformation dosen't exist
            returns identity matrix.
            \param targetFrame - target coordinate frame
            \param currentFrame - current coordinate frame
        */
        Eigen::Affine3d getTransformationMatrix(std::string targetFrame, std::string currentFrame);

        /*
            Transforms Eigen::Affine3d matrix to Gpu-Voxels homogeneous transformation matrix.
            \param - Eigen homogeneous transformation matrix
        */
        static gpu_voxels::Matrix4f eigenTfMatrixToGpuVoxelsTfMatrix(Eigen::Affine3d tfMatrix, Eigen::Vector3f baseOffset = Eigen::Vector3f(0, 0, 0));

        /*
            Deserialization of PointCloud2 sensor message in std::vector<gpu_voxels::Vector3f> 
            (directly insertable in Gpu-Voxels map). Deserialization works for every raw PointCloud 
            which contains points with data type FLOAT32. Returns uncoloured PointCloud (only XYZ, 
            RGB values are ignored).
            \param msg - sensor message
            \param points - converted data
            \return - true if deserialization succeeded 
        */
       static bool deserializationPointCloudVoxels(sensor_msgs::PointCloud2ConstPtr msg, std::vector<gpu_voxels::Vector3f> &points);
       
       /* Updates scene map. If robot removing is ON, this method WILL update robots map and pose! */
       void updateSceneMap();
    
    private:
        /* Create temp voxel maps */
        void createTempVoxelMap();

        /* One kinect callback */
        void kinectCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

        /* Two kinects - kinect1 callback. */
        void kinectCallback1(const sensor_msgs::PointCloud2ConstPtr &msg);

        /* Two kinects - kinect2 callback. */
        void kinectCallback2(const sensor_msgs::PointCloud2ConstPtr &msg);
        
        /* Global and private node handle */
        ros::NodeHandle n, np;

        /* Kinects subscriber (kinectQueue queue) */
        ros::Subscriber sub1, sub2;

        /* Queue for kinects */
        ros::CallbackQueue kinectQueue1, kinectQueue2;

        /* Async spinners */
        ros::AsyncSpinner spinner1, spinner2;

        /* Transform buffer */
        tf2_ros::Buffer tfBuff;

        /* Transform listener */
        tf2_ros::TransformListener tfListener;

        /* Timer uses wall time*/
        TimerWrapper timer; 

        /* Kinects voxel map */
        ProbVoxelMapPtr voxMapTemp, kinectVoxMapMerged;

        /* Pointer on gpu_voxels base object */
        gpu_voxels::GpuVoxels *gvl;

        /* Mutex for multithread synchronization */
        boost::mutex mtx;

        /* Strings */
        std::string targetFrame, oneKinectMapName, oneKinectOutMapName, twoKinectsMapName, twoKinectsOutMapName;
        std::string kinect1Name, kinect2Name, kinect1TopicName, kinect2TopicName;

        /* Origin offset ONLY for Gpu-Voxels */
        Eigen::Vector3f originOffset;

        /* Robots voxel model */
        InsertRobotVoxels *robot;
        
        /* Number of kinects */
        int kinectNum;

        /* Filtering parameters */
        float occupancyThreshold, fillThreshold;

        /* Robot cut and filter */
        bool removeRobot, filter;

        /* Ptrs to voxelized PointClouds*/
        boost::shared_ptr<PointCloud> pc1, pc2, pc;
};
#endif //INSERT_SCENE_VOXELS_H