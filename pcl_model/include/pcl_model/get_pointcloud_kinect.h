#ifndef GET_POINTCLOUD_KINECT_H
#define GET_POINTCLOUD_KINECT_H

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

class GetPointCloudKinect {
    public:
        /* 
            Konstruktor za klasu koja prikuplja PointCloud s Kinect-a i trenutacni joint_states za uzeti PointCloud.
            \param savePath folder u koji se spremlja .pcd file
            \param kinectTopic topic na koji kamera salje dubinsku 3D sliku
            \param jointTopic topic sa vrijednostima joint-a robota
        */
        GetPointCloudKinect(std::string savePath, std::string kinectTopic = "/kinect1/depth/points", std::string jointTopic = "/joint_states");

        /* Pokreni skupljanje podataka (svaki callback u svom thread-u!) - spinneri. */
        void start();

        /* Zaustavi spinner-e */
        void stop();

    private:
        ros::NodeHandle np;
        std::string savePath;
        ros::Subscriber subKinect, subJoint;
        ros::CallbackQueue queueKinect, queueJointState;
        ros::AsyncSpinner spinnerKinect, spinnerJointState;
        boost::mutex mtx;
        sensor_msgs::JointState saveJointStatus;

        void printJointStates();
        void kinectCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud);
        void jointStatesCallback(const sensor_msgs::JointStateConstPtr &jointStates);

};

#endif // GET_POINTCLOUD_KINECT_H