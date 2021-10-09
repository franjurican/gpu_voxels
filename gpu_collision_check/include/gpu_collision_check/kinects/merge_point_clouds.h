#ifndef MERGE_POINT_CLOUDS_H
#define MERGE_POINT_CLOUDS_H

// C++ stl
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MergePointClouds 
{
    public:
        MergePointClouds(ros::NodeHandle n, std::string inTopic1, std::string inTopic2,  std::string outTopic, 
                                                                                            std::string frameOut="world");
        ~MergePointClouds();
        void start();
    
    private:
        std::string outFrame;
        ros::Publisher pub;
        ros::Subscriber sub1, sub2;
        ros::Time startTime, endTime;
        tf2_ros::Buffer tfBuff;
        tf2_ros::TransformListener tfListener;
        pcl::PointCloud<pcl::PointXYZ> pclMergedPointCloud, pclPointCloudTf1, pclPointCloudTf2;
        Eigen::Affine3d transformMatrix;
        geometry_msgs::TransformStamped tfStamped;
        sensor_msgs::PointCloud2 mergedPointCloud, pointCloudTf;

        void pointCloud1(const sensor_msgs::PointCloud2ConstPtr &pointCloud1);
        void pointCloud2(const sensor_msgs::PointCloud2ConstPtr &pointCloud2);       
};
#endif //MERGE_POINT_CLOUDS_H