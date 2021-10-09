#include <iostream>
#include <string>

#include <pcl_model/get_pointcloud_kinect.h>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_pointcloud_kinect");

    // folder s .pcd file-ovima nalazi se u config folderu u ros paketu pcl_model
    std::string packagePath = ros::package::getPath("pcl_model");
    
    GetPointCloudKinect getCloud(packagePath + "/config/");
    
    ROS_INFO_STREAM("Spreman na snimanje .pcd file-a ...");
    getCloud.start();
    ros::spin();
    getCloud.stop();

    return 0;
}