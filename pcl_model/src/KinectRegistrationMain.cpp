#include <iostream>
#include <string>

#include <pcl_model/kinect_registration.h>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "kinect_registration");

    // folder s .pcd file-ovima nalazi se u config folderu u ros paketu pcl_model
    std::string packagePath = ros::package::getPath("pcl_model");

    KinectRegistration reg("kameraPointCloud.pcd", "robotModel.pcd", packagePath + "/config/kinect/");
    reg.setDistanceToRobot(2); // cca. 2 metra za obje kamere
    reg.setMaxIterationsICP(1000);
    reg.setMaxRANSACIterationsICP(2000);
    reg.startRegistration(true);
    reg.printKinectTransformationInfo();
    reg.printBenchmarkInfo();

    return 0;
}