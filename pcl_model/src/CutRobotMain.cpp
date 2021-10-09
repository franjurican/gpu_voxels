#include <iostream>
#include <string>

#include <pcl_model/cut_robot_out.h>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "cut_robot");

    // folder s .pcd file-ovima nalazi se u config folderu u ros paketu pcl_model
    std::string packagePath = ros::package::getPath("pcl_model");

    CutRobotOut cutRobot(packagePath + "/config/asus/", "kameraPointCloud.pcd");
    cutRobot.start();
    
    return 0;
}