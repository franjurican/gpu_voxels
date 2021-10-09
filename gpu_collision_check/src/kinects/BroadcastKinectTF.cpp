// C++ stt
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// ROS msgs
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv){
    // init ros
    ros::init(argc, argv, "kinects_tf_broadcaster");
    ros::NodeHandle n("~");

    // broadcaster and transform
    tf2_ros::TransformBroadcaster tfBr;
    tf2::Quaternion q;
    geometry_msgs::TransformStamped tfStamped;
    std::vector<double> translacija_camera1(3), translacija_camera2(3), rotacija_camera1(3), rotacija_camera2(3);
    std::string camera1FrameName, camera2FrameName, targetFrameName;
    ros::Rate loopRate(30);

    while (ros::ok()) {
        // get param
        if(!n.getParam("camera1/translation", translacija_camera1))
        {   
            ROS_WARN_STREAM("No translation parametar for camera1!");
            ROS_WARN_STREAM("Setting translation of camera1: [0, 0, 0]\n");
            translacija_camera1[0] = 0;
            translacija_camera1[1] = 0;
            translacija_camera1[2] = 0;
            n.setParam("camera1/translation", translacija_camera1);
        }
            
        if(!n.getParam("camera1/rotation", rotacija_camera1))
        {
            ROS_WARN_STREAM("No rotation parametar for camera1!");
            ROS_WARN_STREAM("Setting rotation of camera1: [0, 0, 0]\n");
            rotacija_camera1[0] = 0;
            rotacija_camera1[1] = 0;
            rotacija_camera1[2] = 0;
            n.setParam("camera1/rotation", rotacija_camera1);
        }

        if(!n.getParam("camera1/frameName", camera1FrameName))
        {
            ROS_WARN_STREAM("No frame name for camera1!");
            ROS_WARN_STREAM("Setting frame name for camera1: kinect1\n");
            camera1FrameName = "kinect1";
            n.setParam("camera1/frameName", camera1FrameName);
        }

        if(!n.getParam("camera2/translation", translacija_camera2))
        {
            ROS_WARN_STREAM("No translation parametar for camera2!");
            ROS_WARN_STREAM("Setting translation of camera2: [0, 0, 0]\n");
            translacija_camera2[0] = 0;
            translacija_camera2[1] = 0;
            translacija_camera2[2] = 0; 
            n.setParam("camera2/translation", translacija_camera2);
        }
        
        if(!n.getParam("camera2/rotation", rotacija_camera2))
        {
            ROS_WARN_STREAM("No rotation parametar for camera2!");
            ROS_WARN_STREAM("Setting rotation of camera2: [0, 0, 0]\n");
            rotacija_camera2[0] = 0;
            rotacija_camera2[1] = 0;
            rotacija_camera2[2] = 0;
            n.setParam("camera2/rotation", rotacija_camera2);
        }

        if(!n.getParam("camera2/frameName", camera2FrameName))
        {
            ROS_WARN_STREAM("No frame name for camera2!");
            ROS_WARN_STREAM("Setting frame name for camera2: kinect2\n");
            camera2FrameName = "kinect2";
            n.setParam("camera2/frameName", camera2FrameName);
        }

        if(!n.getParam("targetFrameName", targetFrameName))
        {
            ROS_WARN_STREAM("No target frame name!");
            ROS_WARN_STREAM("Setting target frame name: world\n");
            targetFrameName = "world";
            n.setParam("targetFrameName", targetFrameName);
        }

        // broadcast transform for kinect1
        tfStamped.header.stamp = ros::Time::now();
        tfStamped.header.frame_id = targetFrameName;
        tfStamped.child_frame_id = camera1FrameName;
        tfStamped.transform.translation.x = translacija_camera1[0];
        tfStamped.transform.translation.y = translacija_camera1[1];
        tfStamped.transform.translation.z = translacija_camera1[2];
        q.setRPY(rotacija_camera1[0], rotacija_camera1[1], rotacija_camera1[2]);
        tfStamped.transform.rotation.x = q.x();
        tfStamped.transform.rotation.y = q.y();
        tfStamped.transform.rotation.z = q.z();
        tfStamped.transform.rotation.w = q.w();
        tfBr.sendTransform(tfStamped);

        // broadcast transform for kinect2
        tfStamped.header.stamp = ros::Time::now();
        tfStamped.header.frame_id = targetFrameName;
        tfStamped.child_frame_id = camera2FrameName;
        tfStamped.transform.translation.x = translacija_camera2[0];
        tfStamped.transform.translation.y = translacija_camera2[1];
        tfStamped.transform.translation.z = translacija_camera2[2];
        q.setRPY(rotacija_camera2[0], rotacija_camera2[1], rotacija_camera2[2]);
        tfStamped.transform.rotation.x = q.x();
        tfStamped.transform.rotation.y = q.y();
        tfStamped.transform.rotation.z = q.z();
        tfStamped.transform.rotation.w = q.w();
        tfBr.sendTransform(tfStamped);

        // wait
        loopRate.sleep(); 
    }

    // clear params
    n.deleteParam("camera1/translation");
    n.deleteParam("camera1/rotation");
    n.deleteParam("camera1/frameName");
    n.deleteParam("camera2/translation");
    n.deleteParam("camera2/rotation");
    n.deleteParam("camera2/frameName");
    n.deleteParam("targetFrameName");

    return 0;
}