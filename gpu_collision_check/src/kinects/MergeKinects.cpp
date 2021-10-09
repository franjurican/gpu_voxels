// header for MergePointClouds class
#include <gpu_collision_check/kinects/merge_point_clouds.h>

MergePointClouds::MergePointClouds(ros::NodeHandle n, std::string inTopic1, std::string inTopic2, std::string outTopic,
                                                             std::string frameOut) : tfListener(tfBuff), outFrame(frameOut) 
{

    // subscribe to two pointClouds
    sub1=n.subscribe(inTopic1, 1, &MergePointClouds::pointCloud1, this);
    sub2=n.subscribe(inTopic2, 1, &MergePointClouds::pointCloud2, this);

    // final pointCloud
    pub=n.advertise<sensor_msgs::PointCloud2>(outTopic, 1);
}

void MergePointClouds::pointCloud1(const sensor_msgs::PointCloud2ConstPtr &pointCloud1) 
{
    // measure time
    startTime = ros::Time::now();

    // try to get TF from frame1 to outFrame
    try 
    {
        tfStamped = tfBuff.lookupTransform(outFrame, pointCloud1->header.frame_id, ros::Time(0));
    }
    catch(tf2::TransformException &e) 
    {
        ROS_WARN_STREAM("Couldn't get TF from " << pointCloud1->header.frame_id << " to " << outFrame <<": " << e.what());
        return;
    }
    
    // get tf matrix as Eigen::Affine3d and transform pointCloud1
    tf::transformMsgToEigen(tfStamped.transform, transformMatrix);
    pcl_ros::transformPointCloud(transformMatrix.matrix().cast<float>(), *pointCloud1, pointCloudTf);

    // merge pointCloud1 and pointCloud2
    pcl::fromROSMsg(pointCloudTf, pclPointCloudTf1);
    pclMergedPointCloud = pclPointCloudTf1;
    pclMergedPointCloud += pclPointCloudTf2;
    pcl::toROSMsg(pclMergedPointCloud, mergedPointCloud);

    // set output frame id and time
    mergedPointCloud.header.frame_id = outFrame;
    mergedPointCloud.header.stamp = ros::Time::now();

    // elapsed time
    endTime = ros::Time::now(); 
    ROS_INFO_STREAM("PointCloud1 execution time: " << (endTime - startTime).toNSec()*1e-6 <<" [ms]");

    // publish
    pub.publish(mergedPointCloud); 
}

void MergePointClouds::pointCloud2(const sensor_msgs::PointCloud2ConstPtr &pointCloud2) 
{
    // measure time
    startTime = ros::Time::now();

    // try to get TF from frame2 to outFrame
    try 
    {
        tfStamped = tfBuff.lookupTransform(outFrame, pointCloud2->header.frame_id, ros::Time(0));
    } 
    catch(tf2::TransformException &e) 
    {
        ROS_WARN_STREAM("Couldn't get TF from " << pointCloud2->header.frame_id << " to " << outFrame <<": " << e.what());
        return;
    }

    // get tf matrix as Eigen::Affine3d and transform pointCloud2
    tf::transformMsgToEigen(tfStamped.transform, transformMatrix);
    pcl_ros::transformPointCloud(transformMatrix.matrix().cast<float>(), *pointCloud2, pointCloudTf);

    // merge pointCloud2 and pointCloud1
    pcl::fromROSMsg(pointCloudTf, pclPointCloudTf2);
    pclMergedPointCloud = pclPointCloudTf2;
    pclMergedPointCloud += pclPointCloudTf1;
    pcl::toROSMsg(pclMergedPointCloud, mergedPointCloud);

    // set output frame id and time
    mergedPointCloud.header.frame_id = outFrame;
    mergedPointCloud.header.stamp = ros::Time::now();
 
    // elapsed time
    endTime = ros::Time::now();
    ROS_INFO_STREAM("PointCloud2 execution time: " << (endTime - startTime).toNSec()*1e-6 <<" [ms]");

    // publish
    pub.publish(mergedPointCloud);
}

void MergePointClouds::start() 
{
    ros::spin();
}

MergePointClouds::~MergePointClouds() {}