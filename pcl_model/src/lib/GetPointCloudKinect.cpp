#include <pcl_model/get_pointcloud_kinect.h>

GetPointCloudKinect::GetPointCloudKinect(std::string savePath, std::string kinectTopic, std::string jointTopic) : 
    np("~"), savePath(savePath), spinnerKinect(1, &queueKinect), spinnerJointState(1, &queueJointState)
{
    // kinect subscriber
    this->np.setCallbackQueue(&(this->queueKinect));
    this->subKinect = this->np.subscribe(kinectTopic, 1, &GetPointCloudKinect::kinectCallback, this);

    // joint states subscriber
    this->np.setCallbackQueue(&(this->queueJointState));
    this->subJoint = this->np.subscribe(jointTopic, 10, &GetPointCloudKinect::jointStatesCallback, this);
    this->np.setCallbackQueue(NULL);

    // save pointcloud from kinect
    np.setParam("savePointCloud", false);
}

void GetPointCloudKinect::start()
{
    this->spinnerKinect.start();
    this->spinnerJointState.start();
}

void GetPointCloudKinect::stop()
{
    this->spinnerKinect.stop();
    this->spinnerJointState.stop();
}

void GetPointCloudKinect::printJointStates()
{
    boost::lock_guard<boost::mutex> lock(this->mtx);

    ROS_INFO("Pozicija joint-a za spremljeni PointCloud: ");
    for(int i = 0; i < this->saveJointStatus.name.size(); i++)
        ROS_INFO_STREAM(this->saveJointStatus.name[i] << ": " << this->saveJointStatus.position[i]);
    
    std::cout << std::endl;
}

void GetPointCloudKinect::kinectCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud)
{   
    bool pom;
    pcl::PointCloud<pcl::PointXYZ> pomPC;

    if(this->np.getParam("savePointCloud", pom) && (pom != false))
    {   
        ROS_INFO("Spremam PointCloud s 'kinect-a' u .pcd file!");
        pcl::fromROSMsg(*pointcloud, pomPC);
        pcl::io::savePCDFileBinary(this->savePath + "kinectPointCloud.pcd", pomPC);

        ROS_INFO_STREAM("PCD file: " + this->savePath + "kinectPointCloud.pcd");
        this->printJointStates();
        this->np.setParam("savePointCloud", false);
    }
}

void  GetPointCloudKinect::jointStatesCallback(const sensor_msgs::JointStateConstPtr &jointStates)
{
    boost::lock_guard<boost::mutex> lock(this->mtx);
    this->saveJointStatus = *jointStates;
}