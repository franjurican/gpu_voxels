// header for MergePointClouds class
#include <gpu_collision_check/kinects/merge_point_clouds.h>

int main(int argc, char *argv[]) {
    // init ros
    ros::init(argc, argv, "mergeKinects");
    ros::NodeHandle n;
    std::string topic1, topic2, outTopic;

    // topics
    topic1="/kinect1/depth/points";
    topic2="/kinect2/depth/points";
    outTopic="/merged_kinects/depth/points";

    MergePointClouds m(n, topic1, topic2, outTopic);
    m.start();

    return 0;
}