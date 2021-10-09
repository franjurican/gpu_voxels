// C++ stl
#include <signal.h>

// headers for CPU and GPU based voxelization
#include <gpu_collision_check/gpu_collision/insert_robot_voxels.h>
#include <gpu_collision_check/gpu_collision/insert_scene_voxels.h>
#include <gpu_collision_check/gpu_collision/collision_algorithms_voxels.h>
#include <helpers_gpu_voxels/timer/timer_wrapper.h>

// GPU-Voxels logging
#include <gpu_voxels/logging/logging_gpu_voxels.h>

// handlers for signals
void ctrlchandler(int)
{
  ros::shutdown();
}

void killhandler(int)
{
  ros::shutdown();
}

int main(int argc, char *argv[])
{
    // exit signals
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    // init ros, icl_core and get GpuVoxels instance
    ros::init(argc, argv, "testRobotVoxels");
    icl_core::logging::initialize(argc, argv);
    gpu_voxels::GpuVoxelsSharedPtr gvl = gpu_voxels::GpuVoxels::getInstance();

    // map size in VOXELS and voxel size (map size iz 3x3x3 [m])
    Eigen::Vector3i mapSize(150, 150, 150);
    float voxelSize = 0.02;

    // create robot model for gpu voxels
    InsertRobotVoxels robot(gvl.get(), mapSize, voxelSize);
    robot.initializeRobot("kuka", "kuka/kuka_2cm/kuka_kr10_tool_voxels.urdf", true, Eigen::Vector3f(1.5, 1.5, 0));
    robot.initializeSwept(InsertRobotVoxels::SweptType::ST_MAP);
    robot.initializeRobotBB("kuka/kuka_2cm/kuka_kr10_tool_voxels_BB5.urdf");
    robot.startSubscribing();

    // create scene model for gpu voxels
    InsertSceneVoxels scene(gvl.get(), mapSize, voxelSize, Eigen::Vector3f(1.5 - 0.03, 1.5 - 0.05, 0 + 0.06));
    // scene.registerKinects("kinect1", "kinect1/depth/points", "kinect2", "kinect2/depth/points"); // simulation
    // scene.registerKinects("kinect1", "kinect1/depth/points", "asus", "asus/depth/points"); // real robot
    scene.registerKinect("asus", "asus/depth/points", "base_link");

    // create algorithms object
    CollisionAlgorithmsVoxels algorithms(gvl.get(), &robot, &scene);
    // algorithms.connect2ActionServer("/joint_trajectory_action"); // simulation
    algorithms.connect2ActionServer("/position_trajectory_controller/follow_joint_trajectory"); // real robot

    // start loop
    ROS_ERROR("Pokrecem loop!!!!!!!!!!");
    // algorithms.startLoopRobot(10);
    // algorithms.startLoopScene(10, false, false); 126
    algorithms.collisionCheckPredictionHorizon(false, 125, 1);

    // explicitly call all destructors of objects with automatic storage duration, clear GPU shared memory and exit
    algorithms.~CollisionAlgorithmsVoxels();
    scene.~InsertSceneVoxels();
    robot.~InsertRobotVoxels();
    gvl.reset();
    exit(EXIT_SUCCESS);
}