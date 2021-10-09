// C++ stl
#include <iostream>
#include <signal.h>

// voxelization
#include <gpu_collision_check/gpu_collision/insert_scene_voxels.h>
#include <helpers_gpu_voxels/bb_cuda.h>
#include <helpers_gpu_voxels/path_planner_voxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>

// ROS
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// Eigen
#include <Eigen/Geometry>

// TOPP-RA
#include <topp_ros/GenerateTrajectory.h>

// handlers for signals
void ctrlchandler(int) { ros::shutdown();}
void killhandler(int) { ros::shutdown();}

// namesapce
namespace rvt = rviz_visual_tools;

int main(int argc, char *argv[]) 
{
    // exit signals
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    // map size (voxels), voxel size and MoveIt planning group name
    Eigen::Vector3i mapSize(150, 150, 150);
    float voxelSize = 0.02;
    float scalingV = 0.02*6, scalingA = 0.05*6;
    // const std::string PG = "kuka_arm";
    const std::string PG = "kuka_kr10r1100sixx";

    // init ros, icl_core and gpu_voxels
    ros::init(argc, argv, "path_planner_voxels");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    icl_core::logging::initialize(argc, argv);
    gpu_voxels::GpuVoxelsSharedPtr gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(mapSize.x(), mapSize.y(), mapSize.z(), voxelSize);

    /////////////////////////////////////////
    // conect to TOPP-RA and action server //
    /////////////////////////////////////////
    ros::ServiceClient toppRA = nh.serviceClient<topp_ros::GenerateTrajectory>("/generate_toppra_trajectory");
    if(toppRA.waitForExistence(ros::Duration(2)))
        ROS_INFO("Spojen na service: /generate_toppra_trajectory");
    else
        ROS_ERROR("Nisam se uspio spojiti na service: /generate_toppra_trajectory");

    // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> actionServer("/position_trajectory_controller/follow_joint_trajectory");
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> actionServer("/joint_trajectory_action");
    if(actionServer.waitForServer(ros::Duration(2.0)))
        ROS_INFO("Spojen na action server: /position_trajectory_controller/follow_joint_trajectory");
    else
        ROS_ERROR("Nisam se uspio spojiti na action server: /position_trajectory_controller/follow_joint_trajectory");

    // trajectory cmd publisher -- test 
    // ros::Publisher pubCmd = nh.advertise<trajectory_msgs::JointTrajectory>("/position_trajectory_controller/command", 10);
    ros::Publisher pubCmd = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 10);

    /////////////////////////////////////////////////////////////
    // create scene model for gpu voxels and get current scene //
    /////////////////////////////////////////////////////////////
    InsertSceneVoxels scene(gvl.get(), mapSize, voxelSize, Eigen::Vector3f(1.5 - 0.03, 1.5 - 0.05, 0.1));
    scene.registerKinects("kinect1", "kinect1/depth/points", "asus", "asus/depth/points");
    scene.startSceneSpinners();
    ROS_WARN("Cekam 1 sekundu prije pocetka, kako bi dobio prve PointCloud-ove ...");
    ros::Duration(1).sleep();
    scene.updateSceneMap();
    InsertSceneVoxels::ProbVoxelMapPtr sceneMap = scene.getSceneVoxelMap();
    scene.stopSceneSpinners(); 
    scene.visualizeScene();
    std::getchar(); 

    /*    
    // ------------- get bb for testing (simualtion) ---------------------------
    std::vector<gpu_voxels::Vector3f> box;
    InsertSceneVoxels::ProbVoxelMapPtr mapaProb;
    gvl->addMap(MT_PROBAB_VOXELMAP, "mapaProb");
    mapaProb = boost::dynamic_pointer_cast<gpu_voxels::voxelmap::ProbVoxelMap>(gvl->getMap("mapaProb"));
    box = gpu_voxels::geometry_generation::createBoxOfPoints(Vector3f(0, 0, 0), Vector3f(0.25, 0.1, 0.2), 0.02);
    
    //rotation and translation of bb
    Matrix3f rot = Matrix3f::createFromRPY(0, 0, 0);
    Vector3f trans(1.9, 1.5, 0.02);
    Matrix4f transform = Matrix4f::createFromRotationAndTranslation(rot, trans);
    PointCloud pc;

    pc.update(box);
    pc.transformSelf(&transform);
    mapaProb->insertPointCloud(pc, eBVM_OCCUPIED);
    gvl->visualizeMap("mapaProb"); 
    // --------------------------------------------------------------------------
    */
    //////////////////////////////////////////
    // get objects VoxelCloud and waypoints //
    //////////////////////////////////////////
    bb_cuda::VoxelCloudPtr voxelCloud;
    std::vector<Eigen::Vector3f> waypoints;
    Eigen::Affine3f transformMatrix;
    // ZA SIMULACIJU koristiti mapaProb, za STVARNI SUSTAV sceneMap
    // y-os nije dobro postavljena, odgovara -y osi
    voxelCloud = bb_cuda::getObjectVoxelCloud(sceneMap, Eigen::Vector3f(0.15, -0.5, 0.1), Eigen::Vector3f(1.2, 0.5, 0.55), 
        Eigen::Vector3f(1.5, 1.5, 0), Eigen::Vector3f(0.0, 0.0, 0.0));

    std::getchar();

    ROS_INFO_STREAM("Generirano waypoints-a: " << voxelCloud->size());

    waypoints = path_planner_voxels::generateAndVisualizeWaypoints(voxelCloud, transformMatrix, 0.1);

    /*ROS_INFO_STREAM("Generirano waypoints-a: " << waypoints.size());

    for(int i = 0; i < waypoints.size(); i++)
        std::cout << "Tocka " << i + 1 << ": " << waypoints[i].transpose() << std::endl;

    std::cout << "Matrica rotacije: \n" << transformMatrix.rotation() << std::endl;*/

    ////////////////////////
    // movit path planing //
    ////////////////////////
    ROS_INFO("Pokrecem planiranje putanje u MoveIt-u!");

    // moveit planning group
    moveit::planning_interface::MoveGroupInterface move_group(PG);
    const robot_state::JointModelGroup *jointGroup = move_group.getCurrentState()->getJointModelGroup(PG);
    move_group.setNumPlanningAttempts(20);
    move_group.setPlanningTime(10);
    move_group.setPlannerId("RRTConnect");
    ROS_WARN_STREAM("Planer ID:" << move_group.getPlannerId());

    // start state for planning
    robot_state::RobotState state(*move_group.getCurrentState());
    move_group.setStartState(state);

    // info
    ROS_INFO_STREAM("Reference frame: " << move_group.getPlanningFrame());
    ROS_INFO_STREAM("End effector link: " << move_group.getEndEffectorLink());
    ROS_INFO_STREAM("End effector: " << move_group.getEndEffector());

    ///////////////
    // rotations //
    ///////////////
    // rotation matrix for first side
    Eigen::Affine3f transform_side1 = Eigen::Affine3f::Identity();

    // roll, pitch, yaw fixed frame!
    transform_side1.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
    transform_side1.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()));
    transform_side1.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));

    transform_side1.rotate(Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()));
    transform_side1.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()));
    transform_side1.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
    
    // rotate objects coordinate frame
    transform_side1 = transformMatrix * transform_side1;
    std::cout << "Matrica rotacije za prvu stranicu: \n" << transform_side1.rotation() << std::endl;
    //-----------------------------------------------------------------------------------------------

    // rotation matrix for second side
    Eigen::Affine3f transform_side2 = Eigen::Affine3f::Identity();

    // roll, pitch, yaw fixed frame!
    transform_side2.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
    transform_side2.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()));
    transform_side2.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
    
    // rotate objects coordinate frame
    transform_side2 = transformMatrix * transform_side2;
    std::cout << "Matrica rotacije za drugu stranicu: \n" << transform_side2.rotation() << std::endl;
    //-----------------------------------------------------------------------------------------------

    // rotation matrix for third side
    Eigen::Affine3f transform_side3 = Eigen::Affine3f::Identity();

    // roll, pitch, yaw fixed frame!
    transform_side3.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
    transform_side3.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()));
    transform_side3.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));

    transform_side3.rotate(Eigen::AngleAxisf(-M_PI_4, Eigen::Vector3f::UnitZ()));
    transform_side3.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()));
    transform_side3.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
    
    // rotate objects coordinate frame
    transform_side3 = transformMatrix * transform_side3;
    std::cout << "Matrica rotacije za trecu stranicu: \n" << transform_side3.rotation() << std::endl;
    //-----------------------------------------------------------------------------------------------

    /* geometry_msgs::PoseStamped poseEnd;
    Eigen::Quaternionf q2(transform_side1.rotation());
    q2.normalize();
    poseEnd.header.frame_id = "base";
    poseEnd.pose.orientation.w = q2.w();
    poseEnd.pose.orientation.x = q2.x();
    poseEnd.pose.orientation.y = q2.y();
    poseEnd.pose.orientation.z = q2.z();
    poseEnd.pose.position.x = waypoints[4].x();
    poseEnd.pose.position.y = waypoints[4].y();
    poseEnd.pose.position.z = waypoints[4].z();
    move_group.setPoseTarget(poseEnd);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    if(move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS) 
    {
        ROS_INFO_STREAM("Broj tocaka putanje: " << plan1.trajectory_.joint_trajectory.points.size());
        ROS_INFO("Planning SUCCEEDED! Visualizing plan as trajectory ...");
    }
    else
    {
        ROS_INFO("Planning FAILED!");
    }*/
    
    /////////////////////////////////
    // create waypoints for moveit //
    /////////////////////////////////
    geometry_msgs::Pose poseMsg;
    Eigen::Quaternionf q;
    int n = waypoints.size();
    std::vector<geometry_msgs::Pose> waypointsMoveit(n + 2);

    // first point -> current pose
    waypointsMoveit[0] = move_group.getCurrentPose().pose;

    for(int i = 0; i < n/3; i++)
    {
        // first side    
        poseMsg.position.x = waypoints[i].x();
        poseMsg.position.y = waypoints[i].y(); 
        poseMsg.position.z = waypoints[i].z();

        q = transform_side1.rotation();
        q.normalize();

        poseMsg.orientation.x = q.x();
        poseMsg.orientation.y = q.y();
        poseMsg.orientation.z = q.z();
        poseMsg.orientation.w = q.w();

        waypointsMoveit[i + 1] = poseMsg;

        // second side
        poseMsg.position.x = waypoints[i + n/3].x();
        poseMsg.position.y = waypoints[i + n/3].y(); 
        poseMsg.position.z = waypoints[i + n/3].z();

        q = transform_side2.rotation();
        q.normalize();

        poseMsg.orientation.x = q.x();
        poseMsg.orientation.y = q.y();
        poseMsg.orientation.z = q.z();
        poseMsg.orientation.w = q.w();

        waypointsMoveit[i + n/3 + 1] = poseMsg;

        // third side
        poseMsg.position.x = waypoints[i + 2*n/3].x();
        poseMsg.position.y = waypoints[i + 2*n/3].y(); 
        poseMsg.position.z = waypoints[i + 2*n/3].z();

        q = transform_side3.rotation();
        q.normalize();

        poseMsg.orientation.x = q.x();
        poseMsg.orientation.y = q.y();
        poseMsg.orientation.z = q.z();
        poseMsg.orientation.w = q.w();

        waypointsMoveit[i + 2*n/3 + 1] = poseMsg;
    }

    // final pose
    poseMsg.position.x = waypoints[n/3].x();
    poseMsg.position.y = waypoints[n/3].y();
    poseMsg.position.z = waypoints[n/3].z();

    q = transform_side2.rotation();
    q.normalize();

    poseMsg.orientation.x = q.x();
    poseMsg.orientation.y = q.y();
    poseMsg.orientation.z = q.z();
    poseMsg.orientation.w = q.w();

    waypointsMoveit[n+1] = poseMsg;
    
    /////////////////////////////
    // plan path for waypoints //
    /////////////////////////////
    moveit_msgs::RobotTrajectory trajectory;
    const double jump = 0;
    const double eef_step = 0.02;

    double fraction = move_group.computeCartesianPath(waypointsMoveit, eef_step, jump, trajectory);

    if (fraction > 0.95)
    {
        ROS_INFO_STREAM("Uspjesnost planiranja: " << fraction*100.0);
        ROS_INFO("Pokrecem TOPP-RA ...");

        // max. velocities and accelerations for joints
        std::vector<double> maxV{3.83*scalingV, 3.66*scalingV, 4.71*scalingV, 6.65*scalingV, 5.43*scalingV, 8.59*scalingV};
        std::vector<double> maxA{10.0*scalingA, 3.0*scalingA, 10.0*scalingA, 10.0*scalingA, 10.0*scalingA, 10.0*scalingA};

        // fill trajectory msg for TOPP-RA
        for(int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
        {   
            trajectory.joint_trajectory.points[i].velocities = maxV;
            trajectory.joint_trajectory.points[i].accelerations = maxA;
        }

        // call TOPP-RA
        topp_ros::GenerateTrajectoryRequest topp_req;
        topp_ros::GenerateTrajectoryResponse topp_res;

        topp_req.waypoints = trajectory.joint_trajectory;
        topp_req.sampling_frequency = 100;
        topp_req.plot = false;

        if(toppRA.call(topp_req, topp_res))
        {
            ROS_INFO("TOPP RA: trajektorija isplanirana uspjesno!");

            // put header and joint names
            topp_res.trajectory.header.stamp = ros::Time::now();
            topp_res.trajectory.joint_names = trajectory.joint_trajectory.joint_names;

            // topic cmd
            // pubCmd.publish(topp_res.trajectory);
            // std::getchar();

            // action server 
            ROS_INFO("Saljem trajektoriju na action server ...");
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = topp_res.trajectory;
            actionServer.sendGoalAndWait(goal);
            ROS_INFO("Trajektorija izvrsena. Izlazim iz planiranja putanje ...");
        }
        else
        {
            ROS_ERROR("TOPP RA: pogreska prilikom planiranja trajektorije");
        }
    }
    else
    {
        ROS_ERROR_STREAM("NE pokrecem izvrsavanje trajektorije. Uspjesnost planiranja: " << fraction*100 << " %");
    }

    //scene.~InsertSceneVoxels();
    gvl.reset();
    exit(EXIT_SUCCESS);
}