// header for CollisionAlgorithms class
#include <gpu_collision_check/gpu_collision/collision_algorithms_voxels.h>
#include <helpers_gpu_voxels/bb_cuda.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <topp_ros/GenerateTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

// macro functions
#define name(x) #x
#define RETURN_WITH_MSG(msg, alg) {ROS_WARN_STREAM(msg << ", izlazim iz algoritma: " << name(alg)); return;}

using namespace gpu_voxels;

CollisionAlgorithmsVoxels::CollisionAlgorithmsVoxels(gpu_voxels::GpuVoxels *gvl, InsertRobotVoxels *robot, InsertSceneVoxels *scene) 
                                    : n(), np("~"), robot(robot), scene(scene), timer("algoritmi"), gvl(gvl), actionServerConnected(false)
{
    // check for nullptr (NULL)
    if((this->robot == nullptr) && (this->scene == nullptr))
        this->checkNULL = RobotSceneNULL;
    else if(this->robot == nullptr)
        this->checkNULL = RobotNULL;
    else if(this->scene == nullptr)
        this->checkNULL = SceneNULL;
    else
        this->checkNULL = RobotSceneOK;

    // conect to TOPP-RA service
    toppRA = n.serviceClient<topp_ros::GenerateTrajectory>("/generate_toppra_trajectory");

    ROS_WARN("Cekam (max. 2 sec.) TOPP RA ... ");

    if(toppRA.waitForExistence(ros::Duration(2)))
        ROS_INFO("Spojen na service: /generate_toppra_trajectory");
    else
        ROS_ERROR("Nisam se uspio spojiti na service: /generate_toppra_trajectory");

    // trajectory cmd publisher
    this->pubCmd = n.advertise<trajectory_msgs::JointTrajectory>("/position_trajectory_controller/command", 10);
}

CollisionAlgorithmsVoxels::~CollisionAlgorithmsVoxels() {}

std::ostream &operator<<(std::ostream &output, const CollisionAlgorithmsVoxels &obj)
{
    // info
   ROS_INFO_STREAM("Uspjesno kreirano sucelje za algoritme: " << name(CollisionAlgorithmsVoxels));

    // check for nullptr
    if(obj.checkNULL == CollisionAlgorithmsVoxels::RobotNULL)
    {
        ROS_INFO_STREAM("Pointer \"robot\" je nullptr!"); 
        ROS_INFO_STREAM("Moguce je samo koristiti: " << name(CollisionAlgorithmsVoxels::startLoopScene) << 
                                                                          ", ali ce izbacivanje robota biti ISKLJUCENO!");
    }
    else if(obj.checkNULL == CollisionAlgorithmsVoxels::SceneNULL)
    {
        ROS_INFO_STREAM("Pointer \"scene\" je nullptr!");
        ROS_INFO_STREAM("Moguce je samo koristiti: " << name(CollisionAlgorithmsVoxels::startLoopRobot));
    }
    else if(obj.checkNULL == CollisionAlgorithmsVoxels::RobotSceneNULL)
    {
        ROS_INFO_STREAM("Pointeri \"robot\" i \"scene\" su nullptr-i!");
        ROS_INFO_STREAM("Nije moguce koristiti niti jedan algoritam detekcije kolizije!");
    }
    else
    {
        ROS_INFO_STREAM("Dostupni algoritmi detekcije kolizije:");
        ROS_INFO_STREAM(name(CollisionAlgorithmsVoxels::startLoopRobot) << " - voxelizirani model robota");
        ROS_INFO_STREAM(name(CollisionAlgorithmsVoxels::startLoopScene) << " - voxelizirana scena");
        ROS_INFO_STREAM(name(CollisionAlgorithmsVoxels::collisionCheckPredictionHorizon) << " - predikcijski horizont");
        ROS_INFO_STREAM(name(CollisionAlgorithmsVoxels::collisionCheckSafeZones(ZT_BASE_LINK)) << " - sigurne zone oko baze robota");
        ROS_INFO_STREAM(name(CollisionAlgorithmsVoxels::collisionCheckSafeZones(ZT_ROBOT_MODEL)) << " - sigurne zone oko modela robota");
    }

    return output;
}

void CollisionAlgorithmsVoxels::connect2ActionServer(std::string name)
{
    if(this->actionServerConnected)
    {
        ROS_WARN("Vec sam spojen na action server. NEMA ponovnog spajanja!");
        return;
    }

    // connect to action server
    this->actionServerName = name;
    this->actionServer.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(name, true));

    // wait for action server (max. 2 seconds)
    ROS_INFO_STREAM("Cekam na action server (max. 2 sec) ... ");
    if(this->actionServer->waitForServer(ros::Duration(2.0)))
    {
        this->actionServerConnected = true;
        ROS_INFO_STREAM("Spojen na action server: " << this->actionServerName);
    }
    else
        ROS_WARN_STREAM("Nisam se uspio spojiti na action server: " << this->actionServerName);
}

void CollisionAlgorithmsVoxels::startLoopRobot(float hz, bool visualize)
{   
    if((this->checkNULL == RobotNULL) || (this->checkNULL == RobotSceneNULL))
        RETURN_WITH_MSG("Pointer \"robot\" je nullptr (pogledaj info - operator<<)", CollisionAlgorithmsVoxels::startLoopRobot)
    
    // loop rate and prediction horizon info
    ros::Rate loopRate(hz);
    InsertRobotVoxels::PredictionHorizonStruct predH = robot->getPredictionHorizonInfo();
    int k = 0;

    // loop with target rate (hz)
    while(ros::ok())
    {
        this->timer.tic();
        // update robot pose and map
        robot->updateRobotMap();
        
        // create swept if predictionHorizon.status = true
        if(predH.status)
            robot->updateSweptMap();
        
        // visualize?
        if(visualize)
        {
            robot->visualizeRobot();

            // swept?
            if(predH.status)
                robot->visualizeSwept();   
        }

        // get execution time every 50/hz seconds
        if(k == 50)
        {
            this->timer.tocMili("Vrijeme obrade robota");
            k = 0;
        }

        k++;
        loopRate.sleep();
    }
} 

void CollisionAlgorithmsVoxels::startLoopScene(float hz, bool removeRobot, bool filter,  bool visualize)
{   
    // check ptrs  
    if((this->checkNULL == SceneNULL) || (this->checkNULL == RobotSceneNULL))
        RETURN_WITH_MSG("Pointer \"scene\" je nullptr (pogledaj info - operator<<)", CollisionAlgorithmsVoxels::startLoopScene)
    else if(this->checkNULL == RobotNULL)
    {
        ROS_WARN_STREAM("Pointer \"robot\" je nullptr (pogledaj info - operator<<), iskljucujem izbacivanje robota iz scene!");
        removeRobot = false;
        filter = false;
    }

    // "start" scene
    if(!this->scene->startSceneSpinners())
        RETURN_WITH_MSG("Spinner/i nisu pokrenuti", CollisionAlgorithmsVoxels::startLoopScene)

    // loop rate and counter
    ros::Rate loopRate(hz);
    int k = 0;

    // wait for first PointClouds
    ROS_WARN("Cekam 1 sekundu prije pocetka, kako bi dobio prve PointCloud-ove ...");
    ros::Duration(1).sleep();
    
    // remove robot?
    if(removeRobot)
        this->scene->removeRobotFromScene(this->robot, filter);

    // loop until ros::ok() = true
    while(ros::ok()) 
    {   
        this->timer.tic();
        // update scene map
        scene->updateSceneMap();

        // visualize
        if(visualize)
            scene->visualizeScene();

        // get execution time every 50/hz seconds
        if(k == 50)
        {
            this->timer.tocMili("Vrijeme obrade scene");
            k = 0;
        }

        k++;     
        loopRate.sleep();
    }
} 

void CollisionAlgorithmsVoxels::collisionCheckPredictionHorizon(bool filter, int collThreshold, int jumpThreshold)
{
    // check swept and "start" scene
    if(this->checkNULL != RobotSceneOK)
        RETURN_WITH_MSG("Pointeri \"robot\" i \"scene\" ne smiju biti nullptr", CollisionAlgorithmsVoxels::collisionCheckPredictionHorizon)
    else if(robot->getSweptType() == InsertRobotVoxels::ST_EMPTY)
        RETURN_WITH_MSG("Swept nije incijaliziran", CollisionAlgorithmsVoxels::collisionCheckPredictionHorizon)
    else if(!scene->startSceneSpinners())
        RETURN_WITH_MSG("Spinner-i nisu pokrenuti", CollisionAlgorithmsVoxels::collisionCheckPredictionHorizon)

    // local vars
    int k = 0;
    size_t numColl;
    BitVectorVoxel collisionBits;

    // wait for first PointClouds
    ROS_WARN("Cekam 1 sekundu prije pocetka, kako bi dobio prve PointCloud-ove ...");
    ros::Duration(1).sleep();
        
    // remove robot
    this->scene->removeRobotFromScene(this->robot, filter);

    while(ros::ok())
    {
        this->timer.tic();
        // update scene and swept
        scene->updateSceneMap();
        robot->updateSweptMap();

        // voxels in collision at prediction horizon (set threshold when working with real 3D camera)
        scene->lockThread();
        if(robot->getSweptType() == InsertRobotVoxels::ST_MAP)
            numColl = bb_cuda::collideMapsFastGPU(scene->getSceneVoxelMap(), robot->getSweptVoxelMap(), 0.75, 2, 512);
        else
            numColl = robot->getSweptVoxelList()->collideWithTypes(scene->getSceneVoxelMap().get(), collisionBits);
        scene->unlockThread();

        robot->visualizeSwept();
        scene->visualizeScene();

        // take actions if collision is detected
        if(numColl > collThreshold)
        {
            // eliminate sudden collision "jumps"
            if (k >= jumpThreshold){
                this->collisionRobotAction(numColl, 0.02*2, 0.05*2);
                //ROS_ERROR_STREAM("Broj kolizija: " << numColl);
            }
            else
                k++;
        }
        else
            k = 0;
        
        this->timer.tocMili("Vrijeme detekcije kolizije");
    }
}

void CollisionAlgorithmsVoxels::collisionCheckSafeZones()
{
    if(this->checkNULL != RobotSceneOK)
        RETURN_WITH_MSG("Pointeri \"robot\" i \"scene\" ne smiju biti nullptr", CollisionAlgorithmsVoxels::collisionCheckSafeZones)

    // wait for first PointClouds
    ROS_WARN("Cekam 1 sekundu prije pocetka, kako bi dobio prve PointCloud-ove ...");
    ros::Duration(1).sleep();

    Eigen::Vector3f point = this->robot->getBaseLinkPose();
    
    this->scene->removeRobotFromScene(this->robot, true);

    while(ros::ok())
    {
        this->robot->updateRobotMap();
        this->scene->updateSceneMap();     
    }

}

void CollisionAlgorithmsVoxels::collisionRobotAction(int numColl, float scalingV, float scalingA)
{   
    ROS_ERROR_STREAM("Broj voxela u koliziji: " << numColl);
    
    if (this->actionServerConnected) 
    {
        if(this->actionServer->isServerConnected())
        {

            //this->actionServer->cancelAllGoals();
            /*
            ROS_ERROR("Zaustavljam robot i blokiram na 10 sec!");
            ros::Duration(5).sleep();
            ROS_INFO("Zavrsio cekanje. Ponovno pokrecem robot!");

            // max. velocities and accelerations for joints
            std::vector<double> maxV{3.83*scalingV, 3.66*scalingV, 4.71*scalingV, 6.65*scalingV, 5.43*scalingV, 8.59*scalingV};
            std::vector<double> maxA{10.0*scalingA, 3.0*scalingA, 10.0*scalingA, 10.0*scalingA, 10.0*scalingA, 10.0*scalingA};

            // take rest of trajectory and continue movement
            robot->updateRobotMap();
            gpu_voxels::robot::JointValueMap currentJoints = robot->getJointValues();
            trajectory_msgs::JointTrajectoryPoint trajPoint;
            trajectory_msgs::JointTrajectory traj = robot->getCurrentTrajectory(), newTraj;
            int point;
            float min, a1, a2, a3, a4, a5, a6, sum;

            // joint names
            newTraj.joint_names = traj.joint_names;

            // first point -> current joint states
            trajPoint.positions.push_back(currentJoints["joint_a1"]);
            trajPoint.positions.push_back(currentJoints["joint_a2"]);
            trajPoint.positions.push_back(currentJoints["joint_a3"]);
            trajPoint.positions.push_back(currentJoints["joint_a4"]);
            trajPoint.positions.push_back(currentJoints["joint_a5"]);
            trajPoint.positions.push_back(currentJoints["joint_a6"]);

            trajPoint.velocities = maxV;
            trajPoint.accelerations = maxA;

            newTraj.points.push_back(trajPoint);

            for(int i = 0; i < traj.points.size(); i++)
            {
                a1 = std::pow(currentJoints["joint_a1"] - traj.points[i].positions[0], 2);
                a2 = std::pow(currentJoints["joint_a2"] - traj.points[i].positions[1], 2);
                a3 = std::pow(currentJoints["joint_a3"] - traj.points[i].positions[2], 2);
                a4 = std::pow(currentJoints["joint_a4"] - traj.points[i].positions[3], 2);
                a5 = std::pow(currentJoints["joint_a5"] - traj.points[i].positions[4], 2);
                a6 = std::pow(currentJoints["joint_a6"] - traj.points[i].positions[5], 2);

               sum = std::sqrt(a1 + a2 + a3 + a4 + a5 + a6);

                if(i == 0)
                {
                    min = sum;
                    point = 0;
                }
                else if (sum < min)
                {
                    min = sum;
                    point = i;
                }  
            }

            ROS_INFO_STREAM("Tocnost pronalaska tocke na trajektoriji: " << min);

            for(int i = point; i < traj.points.size(); i++)
            {
                trajPoint = traj.points[i];
                trajPoint.velocities = maxV;
                trajPoint.accelerations = maxA;
                
                newTraj.points.push_back(trajPoint);
            }
        
            // call TOPP-RA
            topp_ros::GenerateTrajectoryRequest topp_req;
            topp_ros::GenerateTrajectoryResponse topp_res;

            topp_req.waypoints = newTraj;
            topp_req.sampling_frequency = 100;
            topp_req.plot = false;

            if(this->toppRA.call(topp_req, topp_res))
            {
                ROS_INFO("TOPP RA: trajektorija isplanirana uspjesno!");

                // put header and joint names
                topp_res.trajectory.header.stamp = ros::Time::now();
                topp_res.trajectory.joint_names = traj.joint_names;

                // topic cmd
                this->pubCmd.publish(topp_res.trajectory);

                // action server 
                // control_msgs::FollowJointTrajectoryGoal goal;
                // goal.trajectory = topp_res.trajectory;
                // this->actionServer->sendGoal(goal); 
            }
            else
                ROS_ERROR("TOPP RA: pogreska prilikom planiranja trajektorije");
            */
        }
        else
            ROS_ERROR_STREAM("Izgubljena konekcija s action serverom: " << this->actionServerName << ", izgledna kolizija!!");
    }
}

void CollisionAlgorithmsVoxels::safeZonesBaseLinkRobotAction(float distanceToBaseLink)
{
    if (distanceToBaseLink < 0.5)
    {
        ROS_ERROR("Zaustavljam robot. Objekt je blizu baze!");
        if(this->actionServerConnected)
            this->actionServer->cancelAllGoals();
    }
}
