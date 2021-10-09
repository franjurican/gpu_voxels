// C++ stl
#include <cmath>

// header for InsertRobotVoxels class
#include <gpu_collision_check/gpu_collision/insert_robot_voxels.h>

// macro function
#define name(x) #x

// namespace
using namespace gpu_voxels; 

InsertRobotVoxels::InsertRobotVoxels(GpuVoxels *gvl, Eigen::Vector3i mapSize, float voxelSize) : n(), np("~"), gvl(gvl), 
        alreadyInit(false), alreadyInitBB(false), alreadySub(false), trajectoryLoaded(false), sweptType(ST_EMPTY), currentTrajPoint(0)
{
    // init map size for gvl pointer
    float vs;
    gvl->getVoxelSideLength(vs);

    if(vs == 0.0)
        this->gvl->initialize(mapSize.x(), mapSize.y(), mapSize.z(), voxelSize);
    else
    {
        uint32_t x, y, z;
        gvl->getDimensions(x, y, z);
        ROS_WARN("Gpu-Voxels (gvl) je vec inicijaliziran! Gpu-Voxels NECE biti ponovno inicijaliziran!");
        ROS_WARN_STREAM("Parametri Gpu-Voxelsa: x = " << x << ", y = " << y << ", z = " << z << ", voxelSize = " << vs);
    }

    // get parameters from ROS parameter server for auto collision check
    if(!np.param<bool>("predictionHorizon/enabled", predictionHorizon.status, true))
        np.setParam("predictionHorizon/enabled", predictionHorizon.status);
    
    if(!np.param<float>("predictionHorizon/minimalAngle", predictionHorizon.minAngle, 0.21)) 
        np.setParam("predictionHorizon/minimalAngle", predictionHorizon.minAngle); 
    
    if(!np.param<float>("predictionHorizon/maximalAngle", predictionHorizon.maxAngle, 0.35)) 
        np.setParam("predictionHorizon/maximalAngle", predictionHorizon.maxAngle);
    
    if(!np.param<float>("predictionHorizon/stepSize", predictionHorizon.stepSize, 0.01))
        np.setParam("predictionHorizon/stepSize", predictionHorizon.stepSize);
    
    if(!np.param<int>("predictionHorizon/sweptLevelsNumber", predictionHorizon.sweptNum, 2))
        np.setParam("predictionHorizon/sweptLevelsNumber", predictionHorizon.sweptNum);

    if(!np.param<std::string>("dummyBaseNameX", dummyX, "base_x"))
        np.setParam("dummyBaseNameX", dummyX);

    if(!np.param<std::string>("dummyBaseNameY", dummyY, "base_y"))
        np.setParam("dummyBaseNameY", dummyY);
    
    if(!np.param<std::string>("dummyBaseNameZ", dummyZ, "base_z"))
        np.setParam("dummyBaseNameZ", dummyZ);

    // print info
    ROS_INFO("Pocetni parametri provjere kolizije:");
    ROS_INFO_STREAM("Predikcijski horizont: " << (this->predictionHorizon.status ? "ukljucen" : "iskljucen")); 
    ROS_INFO_STREAM("Minimalni kut predikcijskog horizonta : " << this->predictionHorizon.minAngle);
    ROS_INFO_STREAM("Maksimalni kut predikcijskog horizonta : " << this->predictionHorizon.maxAngle);
    ROS_INFO_STREAM("Velicina koraka predikcijskog horizonta: " << this->predictionHorizon.stepSize);
    ROS_INFO_STREAM("Broj levela prosirenog volumena: " << this->predictionHorizon.sweptNum);
    ROS_INFO_STREAM("Ime \"laznog\" zgloba baze za gibanje u x smjeru: " + this->dummyX);
    ROS_INFO_STREAM("Ime \"laznog\" zgloba baze za gibanje u y smjeru: " + this->dummyY);
    ROS_INFO_STREAM("Ime \"laznog\" zgloba baze za gibanje u z smjeru: " + this->dummyZ);
    ROS_INFO_STREAM("Uspjesno kreirano sucelje za voxelizaciju robota: " << name(InsertRobotVoxels));
}

InsertRobotVoxels::~InsertRobotVoxels() 
{   
    // delete params
    np.deleteParam("robotName");
    np.deleteParam("usingBBModel");
    np.deleteParam("robotVoxelMapName");
    np.deleteParam("robotVisualizationVoxelMapName");
    np.deleteParam("sweptVoxelName");
    np.deleteParam("predictionHorizon/enabled");
    np.deleteParam("predictionHorizon/minimalAngle");
    np.deleteParam("predictionHorizon/maximalAngle");
    np.deleteParam("predictionHorizon/stepSize");
    np.deleteParam("predictionHorizon/sweptLevelsNumber");
    np.deleteParam("jointStatesTopicName");
    np.deleteParam("dummyBaseNameX");
    np.deleteParam("dummyBaseNameY");
    np.deleteParam("dummyBaseNameZ");

    std::cout << "Pozvan destruktor: " << name(InsertRobotVoxels) << std::endl;
}

void InsertRobotVoxels::initializeRobot(std::string robotName, std::string robotURDF, bool moveBase, Eigen::Vector3f baseLinkPose, 
                                                                                                               bool modelPath)
{   
    // init only ONCE
    if(this->alreadyInit)
    {
        ROS_WARN_STREAM("Robot \"" << this->robotName << "\" je vec inicijaliziran. Robot NECE biti ponovno inicijaliziran.");
        return;
    }

    // save robot name
    this->robotName = robotName;
    this->robotBBName = robotName;

    // add robot generated from a ROS URDF file:
    if(gvl->addRobot(this->robotName, robotURDF, modelPath))
    {   
        // add voxel map for ROBOT
        this->robotMapName = this->robotName + "VoxelMap";
        gvl->addMap(MT_BITVECTOR_VOXELMAP, this->robotMapName);
        this->robotVoxelMap = boost::dynamic_pointer_cast<voxelmap::BitVectorVoxelMap>(gvl->getMap(this->robotMapName));

        // add voxel map for ROBOT visualization
        this->robotVisMapName = this->robotName + "VisualizationVoxelMap";
        gvl->addMap(MT_DISTANCE_VOXELMAP, this->robotVisMapName);
        this->robotVisVoxelMap = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap(this->robotVisMapName));

        // info
        this->alreadyInit = true;
        np.setParam("robotName", this->robotName);
        np.setParam("robotVoxelMapName", this->robotMapName);
        np.setParam("robotVisualizationVoxelMapName", this->robotVisMapName);
        np.setParam("usingBBModel", false);
        ROS_INFO_STREAM("Voxelizirani model robota \"" << this->robotName << "\" ucitan iz URDF-a");
        ROS_INFO_STREAM("Robot \"" << this->robotName << "\" inicijaliziran u node-u: " << ros::this_node::getName());
        ROS_INFO_STREAM("Kreirane voxel mape za robot \"" << this->robotName << "\": "); 
        ROS_INFO_STREAM("Voxel mapa tipa " << name(MT_BITVECTOR_VOXELMAP) << ": " << this->robotMapName);
        ROS_INFO_STREAM("Voxel mapa tipa " << name(MT_DISTANCE_VOXELMAP) << ": " << this->robotVisMapName);      
    } 
    else
    {
        ROS_ERROR("Pogreska prilikom ucitavanja voxeliziranog modela robota iz URDF-a!");
        ros::shutdown();
        return;
    }

    // set initial robot pose (position of base link) with moveRobotBase
    if(moveBase)
        this->moveRobotBase(baseLinkPose, false, this->dummyX, this->dummyY, this->dummyZ);
}

void InsertRobotVoxels::initializeSwept(SweptType ST)
{
    if(!this->alreadyInit)
    {
        ROS_WARN_STREAM("Robot mora biti inicijaliziran (" << name(InsertRobotVoxels::initializeRobot) << ")!");
        ROS_WARN_STREAM("Swept NECE biti inicijaliziran");
        return;   
    }
    else if(this->sweptType != ST_EMPTY)
    {
        ROS_WARN_STREAM("Swept je vec inicijaliziran. Swept NECE biti ponovno inicijaliziran.");
        ROS_WARN_STREAM("Swept tip: " << ( (this->sweptType == ST_LIST) ? "ST_LIST" : "ST_MAP" ) );
        return;
    }
    else if(ST == ST_LIST)
    {
        // add voxel list for SWEPT
        this->sweptType = ST;
        this->sweptListName = this->robotName + "SweptVoxelList";
        gvl->addMap(MT_BITVECTOR_VOXELLIST, this->sweptListName);
        this->sweptVoxelList = boost::dynamic_pointer_cast<voxellist::BitVectorVoxelList>(gvl->getMap(this->sweptListName));

        // info
        np.setParam("sweptVoxelName", this->sweptListName);
        ROS_INFO_STREAM("Kreirana swept voxel lista tipa " << name(MT_BITVECTOR_VOXELLIST) << ": " << this->sweptListName);
    }
    else
    {
        // add voxel map for SWEPT
        this->sweptType = ST;
        this->sweptMapName = this->robotName + "SweptVoxelMap";
        gvl->addMap(MT_BITVECTOR_VOXELMAP, this->sweptMapName);
        this->sweptVoxelMap = boost::dynamic_pointer_cast<voxelmap::BitVectorVoxelMap>(gvl->getMap(this->sweptMapName));

        // add voxel map for SWEPT visualization
        this->sweptVisMapName = this->robotName + "SweptVisualizationVoxelMap";
        gvl->addMap(MT_DISTANCE_VOXELMAP, this->sweptVisMapName);
        this->sweptVisVoxelMap = boost::dynamic_pointer_cast<voxelmap::DistanceVoxelMap>(gvl->getMap(this->sweptVisMapName)); 

        // info
        np.setParam("sweptVoxelName", this->sweptMapName);
        ROS_INFO_STREAM("Kreirana swept voxel mapa tipa " << name(MT_BITVECTOR_VOXELMAP) << ": " << this->sweptMapName);
        ROS_INFO_STREAM("Kreirana swept voxel mapa tipa " << name(MT_DISTANCE_VOXELMAP) << ": " << this->sweptVisMapName);
    }
}

void InsertRobotVoxels::initializeRobotBB(std::string robotURDF, bool modelPath)
{
    // robot init
    if(!this->alreadyInit)
    {
        ROS_WARN_STREAM("Robot mora biti inicijaliziran (" << name(InsertRobotVoxels::initializeRobot) << ")!");
        ROS_WARN_STREAM("BB model robota NECE biti ucitan!");
        return;
    } 
    else if (this->alreadyInitBB)
    {
        ROS_WARN_STREAM("BB model robota \"" << this->robotName << "\" je vec inicijaliziran. BB model robota NECE biti ponovno inicijaliziran.");
        return;
    }

    // load BB robot from URDF
    if(gvl->addRobot(this->robotName + "BB", robotURDF, modelPath))
    {   
        this->robotBBName = this->robotName + "BB";
        this->alreadyInitBB = true; 
        np.setParam("usingBBModel", true);   
        ROS_INFO("BB model robota ucitan!");  
    } 
    else
    {
        ROS_WARN("Pogreska prilikom ucitavanja voxeliziranog BB modela robota iz URDF-a!");
        return;
    }

    // load robot into map
    this->loadRobotIntoMap(this->jointValues); 
}

void InsertRobotVoxels::startSubscribing(std::string jointStatesTopicName)
{
    if(this->alreadySub)
    {
        ROS_WARN_STREAM("Robot \"" << this->robotName << "\" je vec preplacen na topic: " << this->topicName);
        ROS_WARN_STREAM("ROS service \"" << this->serviceName << "\" za robot \"" << this->robotName << "\" je vec kreiran!");
        return;
    }

    // ros callback for joint states (goes into node global queue)
    this->sub = n.subscribe(jointStatesTopicName, 1, &InsertRobotVoxels::jointStateCallback, this);
    this->alreadySub = true;
    this->topicName = jointStatesTopicName;
    np.setParam("jointStatesTopicName", this->topicName);

    // ros service
    this->serviceName = "add_trajectory_for_collision_check";
    this->service = np.advertiseService(this->serviceName, &InsertRobotVoxels::trajectoryServiceCallback, this);

    // info
    ROS_INFO_STREAM("Robot \"" << this->robotName << "\" je preplacen na topic: \"" << this->topicName << "\".");
    ROS_INFO_STREAM("Kreiran ROS service \"" << this->serviceName << "\" za robot \"" << this->robotName << "\".");
}

void InsertRobotVoxels::setJointValuesManually(robot::JointValueMap jointValues)
{
    this->jointValues = jointValues;
}

bool InsertRobotVoxels::isInitialized()
{
    return this->alreadyInit;
}

void InsertRobotVoxels::lockThread()
{
    this->mtx.lock();
}

void InsertRobotVoxels::unlockThread()
{
    this->mtx.unlock();
}

Eigen::Vector3f InsertRobotVoxels::getBaseLinkPose()
{
    // already init?
    if(!alreadyInit)
    {
        ROS_WARN("Robot nije inicijaliziran. Base link ne postoji!");
        return Eigen::Vector3f(NAN, NAN, NAN);
    }

    return this->baseLinkPose;
}

InsertRobotVoxels::SweptType InsertRobotVoxels::getSweptType()
{
    return this->sweptType;
}

std::string InsertRobotVoxels::getRobotName()
{
    return this->robotName;
}

std::string InsertRobotVoxels::getRobotMapName()
{
    return this->robotMapName;
}

std::string InsertRobotVoxels::getRobotVisualizationMapName()
{
    return this->robotVisMapName;
}

std::string InsertRobotVoxels::getSweptListName()
{
    return this->sweptListName;
}

std::string InsertRobotVoxels::getSweptMapName()
{
    return this->sweptMapName;
}

std::string InsertRobotVoxels::getSweptVisualizationMapName()
{
    return this->sweptVisMapName;
}

InsertRobotVoxels::BitVectorVoxelMapPtr InsertRobotVoxels::getRobotVoxelMap()
{
    return (this->alreadyInit ? this->robotVoxelMap : nullptr);
}

InsertRobotVoxels::DistanceVoxelMapPtr InsertRobotVoxels::getRobotVisualizationVoxelMap()
{
    return (this->alreadyInit ? this->robotVisVoxelMap : nullptr);
}

InsertRobotVoxels::BitVectorVoxelListPtr InsertRobotVoxels::getSweptVoxelList()
{
    return ( (this->sweptType == ST_LIST) ? this->sweptVoxelList : nullptr );
}

InsertRobotVoxels::BitVectorVoxelMapPtr InsertRobotVoxels::getSweptVoxelMap()
{
    return ( (this->sweptType == ST_MAP) ? this->sweptVoxelMap : nullptr );
}

InsertRobotVoxels::DistanceVoxelMapPtr InsertRobotVoxels::getSweptVisualizationVoxelMap()
{
    return ( (this->sweptType == ST_MAP) ? this->sweptVisVoxelMap : nullptr );
}

robot::JointValueMap InsertRobotVoxels::getJointValues()
{
    // lock this method
    boost::lock_guard<boost::mutex> guard(this->mtx);

    return this->jointValues;
}

InsertRobotVoxels::PredictionHorizonStruct InsertRobotVoxels::getPredictionHorizonInfo()
{
    return this->predictionHorizon;
}
 
void InsertRobotVoxels::enablePredictionHorizon()
{
    this->predictionHorizon.status = true;
    np.setParam("predictionHorizon/enabled", this->predictionHorizon.status);
    ROS_INFO("Predikcijski horizont ukljucen!");
}

void InsertRobotVoxels::disablePredictionHorizon()
{
    this->predictionHorizon.status = false;
    np.setParam("predictionHorizon/enabled", this->predictionHorizon.status);
    ROS_INFO("Predikcijski horizont iskljucen!");
}

void InsertRobotVoxels::configPredictionHorizon(float minAngle, float maxAngle, float stepSize, int sweptNum)
{
    this->predictionHorizon.minAngle = minAngle;
    this->predictionHorizon.maxAngle = maxAngle;
    this->predictionHorizon.stepSize = stepSize;
    this->predictionHorizon.sweptNum = sweptNum;
    np.setParam("predictionHorizon/minimalAngle", this->predictionHorizon.minAngle);
    np.setParam("predictionHorizon/maximalAngle", this->predictionHorizon.maxAngle);
    np.setParam("predictionHorizon/stepSize", this->predictionHorizon.stepSize);
    np.setParam("predictionHorizon/sweptLevelsNumber", this->predictionHorizon.sweptNum);
    ROS_INFO_STREAM("Promijenjen minimalni kut predikcijskog horizonta  : " << this->predictionHorizon.minAngle);
    ROS_INFO_STREAM("Promijenjen maksimalni kut predikcijskog horizonta  : " << this->predictionHorizon.maxAngle);
    ROS_INFO_STREAM("Promijenjena velicina koraka predikcijskog horizonta  : " << this->predictionHorizon.stepSize);
    ROS_INFO_STREAM("Promijenjen broj levela prosirenog volumena: " << this->predictionHorizon.sweptNum);
}

void InsertRobotVoxels::visualizeRobot()
{
    // already init?
    if(!alreadyInit)
    {
        ROS_WARN("Robot nije inicijaliziran. Robot NECE biti vizualiziran!");
        return;
    }

    gvl->visualizeMap(this->robotVisMapName);       
}

void InsertRobotVoxels::visualizeSwept()
{
    // already init?
    if(this->sweptType == ST_EMPTY)
    {
        ROS_WARN("Swept nije inicijaliziran. Swept NECE biti vizualiziran!");
        return;
    }
    else if (this->sweptType == ST_LIST)
        gvl->visualizeMap(this->sweptListName); 
    else
        gvl->visualizeMap(this->sweptVisMapName);       
}

void InsertRobotVoxels::moveRobotBase(Eigen::Vector3f basePose, bool visualize, std::string dummyX, std::string dummyY, std::string dummyZ)
{   
    // already init?
    if(!alreadyInit)
    {
        ROS_WARN("Robot nije inicijaliziran. Baza robota NECE biti pomaknuta!");
        return;
    }

    // dummy names
    if(dummyX.empty() || dummyY.empty() || dummyZ.empty())
    {
        ROS_WARN("Ime dummy joints-a NE moze biti prazno. Baza nije pomaknuta!");
        return;
    }
    
    // move base
    this->baseLinkPose = basePose;
    this->jointValues[dummyX] = basePose.x();
    this->jointValues[dummyY] = basePose.y();
    this->jointValues[dummyZ] = basePose.z();

    // load robot into map
    this->loadRobotIntoMap(this->jointValues);

    if(visualize)
    {
        this->visualizeRobot();
        ros::Duration(1).sleep(); // wait for visualization rendering only at start!
    }
}

robot::JointValueMap InsertRobotVoxels::jointsLinearInterpolation(robot::JointValueMap start, robot::JointValueMap end, 
                                                                                                                float ratio)
{
    robot::JointValueMap out;
    
    // interpolate
    for(auto i = start.cbegin(); i != start.cend(); i++)
    {
        if (end.find(i->first) != end.end())
        {
            out[i->first] = (end[i->first] - i->second)*ratio + i->second;
        }
        else
        {
            ROS_WARN_STREAM("Mapa \"" << name(robot::JointValueMap end) << "\" NE sadrzi clan \"" << i->first << "\"!");
            ROS_WARN_STREAM("Vracam samo iznos clana \"" << i->first << "\" iz mape \"" << name(robot::JointValueMap start) << "\"!");
            out[i->first] = i->second;
        }
        
    }

    return out;
}

void InsertRobotVoxels::updateRobotMap()
{
    ros::spinOnce();
    this->lockThread();
    this->loadRobotIntoMap(this->jointValues);
    this->unlockThread();
}

void InsertRobotVoxels::updateSweptMap()
{
    float r;
    robot::JointValueMap start, end;

    // lock thread and get current joint values
    this->lockThread();
    start = this->jointValues;
    this->unlockThread();

    // create and load swept
    end = this->selectNextPoint(start, 0.6);
    r = this->calculateRatio(start, end, this->predictionHorizon.stepSize, this->predictionHorizon.minAngle, 
                                                                                           this->predictionHorizon.maxAngle);
    this->createSweptVolume(start, end, this->predictionHorizon.sweptNum, r);
}

trajectory_msgs::JointTrajectory InsertRobotVoxels::getCurrentTrajectory()
{
    if(this->trajectoryLoaded)
        return this->currentTraj;
    
    trajectory_msgs::JointTrajectory traj;
    return traj;
}

int InsertRobotVoxels::getCurrentPointOnTrajectory()
{
    return this->currentTrajPoint;
}

void InsertRobotVoxels::loadRobotIntoMap(robot::JointValueMap joints, bool swept, BitVoxelMeaning ID)
{  
    // already init?
    if(!alreadyInit)
    {
        ROS_WARN("Robot nije inicijaliziran. Robot NECE biti ucitan u mapu!");
        return;
    }

    // set configuration with new joint values
    gvl->setRobotConfiguration(this->robotName, joints);
    if(this->alreadyInitBB)
    {
         gvl->setRobotConfiguration(this->robotBBName, joints);
    }
       
    // for swept DON'T clear map/list
    if(swept)
    {   
        // insert robot into SWEPT list/map
        if(this->sweptType == ST_LIST)
            gvl->insertRobotIntoMap(this->robotName, this->sweptListName, ID);
        else
        {
            gvl->insertRobotIntoMap(this->robotBBName, this->sweptMapName, ID);
            gvl->insertRobotIntoMap(this->robotName, this->sweptVisMapName, eBVM_OCCUPIED);
        }    
    }
    else
    {
        // clear and insert robot into robots map and visualization map
        gvl->clearMap(this->robotMapName);
        gvl->clearMap(this->robotVisMapName);
        gvl->insertRobotIntoMap(this->robotName, this->robotVisMapName, eBVM_OCCUPIED);
        gvl->insertRobotIntoMap(this->robotBBName, this->robotMapName, eBVM_OCCUPIED);
    }
}

bool InsertRobotVoxels::trajectoryServiceCallback(gpu_collision_check::TrajectoryForCollisionRequest &req, 
                                                  gpu_collision_check::TrajectoryForCollisionResponse &res)
{
    // check msg
    if(req.waypoints.joint_names.size() != 6)
        res.error.data = "Invalid number of joints. Number of joints must be 6!";
    else if(!(req.waypoints.joint_names[0] == "joint_a1"))
        res.error.data = "Inavlid name for first joint. Joint name must be \"joint_a1\"!";
    else if(!(req.waypoints.joint_names[1] == "joint_a2"))
        res.error.data = "Inavlid name for second joint. Joint name must be \"joint_a2\"!";
    else if(!(req.waypoints.joint_names[2] == "joint_a3"))
        res.error.data = "Inavlid name for third joint. Joint name must be \"joint_a3\"!";
    else if(!(req.waypoints.joint_names[3] == "joint_a4"))
        res.error.data = "Inavlid name for fourth joint. Joint name must be \"joint_a4\"!";
    else if(!(req.waypoints.joint_names[4] == "joint_a5"))
        res.error.data = "Inavlid name for fifth joint. Joint name must be \"joint_a5\"!";
    else if(!(req.waypoints.joint_names[5] == "joint_a6"))
        res.error.data = "Inavlid name for sixth joint. Joint name must be \"joint_a6\"!";
    else if(req.waypoints.points.size() == 0)
        res.error.data = "Zero trajectory points send!";
    else
    {
        res.error.data = "Server received message. Message seems valid!";
        this->trajectoryLoaded = true;
        this->currentTrajPoint = 0;
        this->currentTraj = req.waypoints;
    }

    return true;
}

void InsertRobotVoxels::createSweptVolume(robot::JointValueMap start, robot::JointValueMap end, int sweptNum, float ratio)
{   
    // already init?
    if(this->sweptType == ST_EMPTY)
    {
        ROS_WARN("Swept nije inicijaliziran. Prosireni volumeni NECE biti kreirani!");
        return;
    } 
    else if (!this->trajectoryLoaded)
        return;

    // new joint values
    robot::JointValueMap jMap;
    float k;

    // clear swept map, only at begining
    if(this->sweptType == ST_LIST)
        this->sweptVoxelList->clearMap();
    else
    {
        this->sweptVoxelMap->clearMap();
        this->sweptVisVoxelMap->clearMap();
    }
    
    // create swept
    for(int i = 0; i < sweptNum; i++)
    {
        // "fill" prediction horizon
        k = ((float)(sweptNum - i))/(sweptNum);
        jMap = this->jointsLinearInterpolation(start, end, ratio*k);
        
        // insert voxels into swept list with ID!
        this->loadRobotIntoMap(jMap, true, BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + i));
    }
}

float InsertRobotVoxels::calculateRatio(robot::JointValueMap start, robot::JointValueMap end, float stepSize, 
                                                                                             float minAngle, float maxAngle)
{   
    if(!this->trajectoryLoaded)
        return 1.0;

    int i = 0;
    float k;
    float joint1, joint2, joint3, joint4, joint5, joint6;
    float jointDif1, jointDif2, jointDif3, jointDif4, jointDif5, jointDif6;

    // joints range
    jointDif1 = std::abs(end["joint_a1"] - start["joint_a1"]);
    jointDif2 = std::abs(end["joint_a2"] - start["joint_a2"]);
    jointDif3 = std::abs(end["joint_a3"] - start["joint_a3"]);
    jointDif4 = std::abs(end["joint_a4"] - start["joint_a4"]);
    jointDif5 = std::abs(end["joint_a5"] - start["joint_a5"]);  
    jointDif6 = std::abs(end["joint_a6"] - start["joint_a6"]);  

    // close to end?
    if( (jointDif1 <= minAngle) && (jointDif2 <= minAngle) && (jointDif3 <= minAngle) && (jointDif4 <= minAngle) 
                                                                    && (jointDif5 <= minAngle) && (jointDif6 <= minAngle))
    {
        if(this->currentTrajPoint == this->currentTraj.points.size())
        {
            this->currentTrajPoint = 0;
            this->trajectoryLoaded = false;
            ROS_WARN("Zavrsio trajektoriju!");
        }
        return 1.0;
    }

    for(k = stepSize; k <= 1; k += stepSize, i++)
    {
        // joints
        joint1 = jointDif1*k;
        joint2 = jointDif2*k;
        joint3 = jointDif3*k;  
        joint4 = jointDif4*k;  
        joint5 = jointDif5*k;  
        joint6 = jointDif6*k;  

        if( (joint1 > maxAngle) || (joint2 > maxAngle) || (joint3 > maxAngle) )
        {
            break;
        }
        else if ( (joint4 > maxAngle) || (joint5 > maxAngle) || (joint6 > maxAngle) )
        {
            if( (joint1 > minAngle) || (joint2 > minAngle) || (joint3 > minAngle) )
            {
                break;
            }
            else if( (jointDif1 < minAngle) && (jointDif2 < minAngle) && (jointDif3 < minAngle) )
            {
                break;
            }     

        } else if( (joint4 > 2*maxAngle) || (joint5 > 2*maxAngle) || (joint6 > 2*maxAngle) ) 
        {
            break;
        }
    }

    return k;
}

gpu_voxels::robot::JointValueMap InsertRobotVoxels::selectNextPoint(gpu_voxels::robot::JointValueMap currentValue, float distanceToPoint)
{
    robot::JointValueMap out;
    float a1, a2, a3, a4, a5, a6, distance; 

    if(!this->trajectoryLoaded)
        return out;

    for(this->currentTrajPoint; this->currentTrajPoint < this->currentTraj.points.size(); this->currentTrajPoint++)
    {
        a1 = std::pow(currentValue["joint_a1"] - this->currentTraj.points[this->currentTrajPoint].positions[0], 2);
        a2 = std::pow(currentValue["joint_a2"] - this->currentTraj.points[this->currentTrajPoint].positions[1], 2);
        a3 = std::pow(currentValue["joint_a3"] - this->currentTraj.points[this->currentTrajPoint].positions[2], 2);
        a4 = std::pow(currentValue["joint_a4"] - this->currentTraj.points[this->currentTrajPoint].positions[3], 2);
        a5 = std::pow(currentValue["joint_a5"] - this->currentTraj.points[this->currentTrajPoint].positions[4], 2);
        a6 = std::pow(currentValue["joint_a6"] - this->currentTraj.points[this->currentTrajPoint].positions[5], 2);

        distance = std::sqrt(a1 + a2 + a3 + a4/5.0 + a5 + a6/5.0);

        if (distance >= distanceToPoint)
        {
            out["joint_a1"] = this->currentTraj.points[this->currentTrajPoint].positions[0];
            out["joint_a2"] = this->currentTraj.points[this->currentTrajPoint].positions[1];
            out["joint_a3"] = this->currentTraj.points[this->currentTrajPoint].positions[2];
            out["joint_a4"] = this->currentTraj.points[this->currentTrajPoint].positions[3];
            out["joint_a5"] = this->currentTraj.points[this->currentTrajPoint].positions[4];
            out["joint_a6"] = this->currentTraj.points[this->currentTrajPoint].positions[5];
            out[this->dummyX] = this->baseLinkPose.x();
            out[this->dummyY] = this->baseLinkPose.y();
            out[this->dummyZ] = this->baseLinkPose.z();

            this->currentTrajPoint++;
            return out;
        }
    }

    out["joint_a1"] = this->currentTraj.points[this->currentTrajPoint - 1].positions[0];
    out["joint_a2"] = this->currentTraj.points[this->currentTrajPoint - 1].positions[1];
    out["joint_a3"] = this->currentTraj.points[this->currentTrajPoint - 1].positions[2];
    out["joint_a4"] = this->currentTraj.points[this->currentTrajPoint - 1].positions[3];
    out["joint_a5"] = this->currentTraj.points[this->currentTrajPoint - 1].positions[4];
    out["joint_a6"] = this->currentTraj.points[this->currentTrajPoint - 1].positions[5];
    out[this->dummyX] = this->baseLinkPose.x();
    out[this->dummyY] = this->baseLinkPose.y();
    out[this->dummyZ] = this->baseLinkPose.z();
    
    return out;
}

void InsertRobotVoxels::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{   
     // lock this method
    boost::lock_guard<boost::mutex> guard(this->mtx);

    // map joints
    for(size_t i = 0; i < msg->name.size(); i++) 
        this->jointValues[msg->name[i]] = msg->position[i];    
}