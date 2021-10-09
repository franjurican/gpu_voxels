// header for InsertSceneVoxels class
#include <gpu_collision_check/gpu_collision/insert_scene_voxels.h>

// macro function
#define name(x) #x

// Namespace
using namespace gpu_voxels; 

InsertSceneVoxels::InsertSceneVoxels(GpuVoxels *gvl, Eigen::Vector3i mapSize, float voxelSize, Eigen::Vector3f originOffset) 
         : n(), np("~"), spinner1(1, &kinectQueue1), spinner2(1, &kinectQueue2), tfListener(tfBuff), timer("scena"), gvl(gvl)
{   
    // kinect number is zero until registration
    this->kinectNum = 0;
    
    // init map
    float vs;
    gvl->getVoxelSideLength(vs);

    if(vs == 0.0)
    {
        this->gvl->initialize(mapSize.x(), mapSize.y(), mapSize.z(), voxelSize);
    }
    else
    {
        uint32_t x, y, z;
        gvl->getDimensions(x, y, z);
        ROS_WARN("Gpu-Voxels (gvl) je vec inicijaliziran! Gpu-Voxels NECE biti ponovno inicijaliziran!");
        ROS_WARN_STREAM("Parametri Gpu-Voxelsa: x = " << x << ", y = " << y << ", z = " << z << ", voxelSize = " << vs);
    }
        
    // origin offset
    this->originOffset = originOffset;

    // robot cut
    this->removeRobot = false;
    np.setParam("scene/removeRobotFromScene", this->removeRobot);
    
    // info
    ROS_INFO("Pocetni parametri scene: ");
    ROS_INFO_STREAM("Rezanje robota iz scene: iskljuceno");
    ROS_INFO_STREAM("Uspjesno kreirano sucelje za voxelizaciju scene: " << name(InsertSceneVoxels));
}

InsertSceneVoxels::~InsertSceneVoxels() 
{
    // delete param
    np.deleteParam("scene/kinectNum");
    np.deleteParam("scene/removeRobotFromScene");

    // print info on exit
    std::cout << "Pozvan destruktor: " << name(InsertSceneVoxels) << std::endl;
}

void InsertSceneVoxels::registerKinect(std::string kinectName1, std::string kinectTopicName1, std::string targetFrame)
{
    // already init and empty name
    if(this->kinectNum != 0)
    {
        ROS_WARN("Kinect/i za scenu je/su vec registriran/i. Novi Kinect/i NECE biti registriran/i!");
        return;
    } 
    else if(kinectName1.empty())
    {
         ROS_WARN("Ime Kinect-a ne moze biti prazno. Novi Kinect NECE biti registriran!");
        return;
    }

    // save kinect number and tagret frame
    this->kinectNum = 1;
    this->targetFrame = "base_link"; //targetFrame;

    // subscribe to callback for ONE kinect (queue = kinectQueue1 !!)
    this->n.setCallbackQueue(&(this->kinectQueue1));
    this->sub1 = this->n.subscribe(kinectTopicName1, 1, &InsertSceneVoxels::kinectCallback, this);
    this->n.setCallbackQueue(NULL);
        
    // create Gpu-Voxels probability map
    this->oneKinectMapName = kinectName1 + "VoxelMap";
    gvl->addMap(MT_PROBAB_VOXELMAP, this->oneKinectMapName);
    this->voxMapTemp = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(gvl->getMap(this->oneKinectMapName));

    // create Gpu-Voxels map for output
    this->oneKinectOutMapName = "oneKinectVoxelMap" ;
    gvl->addMap(MT_PROBAB_VOXELMAP, this->oneKinectOutMapName);
    this->kinectVoxMapMerged = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(gvl->getMap(this->oneKinectOutMapName));

    // VoxelCloud
    this->pc.reset(new PointCloud);
        
    // info
    this->kinect1Name = kinectName1;
    this->kinect1TopicName = kinectTopicName1;
    this->np.setParam("scene/kinectNum", this->kinectNum);
    ROS_INFO_STREAM("Registriran kinect \"" << kinectName1 << "\" na topic-u \"" << kinectTopicName1 << "\".");
    ROS_INFO_STREAM("Kreirana MT_PROBAB_VOXELMAP: " << this->oneKinectOutMapName);
}

void InsertSceneVoxels::registerKinects(std::string kinectName1, std::string kinectTopicName1, std::string kinectName2, 
                                                                       std::string kinectTopicName2, std::string targetFrame)
{   
    // already init and empty name
    if(this->kinectNum != 0)
    {
        ROS_WARN("Kinect/i za scenu je/su vec registriran/i. Novi Kinect/i NECE biti registriran/i!");
        return;
    } 
    else if(kinectName1.empty() || kinectName2.empty())
    {
         ROS_WARN("Ime Kinect-a ne moze biti prazno. Novi Kinect-i NECE biti registrirani!");
        return;
    }

    // save kinect num and tagret frame
    this->kinectNum = 2;
    this->targetFrame = targetFrame;
    
    // subscribe to callback for first kinect (queue = kinectQueue1 !!)
    this->n.setCallbackQueue(&(this->kinectQueue1));
    this->sub1 = this->n.subscribe(kinectTopicName1, 1, &InsertSceneVoxels::kinectCallback1, this);

    // subscribe to callback for second kinect (queue = kinectQueue2 !!)
    this->n.setCallbackQueue(&(this->kinectQueue2));
    this->sub2 = this->n.subscribe(kinectTopicName2, 1, &InsertSceneVoxels::kinectCallback2, this);
        
    // create Gpu-Voxels probability map 
    this->twoKinectsMapName = kinectName1 + kinectName2 + "VoxelMap";
    gvl->addMap(MT_PROBAB_VOXELMAP, this->twoKinectsMapName);
    this->voxMapTemp = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(gvl->getMap(this->twoKinectsMapName));

    // create Gpu-Voxels map for MERGED kinects (output map)
    this->twoKinectsOutMapName = "twoKinectsVoxelMap" ;
    gvl->addMap(MT_PROBAB_VOXELMAP, this->twoKinectsOutMapName);
    this->kinectVoxMapMerged = boost::dynamic_pointer_cast<voxelmap::ProbVoxelMap>(gvl->getMap(this->twoKinectsOutMapName));

    // VoxelClouds
    this->pc.reset(new PointCloud);
    this->pc1.reset(new PointCloud);
    this->pc2.reset(new PointCloud);

    // info
    this->kinect1Name = kinectName1;
    this->kinect1TopicName = kinectTopicName1;
    this->kinect2Name = kinectName2;
    this->kinect2TopicName = kinectTopicName2;
    this->np.setParam("scene/kinectNum", this->kinectNum);
    ROS_INFO_STREAM("Registriran kinect \"" << kinectName1 << "\" na topic-u \"" << kinectTopicName1 << "\".");
    ROS_INFO_STREAM("Registriran kinect \"" << kinectName2 << "\" na topic-u \"" << kinectTopicName2 << "\"."); 
    ROS_INFO_STREAM("Kreirana MT_PROBAB_VOXELMAP: " << this->twoKinectsOutMapName);

    // return node handle to global queue  
    this->n.setCallbackQueue(NULL);                                                        
}

void InsertSceneVoxels::removeRobotFromScene(InsertRobotVoxels *robot, bool filter, float occupancyThreshold, float fillThreshold)
{   
    if(this->removeRobot)
    {
        ROS_WARN("Rezanje robota iz scene je vec ukljuceno.");
    }
    else if(robot->isInitialized())
    {
        this->removeRobot = true;
        this->robot = robot;
        this->filter = filter;
        this->occupancyThreshold = occupancyThreshold;
        this->fillThreshold = fillThreshold;
        np.setParam("scene/removeRobotFromScene", this->removeRobot);
        ROS_INFO("Rezanje robota iz scene: ukljuceno");
    }
    else
    {
        ROS_WARN("Robot nije inicijaliziran. Robot nece biti izrezan iz scene!");
    }  
}

bool InsertSceneVoxels::startSceneSpinners()
{
    if(this->kinectNum == 1)
    {
        // start async spinner for ONE kinect
        this->spinner1.start();
        return true;
    }
    else if(this->kinectNum == 2)
    {
        // start async spinner for first kinect
        this->spinner1.start();

        // start async spinner for second kinect
        this->spinner2.start(); 
        return true;
    } 
    else
    {
        ROS_WARN("Niti jedan kinect NIJE registriran! Kreiranje scene (async spinner-i) nece biti pokrenuto!");
        return false;
    }
     
}

void InsertSceneVoxels::stopSceneSpinners()
{
    if(this->kinectNum == 1)
        this->spinner1.stop();
    else if(this->kinectNum == 2)
    {
        this->spinner1.stop();
        this->spinner2.stop(); 
    } 
}

std::string InsertSceneVoxels::getSceneVoxelMapName()
{
    if(this->kinectNum == 1)
        return this->oneKinectOutMapName;
    else if(this->kinectNum == 2)
        return this->twoKinectsOutMapName;
    else
        return "";    
}

InsertSceneVoxels::ProbVoxelMapPtr InsertSceneVoxels::getSceneVoxelMap()
{
    return ((this->kinectNum != 0) ? this->kinectVoxMapMerged : nullptr);
}

void InsertSceneVoxels::lockThread()
{
    this->mtx.lock();
}

void InsertSceneVoxels::unlockThread()
{
    this->mtx.unlock();
}

void InsertSceneVoxels::visualizeScene()
{
    if(this->kinectNum == 0)
    {
        ROS_WARN("Niti jedan kinect nije registriran. Scena nece biti vizualizirana!");
        return;
    }

    // lock method
    boost::lock_guard<boost::mutex> guard(this->mtx);
    gvl->visualizeMap(this->getSceneVoxelMapName());
}

Eigen::Affine3d InsertSceneVoxels::getTransformationMatrix(std::string targetFrame, std::string currentFrame)
{
    geometry_msgs::TransformStamped tfStamped;
    Eigen::Affine3d transformationMatrix = Eigen::Affine3d::Identity();

    try 
    {
        tfStamped = this->tfBuff.lookupTransform(targetFrame, currentFrame, ros::Time(0));
    }
    catch(tf2::TransformException &e) 
    {
        ROS_WARN_STREAM("Ne mogu dohvatiti TF iz " << currentFrame << " u " << targetFrame <<": " << e.what());
        return transformationMatrix;
    }

    tf::transformMsgToEigen(tfStamped.transform, transformationMatrix);
    return transformationMatrix;
}

gpu_voxels::Matrix4f InsertSceneVoxels::eigenTfMatrixToGpuVoxelsTfMatrix(Eigen::Affine3d tfMatrix, Eigen::Vector3f baseOffset)
{   
    Eigen::Vector3f eulerRPY;
    Eigen::Vector3f translation;
    Matrix3f rot;
    Vector3f trans;

    // cast to float
    eulerRPY = tfMatrix.rotation().eulerAngles(2, 1, 0).cast<float>();
    translation = tfMatrix.translation().cast<float>();

    // translation vector in Gpu-Voxels
    trans.x = translation.x() + baseOffset.x();
    trans.y = translation.y() + baseOffset.y();
    trans.z = translation.z() + baseOffset.z();

    // rotation matrix in Gpu-Voxels
    rot = Matrix3f::createFromRPY(eulerRPY(2), eulerRPY(1), eulerRPY(0));

    return Matrix4f::createFromRotationAndTranslation(rot, trans);
}

bool InsertSceneVoxels::deserializationPointCloudVoxels(sensor_msgs::PointCloud2ConstPtr msg, std::vector<gpu_voxels::Vector3f> &points)
{
    int j, k;
    float *f;
    uint32_t x, y, z, fullWord[3], i, b0, b1, b2, b3;

    // get offsets (only for xyz) from start of point structure and check datatype (only float32 supported)
    for(k = 0; k < msg->fields.size(); k++)
    {
        if(msg->fields[k].name == "x")
        {   
            // x offset
            x = msg->fields[k].offset;

            // check data type (same for y and z)
            if(msg->fields[k].datatype != msg->fields[k].FLOAT32)
                return false;
        }
        else if(msg->fields[k].name == "y")
        {
            // y offset
            y = msg->fields[k].offset;
        }
        else if(msg->fields[k].name == "z")
        {
            // z offset
            z = msg->fields[k].offset;
        }
    }

    // endianness
    if(msg->is_bigendian)
    {
        // bytes position
        b0 = 3; b1 = 2; b2 = 1; b3 = 0;   
    }
    else
    {
        // bytes position
        b0 = 0; b1 = 1; b2 = 2; b3 = 3;  
    }

    // set size of output vector to size of PointCloud
    points.resize(msg->height * msg->width);
    
    // deserialization of data
    for(i = 0, k = 0; i < msg->data.size(); i += msg->point_step, k++)
    {   
        // get raw value of PointXYZ
        fullWord[0]=(msg->data[i+x+b3] << 24) | (msg->data[i+x+b2] << 16) | (msg->data[i+x+b1] << 8) | msg->data[i+x+b0];
        fullWord[1]=(msg->data[i+y+b3] << 24) | (msg->data[i+y+b2] << 16) | (msg->data[i+y+b1] << 8) | msg->data[i+y+b0];
        fullWord[2]=(msg->data[i+z+b3] << 24) | (msg->data[i+z+b2] << 16) | (msg->data[i+z+b1] << 8) | msg->data[i+z+b0];
    
        // convert to float
        f = (float *)fullWord;
        points[k].x = *f;
        points[k].y = *(f + 1);
        points[k].z = *(f + 2);
    }

    return true;
}

void InsertSceneVoxels::updateSceneMap()
{
    // update robots pose and map
    if(this->removeRobot)
        robot->updateRobotMap();
        
    // clear temp map
    this->voxMapTemp->clearMap();

    // remove robot from VoxelCloud?
    if (this->removeRobot)
    {  
        // remove robot
        this->lockThread();
        this->voxMapTemp->insertSensorData(*(this->pc), Vector3f(), true, true, eBVM_OCCUPIED, this->robot->getRobotVoxelMap()->getDeviceDataPtr());
        this->unlockThread();

        // filter
        if(this->filter)
            this->voxMapTemp->erodeInto(*(this->kinectVoxMapMerged), this->fillThreshold, this->occupancyThreshold);
        else
            this->kinectVoxMapMerged->clone(*(this->voxMapTemp));
    }                                                                    
    else 
    {
        // insert VoxelCloud into map  
        this->mtx.lock();
        this->voxMapTemp->insertPointCloud(*(this->pc), eBVM_OCCUPIED); 
        this->mtx.unlock();

        // to out map
        this->kinectVoxMapMerged->clone(*(this->voxMapTemp));
    }   
}

void InsertSceneVoxels::kinectCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{   
    std::vector<Vector3f> points;
    Eigen::Affine3d transMatrixEigen;
    Matrix4f transMatrix;

    // deserialization
    if(!this->deserializationPointCloudVoxels(msg, points))
    {   
        ROS_WARN("Deserijalizacija PointCloud-a nije uspjela! PointXYZ mora biti tipa FLOAT32");
        return;
    }

    // trasnformation for voxelized PointCloud
    transMatrixEigen = this->getTransformationMatrix(this->targetFrame, msg->header.frame_id);
    transMatrix = this->eigenTfMatrixToGpuVoxelsTfMatrix(transMatrixEigen, this->originOffset);

    // create voxelized PointCloud 
    this->lockThread();
    this->pc->update(points);
    this->pc->transformSelf(&transMatrix);
    this->unlockThread();
}

void InsertSceneVoxels::kinectCallback1(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::vector<Vector3f> points;
    Eigen::Affine3d transMatrixEigen;
    Matrix4f transMatrix;

    // deserialization
    if(!this->deserializationPointCloudVoxels(msg, points))
    {   
        ROS_WARN("Deserijalizacija PointCloud-a nije uspjela! PointXYZ mora biti tipa FLOAT32");
        return;
    }

    // trasnformation for voxelized PointCloud
    transMatrixEigen = this->getTransformationMatrix(this->targetFrame, msg->header.frame_id);
    transMatrix = this->eigenTfMatrixToGpuVoxelsTfMatrix(transMatrixEigen, this->originOffset);

    // create voxelized PointCloud 
    this->lockThread();
    this->pc1->update(points);
    this->pc1->transformSelf(&transMatrix);
    this->pc->update(this->pc1.get());
    this->pc->add(this->pc2.get());
    this->unlockThread();
}

void InsertSceneVoxels::kinectCallback2(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::vector<Vector3f> points;
    Eigen::Affine3d transMatrixEigen;
    Matrix4f transMatrix;

    // deserialization
    if(!this->deserializationPointCloudVoxels(msg, points))
    {   
        ROS_WARN("Deserijalizacija PointCloud-a nije uspjela! PointXYZ mora biti tipa FLOAT32");
        return;
    }

    // trasnformation for voxelized PointCloud
    transMatrixEigen = this->getTransformationMatrix(this->targetFrame, msg->header.frame_id);
    transMatrix = this->eigenTfMatrixToGpuVoxelsTfMatrix(transMatrixEigen, this->originOffset);

    // create voxelized PointCloud 
    this->lockThread();
    this->pc2->update(points);
    this->pc2->transformSelf(&transMatrix);
    this->pc->update(this->pc2.get());
    this->pc->add(this->pc1.get());
    this->unlockThread();
}