#include <pcl_model/kinect_registration.h>

// log za terminal
#define DEVELOPMENT_LOG_KINECT 1
#define LOG_KINECT_REG(msg) if(DEVELOPMENT_LOG_KINECT) std::cout << "[Registracija kinecta] " << msg << std::endl;

KinectRegistration::KinectRegistration(std::string kinectPCD, std::string modelPCD, std::string savePath) : 
    kinect(new PC_XYZ), model(new PC_XYZ), filteredKinect(new PC_XYZ), icpIn(new PC_XYZ), icpOut(new PC_XYZ), 
    kinectNormal(new PC_XYZN), modelNormal(new PC_XYZN), scpOut(new PC_XYZN), kinectFPFH(new PC_FPFH), modelFPFH(new PC_FPFH)
{
    // ucitaj PointCloud-ove
    pcl::io::loadPCDFile(savePath + kinectPCD, *kinect);
    pcl::io::loadPCDFile(savePath + modelPCD, *model);

    // postavi pocetne vrijednosti parametara
    this->robotDistance_ = -1;
    this->voxelSize_ = 0.005;
    this->searchRadiusNormals_ = 0.01;
    this->searchRadiusFPFH_ = 0.025;
    this->maxIterSCP_ = 100000;
    this->randomnessSCP_ = 5;
    this->similaritySCP_ = 0.8;
    this->distanceSCP_ = 4 * this->voxelSize_;
    this->visibilitySCP_ = 0.2;
    this->maxIterICP_ = 100000;
    this->maxRANSACIterICP_ = 200000;

    // ostali parametri
    this->sampleSCP_ = 3;
    this->fitnessEpsilonICP_ = -1.797e+8;
    this->transformationEpsilonICP_ = 1e-18;
    this->transformationMatrix = Eigen::Matrix4f::Identity();

    // print info
    LOG_KINECT_REG("Velicina ucitanog PointCloud-a Kinect-a: " << this->kinect->size())
    LOG_KINECT_REG("Velicina ucitanog PointCloud-a modela robota: " << this->model->size())
}

KinectRegistration::KinectRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &kinectPC, pcl::PointCloud<pcl::PointXYZ>::Ptr &modelPC) : 
    kinect(new PC_XYZ), model(new PC_XYZ), filteredKinect(new PC_XYZ), icpIn(new PC_XYZ), icpOut(new PC_XYZ), 
    kinectNormal(new PC_XYZN), modelNormal(new PC_XYZN), scpOut(new PC_XYZN), kinectFPFH(new PC_FPFH), modelFPFH(new PC_FPFH)
{
    // spremi PointCloud-ove
    this->kinect = kinectPC;
    this->model = modelPC;

    // postavi pocetne vrijednosti parametara
    this->robotDistance_ = -1;
    this->voxelSize_ = 0.005;
    this->searchRadiusNormals_ = 0.01;
    this->searchRadiusFPFH_ = 0.025;
    this->maxIterSCP_ = 100000;
    this->randomnessSCP_ = 5;
    this->similaritySCP_ = 0.8;
    this->distanceSCP_ = 4 * this->voxelSize_;
    this->visibilitySCP_ = 0.2;
    this->maxIterICP_ = 100000;
    this->maxRANSACIterICP_ = 200000;

    // ostali parametri
    this->sampleSCP_ = 3;
    this->fitnessEpsilonICP_ = -1.797e+8;
    this->transformationEpsilonICP_ = 1e-18;
    this->transformationMatrix = Eigen::Matrix4f::Identity();

    // print info
    LOG_KINECT_REG("Velicina ucitanog PointCloud-a Kinect-a: " << this->kinect->size())
    LOG_KINECT_REG("Velicina ucitanog PointCloud-a modela robota: " << this->model->size())
}

void KinectRegistration::setDistanceToRobot(float distance2Robot)
{
    this->robotDistance_ = distance2Robot;
}

void KinectRegistration::setVoxelSize(float voxelSize)
{
    this->voxelSize_ = voxelSize;
}

void KinectRegistration::setRadiusForNormals(float searchRadius)
{
    this->searchRadiusNormals_ = searchRadius;
}

void KinectRegistration::setRadiusForFPFH(float searchRadius)
{
    this->searchRadiusFPFH_ = searchRadius;
}

void KinectRegistration::setMaxIterationsRobotDetection(int maxIterations)
{
    this->maxIterSCP_ = maxIterations; 
}

void KinectRegistration::setRandomnessRobotDetection(int rand)
{
    this->randomnessSCP_ = rand; 
}

void KinectRegistration::setSimilarityThresholdRobotDetection(float threshold)
{
    this->similaritySCP_ = threshold;
}

void KinectRegistration::setMaxDistanceRobotDetection(float threshold)
{
    this->distanceSCP_ = threshold;
}

void KinectRegistration::setVisibilityPercantageRobotDetection(float threshold)
{
    this->visibilitySCP_ = threshold;
}

void KinectRegistration::setMaxIterationsICP(int maxIterations)
{
    this->maxIterICP_ = maxIterations;
}

void KinectRegistration::setMaxRANSACIterationsICP(int maxRANSACIterations)
{
    this->maxRANSACIterICP_ = maxRANSACIterations;
}

void KinectRegistration::startRegistration(bool visualize)
{
    // prvo filtriraj PointCloud s Kinect-a (uklanjanje nedefiniranih tocaka)
    this->filterKinectPointCloud();
    LOG_KINECT_REG("Velicina filtriranog (1. stupanj) PointCloud-a Kinect-a: " << this->filteredKinect->size());

    // downsample PointCloud-ove Kinect-a i modela robota
    this->voxelGridFilter();
    LOG_KINECT_REG("Velicina filtriranog (2. stupanj) PointCloud-a Kinect-a: " << this->filteredKinect->size())
    LOG_KINECT_REG("Velicina filtriranog (2. stupanj) PointCloud-a modela robota: " << this->model->size())

    // kopiraj PointCloud-ove u tip pogodan za trazenje znacajki
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*(this->filteredKinect), *(this->kinectNormal));
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*(this->model), *(this->modelNormal));

    // izracunaj normale i znacajke PointCloud-a Kinect-a i modela robota
    this->calculateNormals();
    this->calculateFeatures();

    // trazi model robota u PointCloud-u robota i zatim pokreni ICP
    SCP_XYZN find;
    ICP_XYZ icp;
    LOG_KINECT_REG("Pokrecem trazenje robota u PointCloud-u Kinect-a ...")
    this->findRobot(find);

    if(find.hasConverged())
    {
        LOG_KINECT_REG("Robot pronaden, pokrecem ICP ...")
        this->startICP(find, icp);
        LOG_KINECT_REG("ICP " << (icp.hasConverged() ? "konvergira" : "divergira"))

        if(icp.hasConverged())
        {
            LOG_KINECT_REG("Konacna tocnost registracije Kinect-a: " << icp.getFitnessScore())
            this->transformationMatrix = (icp.getFinalTransformation() * find.getFinalTransformation()).inverse();
        }
    }
    else
    {
        LOG_KINECT_REG("Robot nije pronaden!")
    }

    // vizualizacija PointCloud-ova
    if(visualize)
        this->visualizePointClouds();  
}

Eigen::Matrix4f KinectRegistration::getTransformationMatrix()
{
    return this->transformationMatrix;
}

void KinectRegistration::printKinectTransformationInfo()
{
    Eigen::Affine3f trans(this->transformationMatrix);
    Eigen::Quaternionf q(trans.rotation());
    Eigen::Vector3f euler = trans.rotation().eulerAngles(2, 1, 0);
    
    std::cout << "Transformacijska matrica: " << std::endl;
    std::cout << this->transformationMatrix << std::endl;
    std::cout << "Translacija: " << trans.translation().transpose() << std::endl;
    std::cout << "Rotacija (quaternion - xyzw): " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    std::cout << "Rotacija (RPY): " << euler.z() << " " << euler.y() << " " << euler.x() << std::endl;
}

void KinectRegistration::printAllParameters()
{
    
}

void KinectRegistration::printBenchmarkInfo()
{
    std::chrono::seconds sec;
    std::chrono::milliseconds mili, ukupno(0);

    std::cout << "Benchmark po komponentama: " << std::endl;

    for(auto it = time.cbegin(); it != time.cend(); it++)
    {   
        ukupno += it->second;
        sec = std::chrono::duration_cast<std::chrono::seconds>(it->second);
        mili = it->second - sec;
        std::cout << it->first << ": " << sec.count() << " [s] i " << mili.count() << " [ms]" << std::endl;
    }

    sec = std::chrono::duration_cast<std::chrono::seconds>(ukupno);
    mili = ukupno - sec;
    std::cout << "Ukupno: " << sec.count() << " [s] i " << mili.count() << " [ms]" << std::endl;
}

void KinectRegistration::filterKinectPointCloud()
{   
    this->tic();
    if(this->robotDistance_ > 0)
    {
        for(PC_XYZ::iterator it = kinect->begin(); it != kinect->end(); it++)
        {
            if(it->z != NAN && it->z > this->robotDistance_)
                it->z = NAN;
        }
    }

    // ukloni nedefinirane tocke
    std::vector<int> pom;
    pcl::removeNaNFromPointCloud(*(this->kinect), *(this->filteredKinect), pom);
    this->toc("Filtriranje 1. stupanj");
}

void KinectRegistration::voxelGridFilter()
{
    this->tic();
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(this->voxelSize_, this->voxelSize_, this->voxelSize_);

    grid.setInputCloud(filteredKinect);
    grid.filter(*(this->filteredKinect));

    grid.setInputCloud(this->model);
    grid.filter(*(this->model));
    this->toc("Filtriranje 2. stupanj");
}

void KinectRegistration::calculateNormals()
{   
    this->tic();
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> normEst;
    normEst.setRadiusSearch(this->searchRadiusNormals_);

    normEst.setInputCloud(this->kinectNormal);
    normEst.compute(*(this->kinectNormal));

    normEst.setInputCloud(this->modelNormal);
    normEst.compute(*(this->modelNormal));
    this->toc("Racunanje normala");
}

void KinectRegistration::calculateFeatures()
{
    this->tic();
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfhEst;
    fpfhEst.setRadiusSearch(this->searchRadiusFPFH_);

    fpfhEst.setInputCloud(this->kinectNormal);
    fpfhEst.setInputNormals(this->kinectNormal);
    fpfhEst.compute(*(this->kinectFPFH));

    fpfhEst.setInputCloud(this->modelNormal);
    fpfhEst.setInputNormals(this->modelNormal);
    fpfhEst.compute(*(this->modelFPFH));
    this->toc("Racunanje znacajki");
}

void KinectRegistration::findRobot(SCP_XYZN &find)
{
    this->tic();
    find.setInputCloud(this->modelNormal);
    find.setSourceFeatures(this->modelFPFH);
    find.setInputTarget(this->kinectNormal);
    find.setTargetFeatures(this->kinectFPFH);

    find.setMaximumIterations(this->maxIterSCP_);
    find.setNumberOfSamples(this->sampleSCP_);
    find.setCorrespondenceRandomness(this->randomnessSCP_);
    find.setSimilarityThreshold(this->similaritySCP_);
    find.setMaxCorrespondenceDistance(this->distanceSCP_);
    find.setInlierFraction(this->visibilitySCP_);
    find.align(*(this->scpOut));
    this->toc("Trazenje robota (SCP)");
}

void KinectRegistration::startICP(SCP_XYZN &find, ICP_XYZ &icp)
{   
    this->tic();
    pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*(this->scpOut), *(this->icpIn));
    icp.setInputCloud(this->icpIn);
    icp.setInputTarget(this->filteredKinect);

    icp.setEuclideanFitnessEpsilon(this->fitnessEpsilonICP_);
    icp.setMaximumIterations(this->maxIterICP_);
    icp.setRANSACIterations(this->maxRANSACIterICP_);
    icp.setTransformationEpsilon(this->transformationEpsilonICP_);
    icp.align(*(this->icpOut));   
    this->toc("Fino tuniranje (ICP)");
}

void KinectRegistration::visualizePointClouds()
{
    int v1 = 0, v2 = 1;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Registracija Kinect-a"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> colorGreen(this->kinectNormal, 20, 180, 20);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> colorBlue(this->modelNormal, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorB(this->icpOut, 0, 0, 255);

    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);

    viewer->addPointCloud(this->kinectNormal, colorGreen, "kinectV1", v1);
    viewer->addPointCloud(this->scpOut, colorBlue, "modelSCP", v1);

    viewer->addPointCloud(this->kinectNormal, colorGreen, "kinectV2", v2);
    viewer->addPointCloud(this->icpOut, colorB, "modelICP", v2);

    while(!viewer->wasStopped())
        viewer->spinOnce();
}

void KinectRegistration::tic()
{
    t1 = std::chrono::high_resolution_clock::now();
}

void KinectRegistration::toc(std::string name)
{
    t2 = std::chrono::high_resolution_clock::now();
    time[name] = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
}


