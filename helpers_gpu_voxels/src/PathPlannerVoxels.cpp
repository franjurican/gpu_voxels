// header for path_planner_voxels
#include <helpers_gpu_voxels/path_planner_voxels.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// ROS
#include <ros/ros.h>

std::vector<Eigen::Vector3f> path_planner_voxels::getMinimumOrientedBoundingBox(bb_cuda::VoxelCloudPtr voxelCloud, Eigen::Affine3f &transformMatrix)
{

    // local vars
    float minX, maxX, minY, maxY, minZ, maxZ;
    std::vector<Eigen::Vector3f> boundingBox;
    Eigen::Vector4f centorid;
    Eigen::Matrix3f covariance, eigenVectors;
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);

    // check VoxelCloud size
    if(voxelCloud->size() < 10)
    {
        ROS_ERROR("Velicina VoxelCloud-a je manja od 10 voxela. Izlazim iz trazenja bounding box-a!");
        return boundingBox;
    }
  
    // convert VoxelCloud into PointCloud
    for(auto i = voxelCloud->begin(); i != voxelCloud->end(); i++)
    {
        point.x = i->x();
        point.y = i->y();
        point.z = i->z();
        cloud->push_back(point);
    }

    // get centorid and covariance matrix of object (needed for translation and rotation)
    pcl::compute3DCentroid(*cloud, centorid);
    pcl::computeCovarianceMatrixNormalized(*cloud, centorid, covariance);

    // find eigen vectors of covariance matrix (self-adjoint -> adj(A) = A)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> matrixSolver(covariance, Eigen::ComputeEigenvectors);
    eigenVectors = matrixSolver.eigenvectors();
 
    // x axis > z axis > y axis - eigenvalues!!
    Eigen::Vector3f test = eigenVectors.col(0);
    eigenVectors.col(0) = eigenVectors.col(2);
    eigenVectors.col(1) = test;

    // x in positive y direction
    if(eigenVectors.col(0).y() < 0)
        eigenVectors.col(0) = - eigenVectors.col(0);

    // secure orthogonality ("right-handed" CS) of eigen vectors
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));

    // DON'T flip object (here can be defined and other restrictions!!)
    if(eigenVectors.col(2).z() < 0)
    {
        eigenVectors.col(1) = - eigenVectors.col(1);
        eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    }

    // calculate transformation matrix of object
    transformMatrix = Eigen::Affine3f::Identity();
    transformMatrix.translate(centorid.head<3>());
    transformMatrix.rotate(eigenVectors);
   
    // transform VoxelCloud to origin and find MINIMUM bounding box (pcl built in function is VERY SLOW!!)
    pcl::transformPointCloud(*cloud, *cloudOrigin, transformMatrix.inverse(Eigen::TransformTraits::Affine));

    minX = cloudOrigin->points[0].x;    maxX = cloudOrigin->points[0].x;
    minY = cloudOrigin->points[0].y;    maxY = cloudOrigin->points[0].y;
    minZ = cloudOrigin->points[0].z;    maxZ = cloudOrigin->points[0].z;

    for(int i =1; i < cloudOrigin->size(); i++)
    {
        point = cloudOrigin->points[i];
        minX = (point.x < minX) ? point.x : minX;   maxX = (point.x > maxX) ? point.x : maxX;
        minY = (point.y < minY) ? point.y : minY;   maxY = (point.y > maxY) ? point.y : maxY;
        minZ = (point.z < minZ) ? point.z : minZ;   maxZ = (point.z > maxZ) ? point.z : maxZ;
    }

    // minimum bounding box around origin!
    boundingBox.push_back(Eigen::Vector3f(minX, minY, minZ));
    boundingBox.push_back(Eigen::Vector3f(maxX, maxY, maxZ));  

    return boundingBox;
}

std::vector<Eigen::Vector3f> path_planner_voxels::generatePathWaypoints(std::vector<Eigen::Vector3f> bb, Eigen::Affine3f transformMatrix, float interpolation, float distance)
{
    Eigen::Vector3f min, max, point;
    float stepSize;
    int numberOfPoints;
    std::vector<Eigen::Vector3f> waypoints;

    // check BB size
    if(bb.size() < 2)
    {
        ROS_ERROR("Premali bounding box oko ishodista. Izlazim iz generiranja waypoints-a!");
        return waypoints;
    }

    // min-max point
    min = bb[0];
    max = bb[1];

    // step size for interpolation
    numberOfPoints = std::ceil((max.x() - min.x())/interpolation);
    stepSize = (max.x() - min.x())/numberOfPoints;

    // resize vector to number of points
    waypoints.resize((numberOfPoints + 1)*3);

    for(int i = 0; i <= numberOfPoints; i++)
    {   
        point = Eigen::Vector3f(min.x() + stepSize*i, min.y() - (max.z() + distance), max.z() + distance);
        waypoints[i] = transformMatrix * point;

        point =  Eigen::Vector3f(max.x() - stepSize*i, 0, max.z() + distance);
        waypoints[i + numberOfPoints + 1] = transformMatrix * point;

        point = Eigen::Vector3f(min.x() + stepSize*i, max.y() + max.z() + distance, max.z() + distance);
        waypoints[i + (numberOfPoints + 1)*2] = transformMatrix * point;
    }

    return waypoints;
}

std::vector<Eigen::Vector3f> path_planner_voxels::generateAndVisualizeWaypoints(bb_cuda::VoxelCloudPtr voxelCloud, 
                                                      Eigen::Affine3f &transformMatrix, float distance, float interpolation)
{
    // PoinClouds for visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>); 

    // Eigen vectors and transformation matrix
    std::vector<Eigen::Vector3f> boundingBox, waypointsVector;
    Eigen::Vector3f min, max;

    // pcl point
    pcl::PointXYZ point;

    // convert VoxelCloud into PointCloud
    for(auto i = voxelCloud->begin(); i != voxelCloud->end(); i++)
    {
        point.x = i->x();
        point.y = i->y();
        point.z = i->z();
        cloud->push_back(point);
    }

    // get bounding box around origin and transform PointCloud
    boundingBox = path_planner_voxels::getMinimumOrientedBoundingBox(voxelCloud, transformMatrix);

    // check BB
    if(boundingBox.empty())
        return waypointsVector;

    pcl::transformPointCloud(*cloud, *cloudOrigin, transformMatrix.inverse(Eigen::TransformTraits::Affine));

    // generate waypoints and create PC for waypoints
    waypointsVector = path_planner_voxels::generatePathWaypoints(boundingBox, transformMatrix, interpolation, distance);

    // check waypoints
    if(waypointsVector.empty())
        return waypointsVector;
  
    for(auto i = waypointsVector.begin(); i != waypointsVector.end(); i++)
    {
        point.x = i->x();
        point.y = i->y();
        point.z = i->z();
        waypoints->push_back(point);
    }

    // visualize - for testing
    pcl::visualization::PCLVisualizer viewer("VoxelCloud - Bounding box (PCA analysis) and waypoints");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 20, 225, 20);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloudOrigin, 225, 20, 20);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(waypoints, 10, 10, 255);

    // send VoxelClouds to visualizer
    viewer.addPointCloud(cloud, color1, "voxelCloud");
    viewer.addPointCloud(cloudOrigin, color2, "voxelCloudOrigin");
    viewer.addPointCloud(waypoints, color3, "waypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "voxelCloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "voxelCloudOrigin");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "waypoints");

    // add bounding box
    min = boundingBox[0];
    max = boundingBox[1];
    viewer.addCube(min.x(), max.x(), min.y(), max.y(), min.z(), max.z(), 1.0, 1.0, 1.0, "BBorigin");
    //viewer.addCube(transformMatrix.translation().head<3>(), Eigen::Quaternionf(transformMatrix.rotation()), 
    //                max.x() - min.x(), max.y() - min.y(), max.z() - min.z(), "BBreal");

    // other visualization options
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05);
    viewer.addText("Green - VoxelCloud of object", 10, 65, 15, 20, 20, 245, "prvi");
    viewer.addText("Red - VoxelCloud at origin", 10, 50, 15, 20, 20, 245, "drugi");
    viewer.addText("Blue - Waypoints", 10, 35, 15, 20, 20, 245, "treci");
    viewer.addText("White - Bounding box", 10, 20, 15, 20, 20, 245, "cetvrti");
  
    // start visualizer
    while (!viewer.wasStopped())
        viewer.spinOnce();

    viewer.close();

    return waypointsVector;
}