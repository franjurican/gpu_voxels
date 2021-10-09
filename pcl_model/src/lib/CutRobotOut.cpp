#include <pcl_model/cut_robot_out.h>

CutRobotOut::CutRobotOut(std::string savePath, std::string name) : savePath(savePath), inputPC(new PC_XYZ), outputPC(new PC_XYZ), areaSelectedAndValid(false)
{
    // ucitaj .pcd file
    pcl::io::loadPCDFile(this->savePath + name, *inputPC);
}

void CutRobotOut::start()
{
    int v1 = 0, v2 = 1;
    pcl::visualization::PCLVisualizer viewer("Cut robot");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorGreen(inputPC, 20, 180, 20);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorBlue(inputPC, 20, 20, 180);

    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);

    viewer.addCoordinateSystem(1, "cs1", v1);
    viewer.addCoordinateSystem(1, "cs2", v2);

    viewer.registerAreaPickingCallback(&CutRobotOut::callback, *this);

    viewer.addPointCloud(inputPC, colorGreen, "input", v1);
    viewer.addPointCloud(outputPC, colorBlue, "output", v2);

     while(!viewer.wasStopped())
     {  
        if(this->areaSelectedAndValid)
        {
            std::cout << "Selected area visualized" << std::endl;
            viewer.updatePointCloud(outputPC, colorBlue, "output");
            this->areaSelectedAndValid = false;
        }
        viewer.spinOnce();
    }

    pcl::io::savePCDFileBinary(this->savePath + "robotCut.pcd", *outputPC);
    std::cout << "PCD file: " + this->savePath + "robotCut.pcd" << std::endl;
}

void CutRobotOut::callback(const pcl::visualization::AreaPickingEvent &event, void *viewer)
{
    pcl::PointIndices::Ptr area(new pcl::PointIndices);

    if(event.getPointsIndices(area->indices))
    {   
        std::cout << "Area selected!" << std::endl;
        this->areaSelectedAndValid = true;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(inputPC);
        extract.setIndices(area);
        extract.filter(*outputPC);
        std::cout << "Area PointCloud size:" << outputPC->size() << std::endl;
    }
    else
        std::cout << "Selected area is not valid!" << std::endl;
}