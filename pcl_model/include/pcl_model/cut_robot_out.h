#ifndef CUT_ROBOT_OUT_H
#define CUT_ROBOT_OUT_H

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/area_picking_event.h>

class CutRobotOut {
    public:
        /* 
            Konstruktor za klasu koja kreira GUI za rezanje robota iz scene.
            \param savePath folder u koji se spremlja .pcd file
            \param name ime .pcd file-a iz kojega se zeli izrezati robot
        */
        CutRobotOut(std::string savePath, std::string name);

        /* Pokreni GUI. Nakon izlaska iz GUI-a, automatski ce biti spremljen izrezani robot u folder zadan u konstruktoru! */
        void start();

    private:
        typedef pcl::PointCloud<pcl::PointXYZ> PC_XYZ;

        PC_XYZ::Ptr outputPC, inputPC;
        bool areaSelectedAndValid;
        std::string savePath;

        void callback(const pcl::visualization::AreaPickingEvent &event, void *viewer);
};

#endif // CUT_ROBOT_OUT_H