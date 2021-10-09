#ifndef KINECT_REGISTRATION_H
#define KINECT_REGISTRATION_H

// C++ STL
#include <iostream>
#include <string>
#include <map>
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Eigen
#include <Eigen/Geometry>

class KinectRegistration 
{   
    public:
        /* 
            Konstruktor za registraciju Kinect-a.
            \param kinectPCD ime PCD file-a koji sadrzi PointCloud s Kinect-a
            \param modelPCD ime PCD file-a koji sadrzi PointCloud modela robota
            \param savePath lokacija foldera u kojem se nalaze PCD file-ovi, ako parametar nije 
            zadan pretrazuje se folder iz kojeg je program pozvan (mora zavrsavati s '/' !!)
        */
        KinectRegistration(std::string kinectPCD, std::string modelPCD, std::string savePath = "");

        /* 
            Konstruktor za registraciju Kinect-a.
            \param kinectPC PointCloud s Kinect-a
            \param modelPC PointCloud modela robota
        */
        KinectRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &kinectPC, pcl::PointCloud<pcl::PointXYZ>::Ptr &modelPC);

        /*
            Postavi procjenu udaljenosti Kinect-a do robota (radi ubrzavanja algoritma). Opcionalni parametar!
            \param distance2Robot udaljenost Kinect-a do robota (procijenjena udaljenost)
        */
        void setDistanceToRobot(float distance2Robot);

        /* 
            Postavi velicinu voxela za 'VoxelGrid' PointCloud filter.
            \param voxelSize velicina voxela
        */
        void setVoxelSize(float voxelSize);

        /* 
            Postavi radijus sfere za pretrazivanje susjednih tocaka unutar PointCloud-a prilikom estimacije normala.
            \param searchRadius radijus pretrazivanja
        */
        void setRadiusForNormals(float searchRadius);

        /* 
            Postavi radijus sfere za pretrazivanje susjednih tocaka unutar PointCloud-a prilikom trazenja znacajki (FPFH).
            \param searchRadius radijus pretrazivanja
        */
        void setRadiusForFPFH(float searchRadius);

        /* 
            Postavi broj iteracija za pronalazak robota u PointCloud-u Kinect-a.
            \param maxIterations maksimalni broj iteracija 
        */
        void setMaxIterationsRobotDetection(int maxIterations);

        /* 
            Postavi slucajnost prilikom odabira tocaka (znacajki) za pronalazak robota u PointCloud-u Kinect-a.
            \param rand veci broj predstavlja vecu slucajnost kod odabira znacajki
        */
        void setRandomnessRobotDetection(int rand);

        /* 
            Postavi granicu slicnosti izmedu dvije znacajke na PointCloud-u modela robota i PointCloud-u Kinect-a.
            \param threshold slicnost znacajki (vrijednost iz intervala [0, 1])
        */
        void setSimilarityThresholdRobotDetection(float threshold);

        /* 
            Postavi maksimalnu udaljenost izmedu dvije znacajke na PointCloud-u modela robota i PointCloud-u Kinect-a. 
            Ako su tocke udaljenije od ove vrijednosti, tada ce tocke biti ignorirane u procesu trazenja robota u PointCloud-u Kinect-a.
            \param maxDistance udaljenost znacajki
        */
        void setMaxDistanceRobotDetection(float threshold);

        /* 
            Postavi minimalni postotak preklapanja PointCloud-a modela robota i PointCloud-a Kinect-a. Ovaj parametar predstavlja procjenu 
            vidljivosti robota u PointCloud-u Kinect-a, odnosno koliki je dio robota vidljiv u PointCloud-u Kinect-a.
            \param threshold postotak vidljivosti robota (vrijednost iz intervala [0, 1])
        */
        void setVisibilityPercantageRobotDetection(float threshold);

        /* 
            Postavi broj iteracija za 'fino poravnavanje' robota u PointCloud-u Kinect-a.
            \param maxIterations maksimalni broj iteracija 
        */
        void setMaxIterationsICP(int maxIterations);

        /* 
            Postavi broj RANSAC iteracija za 'fino poravnavanje' robota u PointCloud-u Kinect-a.
            \param maxIterations maksimalni broj RANSAC iteracija 
        */
        void setMaxRANSACIterationsICP(int maxRANSACIterations);

        /*
            Zapocni postupak registracije Kinect-a. Funkcija blokira ukoliko je ukljucena vizualizacija!
            \param visualize vizualiziraj rezultat registracije Kinect-a
        */
        void startRegistration(bool visualize = false);

        /*
            Vrati matricu transformacije Kinect-a u koordinatni sustav baze robota.
            \return matrica transformacije Kinect->Baza. U slucaju divergencije algoritma registracije Kinect-a vraca jedinicnu matricu!
        */
        Eigen::Matrix4f getTransformationMatrix();
        
        /* Ispis informacija o transformaciji Kinect-a u koordinatni sustav baze robota. */
        void printKinectTransformationInfo();

        /* Ispisi sve parametre za registraciju Kinect-a. */
        void printAllParameters();

        /* Ispisi 'benchmark' informacije */
        void printBenchmarkInfo();
        
    private:
        typedef pcl::PointCloud<pcl::PointXYZ> PC_XYZ;
        typedef pcl::PointCloud<pcl::PointNormal> PC_XYZN;
        typedef pcl::PointCloud<pcl::FPFHSignature33> PC_FPFH;
        typedef pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> SCP_XYZN;
        typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICP_XYZ;

        /* Filtriranje ulaznog PointCloud-a s Kinect-a (brisanje NaN point-a i point-a koji su iza robota ukoliko je parametar distanceToRobot postavljen!) */
        void filterKinectPointCloud();

        /* Downsampling ulaznih PointCloud-ova */
        void voxelGridFilter();

        /* Izracunaj normale PointCloud-ova Kinect-a i modela robota */
        void calculateNormals();

        /* Izracunaj znacajke PointCloud-ova Kinect-a i modela robota */
        void calculateFeatures();

        /* Trazi model robota u PointCloud-u Kinect-a */
        void findRobot(SCP_XYZN &find);

        /* Pokreni ICP samo ako trazenje robota KONVERGIRA! */
        void startICP(SCP_XYZN &find, ICP_XYZ &icp);

        /* Kreiranje VTK-a vizualizatora. Funkcija blokira! */
        void visualizePointClouds();

        /* Pokreni timer */
        void tic();

        /* Spremi vriujednost timera */
        void toc(std::string name);

        int maxIterSCP_, randomnessSCP_, sampleSCP_, maxIterICP_, maxRANSACIterICP_;
        float robotDistance_, voxelSize_, searchRadiusNormals_, searchRadiusFPFH_, similaritySCP_, distanceSCP_, visibilitySCP_;
        double fitnessEpsilonICP_, transformationEpsilonICP_;
        PC_XYZ::Ptr kinect, model, filteredKinect, icpIn, icpOut;
        PC_XYZN::Ptr kinectNormal, modelNormal, scpOut;
        PC_FPFH::Ptr kinectFPFH, modelFPFH;
        Eigen::Matrix4f transformationMatrix;
        std::chrono::system_clock::time_point t1, t2;
        std::map<std::string, std::chrono::milliseconds> time;
};
#endif // KINECT_REGISTRATION_H