#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <includes.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

// Application-specific headers

class visualizer{
public:
    void run(std::string& patientName, bool& firstInit, bool saveData);

    visualizer(const std::string& folderPath);

    // static pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filename);

    static void setCameraPositionBasedOnBoundingBox(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static void visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudVector, std::vector<float>& pcdInfo, std::string& patientName);
    void translateToPointCloud(const std::vector<std::vector<std::array<double, 4>>>& dicomData, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_dicom);

    void meshVTK();
    void saveCloudWithNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);

    static void outputData(const std::string& patientName, const std::vector<float>& pcdInfo, bool& firstInit);

private:
    std::string folder;
    std::vector<std::vector<std::array<double, 4>>> voxelVector;
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloud;

};


#endif // VISUALIZER_H
