#ifndef VISUALIZER_H
#define VISUALIZER_H

// PCL core functionality
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/vtk_lib_io.h> // For loading STL files

// PCL visualization
#include <pcl/visualization/pcl_visualizer.h>

// PCL processing

// #include <pcl/common/transforms.h>
// #include <pcl/filters/voxel_grid.h>

// #include <opencv2/opencv.hpp> // Include OpenCV for cv::Mat and cv::Point

// Application-specific headers

class visualizer{
public:
    void run();

    visualizer(const std::string& folderPath);

    // static pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filename);

    static void setCameraPositionBasedOnBoundingBox(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static void visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudVector);
    void translateToPointCloud(const std::vector<std::vector<std::array<double, 4>>>& dicomData, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_dicom);

    void meshVTK();
    void saveCloudWithNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);

private:
    std::string folder;
    std::vector<std::vector<std::array<double, 4>>> voxelVector;
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloud;

};


#endif // VISUALIZER_H
