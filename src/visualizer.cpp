#include "includes.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/io/vtk_io.h>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include "visualizer.h"
#include "rDICOM.h"

visualizer::visualizer(const std::string& folderPath) : folder(folderPath) {
    voxelVector = rDICOM::getDICOMdata(folder);
}

void visualizer::translateToPointCloud(const std::vector<std::vector<std::array<double, 4>>>& dicomData, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_dicom){
    double HuMin = -100.0;
    double HuMax = 100.0;
    // No need to check if the cloud pointer is not null, smart pointers handle this
    // Reserve space for points in the cloud to improve performance

    for (int cloud = 0; cloud < 3; cloud++) {
        cloud_dicom[cloud]->points.reserve(dicomData[cloud].size());
        try {
            for (const auto& data : dicomData[cloud]) {
                // Check if the fourth value is within the specified Hounsfield Unit range
                if (data[3] >= HuMin && data[3] <= HuMax) {
                    // If within range, use the first three values as coordinates for the point cloud
                    pcl::PointXYZ point(static_cast<float>(data[0]), static_cast<float>(data[1]), static_cast<float>(data[2]));
                    cloud_dicom[cloud]->points.push_back(point);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception caught: " << e.what() << std::endl;
        }

        // Update the cloud size based on the actual number of points
        cloud_dicom[cloud]->width = static_cast<uint32_t>(cloud_dicom[cloud]->points.size());
        cloud_dicom[cloud]->height = 1; // This specifies the cloud is unorganized
        cloud_dicom[cloud]->is_dense = false; // Assuming not all points in the cloud might be valid
}
}

void visualizer::saveCloudWithNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pcd);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(15);
    ne.compute(*normals);

    // Concatenate XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*pcd, *normals, *cloud_with_normals);

    pcl::io::savePCDFileASCII("/home/jcvetic/INSPIRATION/Visualize_pointclouds/pointcloud1.pcd", *cloud_with_normals);
}

void visualizer::setCameraPositionBasedOnBoundingBox(pcl::visualization::PCLVisualizer& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    float maxDimension = std::max(maxPt.x - minPt.x, std::max(maxPt.y - minPt.y, maxPt.z - minPt.z));
    float distance = maxDimension * 3;
    viewer.setCameraPosition(centroid[0], centroid[1], centroid[2] + distance,
                             centroid[0], centroid[1], centroid[2], 0, 1, 0);
}


void visualizer::visualizePointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudVector, std::vector<float>& pcdInfo, std::string& patientName) {
    if (patientName.size() > 30){
        size_t pos = patientName.find_last_of('/');
        patientName = patientName.substr(pos + 1);
    }

    std::string savefolder = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/results/" + patientName;
    if (!std::filesystem::exists(savefolder)){
        if(std::filesystem::create_directory(savefolder)){
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud1 = cloudVector[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2 = cloudVector[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud3 = cloudVector[2];

    for (int i=0; i < cloudVector.size(); i++){
        pcdInfo.push_back(cloudVector[i]->size());
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setLeafSize(0.5f, 0.5f, 0.5f); // Set the voxel size

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud3(new pcl::PointCloud<pcl::PointXYZ>());

    voxelFilter.setInputCloud(pointCloud1);
    voxelFilter.filter(*filteredPointCloud1);
    std::cout << "Filtered axial cloud = " << filteredPointCloud1->size() << std::endl;

    voxelFilter.setInputCloud(pointCloud2);
    voxelFilter.filter(*filteredPointCloud2);
    std::cout << "Filtered sagittal cloud = " << filteredPointCloud2->size() << std::endl;


    voxelFilter.setInputCloud(pointCloud3);
    voxelFilter.filter(*filteredPointCloud3);
    std::cout << "Filtered coronal cloud = " << filteredPointCloud3->size() << std::endl;


    pcdInfo.push_back(filteredPointCloud1->size());
    pcdInfo.push_back(filteredPointCloud2->size());
    pcdInfo.push_back(filteredPointCloud3->size());

    std::cout << "Total sum of filtered clouds = " << filteredPointCloud1->size() + filteredPointCloud2->size() + filteredPointCloud3->size() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter_combined;
    voxelFilter_combined.setLeafSize(0.5f, 0.5f, 0.5f); // Set the voxel size

    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *combined_cloud = *filteredPointCloud1 + *filteredPointCloud2+ *filteredPointCloud3;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_nods(new pcl::PointCloud<pcl::PointXYZ>());
    // *combined_cloud_nods = *pointCloud1+*pointCloud2+*pointCloud3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxelFilter_combined.setInputCloud(combined_cloud);
    voxelFilter_combined.filter(*filtered_combined_cloud);
    std::cout << "Filtered combined cloud = " << filtered_combined_cloud->size() << std::endl;

    pcdInfo.push_back(combined_cloud->size());
    pcdInfo.push_back(filtered_combined_cloud->size());

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(filtered_combined_cloud);
    outrem.setRadiusSearch(10);
    outrem.setMinNeighborsInRadius(250);
    outrem.setKeepOrganized(false);
    outrem.filter(*filtered_combined_cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_combined_cloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh(4.0);
    sor.filter(*filtered_combined_cloud);

    // pcl::io::savePCDFileASCII("/home/jcvetic/INSPIRATION/Visualize_pointclouds/filtered_combined_cloud.pcd", *filtered_combined_cloud);
    // pcl::io::savePCDFileASCII(savefolder + "/filteredCC_PCD.pcd", *filtered_combined_cloud);

    pcl::PCLPointCloud2 filteredpc2;
    pcl::toPCLPointCloud2(*filtered_combined_cloud,filteredpc2);
    // pcl::io::saveVTKFile("/home/jcvetic/INSPIRATION/Visualize_pointclouds/filtered_combined_cloud.vtk", filteredpc2);
    // pcl::io::saveVTKFile(savefolder + "/filteredCC_VTK_0_38.vtk", filteredpc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr diffIndices(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(filteredPointCloud1);

    float radius = 1.0;
    for (int i = filtered_combined_cloud->points.size() - 1; i >= 0; --i){
        pcl::PointXYZ search_point = filtered_combined_cloud->points[i];
        std::vector<int> point_idx_search;
        std::vector<float> point_squared_distance;

        if (kdtree.radiusSearch(search_point, radius, point_idx_search, point_squared_distance) == 0){
            diffIndices->points.push_back(search_point);
            filtered_combined_cloud->points.erase(filtered_combined_cloud->begin() + i);
        }
    }

    diffIndices->width = filtered_combined_cloud->width;
    diffIndices->height = filtered_combined_cloud->height;
    diffIndices->is_dense = true;

    pcdInfo.push_back(diffIndices->size());

    // *********************** COMBINED_CLOUD ************************************************************************************************

    // pcl::visualization::PCLVisualizer viewer("Filtered_combined_cloud");
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Filtered_combined_cloud"));
    viewer->setSize(1920/2,1080);
    viewer->setPosition(1920+1920/2,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(filtered_combined_cloud, 255, 255/2, 0); // 255,255/2,0
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color88(diffIndices, 0, 0, 255); // 0,0,255

    viewer->addPointCloud(filtered_combined_cloud, color, "cloud");
    viewer->addPointCloud(diffIndices, color88, "cloud88");
    viewer->setBackgroundColor(0, 0, 0);
    visualizer::setCameraPositionBasedOnBoundingBox(*viewer, filtered_combined_cloud);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud88");

// dodano --- ines crop
    // pcl::CropBox<pcl::PointXYZ> boxfilter;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr croppedcloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr croppeddiff(new pcl::PointCloud<pcl::PointXYZ>);

    // boxfilter.setMin(Eigen::Vector4f(-300.0,125.0,-300.0,1.0));
    // boxfilter.setMax(Eigen::Vector4f(300.0,250.0,300.0,1.0));
    // boxfilter.setInputCloud(filtered_combined_cloud);
    // boxfilter.filter(*croppedcloud);
    // boxfilter.setInputCloud(diffIndices);
    // boxfilter.filter(*croppeddiff);

    // pcl::visualization::PCLVisualizer viewer44("cropped");
    // std::unique_ptr<pcl::visualization::PCLVisualizer> viewer44(new pcl::visualization::PCLVisualizer("cropped"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color44(croppedcloud, 255/2, 255, 255/2); // 255, 0, 0
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color45(croppeddiff, 255/2, 255, 255/2); // 255, 0, 0

    // // roc
    // pcl::PointCloud<pcl::PointXYZ>::Ptr boxfiltered(new pcl::PointCloud<pcl::PointXYZ>());
    // *boxfiltered = *croppedcloud + *croppeddiff;
    // pcl::io::savePCDFileASCII("/home/jcvetic/INSPIRATION/Visualize_pointclouds/cropped.pcd", *boxfiltered);
    // // roc_end

    // viewer44->addPointCloud(croppedcloud, color44, "cloud44");
    // viewer44->addPointCloud(croppeddiff, color45, "cloud45");
    // viewer44->setBackgroundColor(0,0,0);
    // viewer44->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud44");
    // viewer44->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud45");
// ines crop end

// ********************************** AXIAL_VIEW ********************************************************************************************

    // pcl::visualization::PCLVisualizer viewer2("Filtered_axial_view");
    // std::unique_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("Filtered_axial_view"));
    // viewer2->setSize(1920/2,1080);
    // viewer2->setPosition(0,0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(filteredPointCloud1, 255, 0, 0); // 255, 0, 0

    // viewer2->addPointCloud(filteredPointCloud1, color1, "cloud1");

    // viewer2->setBackgroundColor(1, 1, 1);
    // visualizer::setCameraPositionBasedOnBoundingBox(*viewer2, filteredPointCloud1);

    // viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

    // pcl::io::savePCDFileASCII("/home/jcvetic/INSPIRATION/Visualize_pointclouds/pointcloud1.pcd", *filteredPointCloud1);
    // pcl::io::savePCDFileASCII(savefolder + "/axial.pcd", *filteredPointCloud1);

    // ********************************** SAGITAL_VIEW ********************************************************************************************

    // pcl::visualization::PCLVisualizer viewer3("Filtered_sagital_view");
    // std::unique_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Filtered_sagital_view"));
    // viewer3->setSize(1920/2,1080);
    // viewer3->setPosition(1920/2,0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(filteredPointCloud2, 0, 255, 0); // 255, 0, 0

    // viewer3->addPointCloud(filteredPointCloud2, color2, "cloud2");

    // viewer3->setBackgroundColor(255, 255, 255);
    // visualizer::setCameraPositionBasedOnBoundingBox(*viewer3, filteredPointCloud2);

    // viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
    // pcl::io::savePCDFileASCII("/home/jcvetic/INSPIRATION/Visualize_pointclouds/pointcloud2.pcd", *filteredPointCloud2);
    // pcl::io::savePCDFileASCII(savefolder + "/sagittal.pcd", *filteredPointCloud2);

    // ********************************** CORONAL_VIEW ********************************************************************************************

    // pcl::visualization::PCLVisualizer viewer4("Filtered_coronal_view");
    // std::unique_ptr<pcl::visualization::PCLVisualizer> viewer4(new pcl::visualization::PCLVisualizer("Filtered_coronal_view"));
    // viewer4->setSize(1920/2,1080);
    // viewer4->setPosition(1920,0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(filteredPointCloud3, 20, 100, 255); // 255, 0, 0

    // viewer4->addPointCloud(filteredPointCloud3, color3, "cloud3");

    // viewer4->setBackgroundColor(255, 255, 255);
    // visualizer::setCameraPositionBasedOnBoundingBox(*viewer4, filteredPointCloud3);

    // viewer4->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud3");
    // pcl::io::savePCDFileASCII("/home/jcvetic/INSPIRATION/Visualize_pointclouds/pointcloud3.pcd", *filteredPointCloud3);
    // pcl::io::savePCDFileASCII(savefolder + "/coronal.pcd", *filteredPointCloud3);

    while (!viewer->wasStopped()){
        viewer->spin();

    // //     viewer2->spinOnce();
    // //     viewer3->spinOnce();
    // //     viewer4->spinOnce();
    // //     // viewer44.spinOnce();

    // //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // return 0;
}

void visualizer::meshVTK(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jcvetic/INSPIRATION/Visualize_pointclouds/pointcloud1.pcd", *cloud) == -1){
        PCL_ERROR("Couldn't read file input.pcd \n");
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jcvetic/INSPIRATION/Visualize_pointclouds/filtered_combined_cloud.pcd", *combined_cloud) == -1){
        PCL_ERROR("Couldn't read file input.pcd \n");
    }

    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>()), normals2(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    Eigen::Vector4f centroid, centroid2;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::compute3DCentroid(*combined_cloud, centroid2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr demeaned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr demeaned_combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::demeanPointCloud(*cloud, centroid, *demeaned_cloud);
    pcl::demeanPointCloud(*combined_cloud, centroid2, *demeaned_combined_cloud);
    std::cout << demeaned_cloud->size() << std::endl;

    ne.setInputCloud(demeaned_cloud);
    ne.setSearchMethod(tree);
    // ne.setKSearch(100);
    ne.setRadiusSearch(3.0);
    // ne.setViewPoint(std::numeric_limits<float>::max (),
    //                 std::numeric_limits<float>::max (),
    //                 std::numeric_limits<float>::max ());
    ne.compute(*normals);

    ne.setInputCloud(demeaned_combined_cloud);
    ne.setRadiusSearch(3.0);
    ne.compute(*normals2);

    for (size_t i = 0; i < demeaned_cloud->points.size(); ++i) {
        pcl::flipNormalTowardsViewpoint(
            demeaned_cloud->points[i],      // The current point
            0.0f, 0.0f, 0.0f,      // The viewpoint coordinates
            normals->points[i].normal_x,  // Normal x component
            normals->points[i].normal_y,  // Normal y component
            normals->points[i].normal_z   // Normal z component
            );
    }

    for (size_t i = 0; i < demeaned_combined_cloud->points.size(); ++i) {
        pcl::flipNormalTowardsViewpoint(
            demeaned_combined_cloud->points[i],      // The current point
            0.0f, 0.0f, 0.0f,      // The viewpoint coordinates
            normals2->points[i].normal_x,  // Normal x component
            normals2->points[i].normal_y,  // Normal y component
            normals2->points[i].normal_z   // Normal z component
            );
    }

    // Combine XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>()), combined_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*demeaned_cloud, *normals, *cloud_with_normals);
    pcl::concatenateFields(*demeaned_combined_cloud, *normals2, *combined_cloud_with_normals);

    std::cout << cloud_with_normals->size() << std::endl;
    std::cout << normals->size() << std::endl;

    // Create search trees
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(8);  // Set the depth of the Poisson reconstruction (9 is a good default)
    poisson.setInputCloud(cloud_with_normals);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    poisson.setInputCloud(combined_cloud_with_normals);
    pcl::PolygonMesh mesh2;
    poisson.reconstruct(mesh2);

    pcl::visualization::PCLVisualizer viewer("Combined_cloud_NO_demean");
    viewer.setSize(1920,1080);
    viewer.setPosition(0,0);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPolygonMesh(mesh, "mesh");
    visualizer::setCameraPositionBasedOnBoundingBox(viewer,demeaned_cloud);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.85, 0.675, "mesh");
    // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(demeaned_cloud,normals,25, 10,"normals");
    viewer.addSphere(pcl::PointXYZ(0.0f,0.0f,0.0f), 4, 0.0, 1.0, 1.0, "centroid_sphere");

    pcl::visualization::PCLVisualizer viewer2("Combined_cloud_DEMEAN");
    viewer2.setSize(1920,1080);
    viewer2.setPosition(1920,0);
    viewer2.setBackgroundColor(0, 0, 0);
    viewer2.addPolygonMesh(mesh2, "mesh2");
    visualizer::setCameraPositionBasedOnBoundingBox(viewer2,demeaned_combined_cloud);
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.85, 0.675, "mesh2");


    // viewer2.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(demeaned_combined_cloud,normals2,25, 10,"normals");
    viewer2.addSphere(pcl::PointXYZ(0.0f,0.0f,0.0f), 4, 0.0, 1.0, 1.0, "centroid_sphere");

    // viewer2.addPolygonMesh(mesh2, "mesh2");
    // viewer2.addCoordinateSystem(1.0);
    // viewer2.setBackgroundColor(0.0, 0.0, 0.0);
    // viewer2.resetCameraViewpoint("mesh2");

    // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 2, 2, "normals");

    while (!viewer.wasStopped() && !viewer2.wasStopped()) {
        viewer.spinOnce();
        viewer2.spinOnce();
    }
}


void visualizer::outputData(const std::string& patientName, const std::vector<float>& pcdInfo, bool& firstInit){
    std::ofstream file; bool fileExists;
    struct stat buffer; std::string savepath;

    savepath = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/results/pcdData.csv";
    
    if ((stat (savepath.c_str(), &buffer) == 0)){
        fileExists = true;
    }
    else{
        fileExists = false;
    }

    if (firstInit){
        if (fileExists){
            file.open(savepath, std::ios::app);
        }
        else{
            file.open(savepath, std::ios::out);
            file << "Patient, rawApcd, rawSpcd, rawCpcd, fApcd, fSpcd, fCpcd, Combpcd, fCombpcd, diff(fApcd,fCombpcd)\n";
        }
    }
    else{
        file.open(savepath, std::ios::app);
    }

    if (file.is_open()){
        file << patientName << ",";

        for (int i = 0; i < pcdInfo.size(); ++i){
            file << pcdInfo[i];
            if (i < pcdInfo.size() -1){
                file << ",";
            } 
        }
        file << "\n";
    }
    file.close();
    // std::cout << "Data saved!" << std::endl;
}

void visualizer::run(std::string& patientName, bool& firstInit, bool saveData) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloud(3);
    std::vector<float> pcdInfo;
    for (int i = 0; i < 3; i++) {
        pointCloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    translateToPointCloud(voxelVector, pointCloud);
    // std::cout << "starting cloudWithNormals process" << std::endl;
    // saveCloudWithNormals(pointCloud[0]);
    // std::cout << "done" << std::endl;
    visualizePointClouds(pointCloud, pcdInfo, patientName);
    // meshVTK();

    if (saveData){
        outputData(patientName, pcdInfo, firstInit);
        std::cout << "Patient data saved." << std::endl << std::endl;
    }
}
