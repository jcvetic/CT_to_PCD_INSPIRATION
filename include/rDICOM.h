#ifndef RDICOM_H
#define RDICOM_H
#include <opencv2/opencv.hpp>

// Forward declaration from VTK to reduce compile time since the full class definition is not required here
class vtkImageData;
class vtkPolyData;

class rDICOM
{
private:
    // static cv::Mat ConvertVTKToOpenCVMat(vtkSmartPointer<vtkImageData> slice);

    // Convert a VTK ImageData slice to an OpenCV Mat, internally used for image processing
    static cv::Mat ConvertVTKSliceToOpenCVMat(vtkSmartPointer<vtkImageData> slice);

    // Find the largest contour in an OpenCV Mat image
    static std::vector<cv::Point> FindLargestContour(const cv::Mat& image, bool includeInteriorPoints, int persp, int sliceNum);

    // Convert a contour represented by cv::Point vectors to VTK PolyData
    static vtkSmartPointer<vtkPolyData> ConvertContourToVTKPolyData(const std::vector<cv::Point>& contour);

    // Process a single slice of vtkImageData to extract the largest contour as vtkPolyData
    static vtkSmartPointer<vtkPolyData> ExtractLargestContourFromSlice(vtkSmartPointer<vtkImageData> slice, int persp, int sliceNum);


public:
    static std::vector<std::vector<std::array<double, 4>>> getDICOMdata(std::string folder);
    static vtkSmartPointer<vtkPolyData> ExtractSkinSurfaceFromCT(const std::string& dicomFolderPath); // If it's static
    static void ProcessAndReplaceAllSlices(vtkSmartPointer<vtkImageData> volume, int persp);

};

#endif // RDICOM_H
