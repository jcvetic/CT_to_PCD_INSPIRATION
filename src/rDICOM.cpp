#include "includes.h"

#include <eigen3/Eigen/Eigen>
#include <vtkSmartPointer.h>
#include <vtkDICOMImageReader.h>
#include <vtkMetaImageReader.h>
#include <vtkImageData.h>
#include <vtkMarchingCubes.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageThreshold.h>
#include <vtkImageResample.h>
#include <vtkContourFilter.h>
#include <vtkDecimatePro.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkSTLWriter.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkIdList.h>
#include <vtkImageReslice.h>
#include <vtkMatrix4x4.h>
#include <vtkDecimatePro.h>
#include <vtkTriangle.h>

#include "rDICOM.h"

// Member variable to store the points
std::vector<std::array<double, 4>> pointsVector;
std::vector<double> xyzList(6);
bool foundXYZ = false;
int border = 0; float maxdist;

cv::Mat rDICOM::ConvertVTKSliceToOpenCVMat(vtkSmartPointer<vtkImageData> slice) {
    int dims[3];
    // Get the dimensions of the slice (width, height, depth)
    slice->GetDimensions(dims);

    // Assuming the slice is a single channel grayscale image
    cv::Mat cvImage = cv::Mat::zeros(dims[1], dims[0], CV_8UC1); // OpenCV uses rows, cols
    //std::cout<<"Slice dim within ConvertVTKtoMAT = "<<cvImage.size()<<std::endl;

    // Access the scalar values directly from vtkImageData
    vtkDataArray* scalars = slice->GetPointData()->GetScalars();

    // It's important to know the type of the data in the vtkImageData.
    // This example assumes the data is stored as short (16-bit) integers, which is common for CT data.
    for (int y = 0; y < dims[1]; ++y) {
        for (int x = 0; x < dims[0]; ++x) {
            // vtkImageData is stored in a 1D array, so compute the index
            int idx = y * dims[0] + x;
            short value = scalars->GetTuple1(idx);

            // Map HU values between -500 and 200 to 255, else to 0, -450 100
            if (value >= -200 && value <= 220) { // 175,150
                cvImage.at<uchar>(y, x) = 255;
            } else {
                cvImage.at<uchar>(y, x) = 0;
            }
        }
    }

    // cv::imshow("image123", cvImage);
    // cv::waitKey(0);
    return cvImage;
}

double contArea; bool firstInit = true; double contLength; double contArea2; 
// int iterator = 1608;
// AXIAL 0, SAGITAL 1, CORONAL 2 VIEW -------------------------------------------------------------------------------------------------------------------------------
std::vector<cv::Point> rDICOM::FindLargestContour(const cv::Mat& image, bool includeInteriorPoints, int persp, int sliceNum) {
    std::vector<std::vector<cv::Point>> contours;
    double diffResult;
    cv::Mat image2; cv::Mat tempBGR; cv::Mat test123;
    image2 = image.clone();
    tempBGR = image2.clone();
    cv::cvtColor(tempBGR, tempBGR, cv::COLOR_GRAY2BGR);
    cv::findContours(image2, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); //external, NONE

    for (auto it = contours.begin(); it != contours.end();) {
        cv::Moments m = cv::moments(*it);

        if (m.m00 != 0) {
            cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
            // std::cout << center.x << std::endl;
            if (persp == 0){
                if (center.x > 140 && center.x < 360 && center.y > 140 && center.y < 360 && cv::contourArea(*it) > 500) {
                    ++it;
                }
                else{
                    it = contours.erase(it);
                }                
            }
            else if (persp == 1){
                if (center.x > 100 && center.x < 400 && center.y > 10 && cv::contourArea(*it) > 500) {
                    ++it;
                }
                else {
                    it = contours.erase(it);
                }
            }
            else{
                if (center.x > 100 && center.x < 400 && center.y > 10 && cv::contourArea(*it) > 500) {
                    ++it;
                }
                else {
                    it = contours.erase(it);
                }
            }
        }
        else {
            it = contours.erase(it);
        }
    }

    if (contours.empty()) {
        return std::vector<cv::Point>(); // Return empty if no contours found
    }

    // Sort contours by area in descending order
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
        return cv::contourArea(c1, false) > cv::contourArea(c2, false);
    });

    // The largest contour is now the first in the sorted vector
    std::vector<cv::Point> largestContour;
    std::vector<std::vector<cv::Point>> larCont;

    image2.setTo(cv::Scalar(0));
    cv::drawContours(image2, contours, -1, cv::Scalar(255),cv::FILLED);

    std::vector<cv::Point> pointList;
    for (int j=0; j<image2.cols; j++) {
        if (image2.at<uchar>(0,j) == 255) {
            pointList.push_back(cv::Point(j,0));
        }
    }

    if (pointList.size() > 1) {
        cv::line(image2, pointList[0], pointList.back(), 255, 1);
    }

    contours.clear();
    cv::findContours(image2, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
        return cv::contourArea(c1, false) > cv::contourArea(c2, false);});

    largestContour = contours.front();
    largestContour.erase(std::remove_if(largestContour.begin(), largestContour.end(),[](const cv::Point& point) { return point.y == 0; }),largestContour.end());
    larCont.push_back(largestContour);

    if (firstInit){ 
        contArea = cv::contourArea(largestContour);
        contLength = cv::arcLength(largestContour, true);
        firstInit = false;
    }

    std::vector<cv::Point> contHull; double contHullArea; double solidity; double contourArea;
    contourArea = cv::contourArea(largestContour);
    cv::convexHull(largestContour,contHull);
    contHullArea = cv::contourArea(contHull);
    solidity = contourArea / contHullArea;

    if (contourArea > 40000){
        if (solidity < 0.88){
            return std::vector<cv::Point>();
        }
    }

    if (8000 < std::abs(contArea - cv::contourArea(largestContour)) && std::abs(contArea - cv::contourArea(largestContour)) < 120000){
        cv::Moments m = cv::moments(largestContour);
        cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
        int centery = center.y; int pointOnContour;

        for (int i = centery; i < image2.rows; i+=2 ){
            pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(center.x, i),false);
            if (pointOnContour == -1) {
                if (image2.at<uchar>(i, center.x) == 255) {
                    // if (persp == 1){
                    // cv::cvtColor(image2,image2,cv::COLOR_GRAY2BGR);
                    // cv::drawContours(image2, larCont, -1, cv::Scalar(255,0,255),2); // image2 inace!!1
                    // cv::circle(image2, cv::Point2f(center.x,i), 8, cv::Scalar(0,255,0), -1);
                    // cv::circle(image2, cv::Point2f(center.x,center.y), 8, cv::Scalar(255,255,0), -1);
                    // cv::imshow("image123", image2);
                    // cv::waitKey(0);
                    // }
                    return std::vector<cv::Point>();
                }
            }
        }

        for (int i = centery; i >= 0; i-=2 ){
            pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(center.x, i),false);
            if (pointOnContour == -1) {
                if (image2.at<uchar>(i, center.x) == 255) {
                    return std::vector<cv::Point>();
                }
            }
        }

        int centerx = center.x;

        for (int i = centerx; i < image2.cols; i+=2 ){
            pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(i, center.y),false);

            if (pointOnContour == -1) {
                // cv::circle(image2, cv::Point2f(center.x,i), 1, 255, 3);
                if (image2.at<uchar>(center.y, i) == 255) {
                    return std::vector<cv::Point>();
                }
            }
        }
        for (int i = centerx; i >= 0; i-=2 ){
            pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(i, center.y),false);

            if (pointOnContour == -1) {
                if (image2.at<uchar>(center.y, i) == 255) {
                    return std::vector<cv::Point>();
                }
            }
        }

        if (contourArea > 120000) {
            return std::vector<cv::Point>();
        }
        else if(std::abs(contArea - cv::contourArea(largestContour)) > 80000){ //elseif
            return std::vector<cv::Point>();
        }
        if (solidity < 0.75){
            return std::vector<cv::Point>();
        }
    }

    contArea = cv::contourArea(largestContour);

    int intersec = 0; int onContourCounter; int prevColumn; int rowIdx; int stepsize; int contourcount; int colbegin; int colend;
    if (contourArea > 15000){ // 40000 
        if (persp == 1 || persp == 2){
            if (persp == 1){
                stepsize = 4; contourcount = 4; colbegin = image2.cols-1; colend = 0;
            }
            else{
                stepsize = 8; contourcount = 3; colbegin = image2.cols/2; colend = 0;
            }
            for (int j = 8; j <= image2.rows/2; j+=stepsize){ //4
                onContourCounter = 0; prevColumn = -1;
                for (int i = colbegin; i >= colend; i--){
                    double pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(i,j),false);

                    if (pointOnContour == 0){
                        if (prevColumn == -1 || (prevColumn - i >= 4)){ //8
                            prevColumn = i;
                            onContourCounter += 1;
                        }
                    }
                    if (onContourCounter >= contourcount){
                        intersec+=1;
                        if (intersec == 4){
                            return std::vector<cv::Point>();
                        }
                        break;
                    }
                }
            }
        }
        if (persp == 0 || persp == 2){ 
            intersec = 0; rowIdx = -1;
            for (int j = image2.rows/2; j <= image2.rows-1; j+=4){
                if (image2.at<uchar>(j, image2.cols/2) == 255){
                    rowIdx = j;
                }
            }
            if (rowIdx == -1){
                rowIdx = image2.rows - 40;
            }
            for (int j = image2.rows/2; j <= rowIdx-10; j+=4){ 
                onContourCounter = 0; prevColumn = -1;
                for (int i = image2.cols-1; i >= 0; i--){
                    double pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(i,j),false);

                    if (pointOnContour == 0){
                        if (prevColumn == -1 || (prevColumn - i >= 6)){ // 8
                            cv::circle(tempBGR, cv::Point2f(i,j), 2, cv::Scalar(255,0,0), 2);
                            prevColumn = i;
                            onContourCounter += 1;
                        }
                    }

                    if (onContourCounter >= 4){
                        intersec += 1;
                        if (intersec >= 5){
                            return std::vector<cv::Point>();
                        }
                        break;
                    }
                }
            }
        }
        if (persp == 0){
            intersec = 0; std::vector<int> intens;
            for (int j = 0; j <= image2.rows/2; j+=8){
                onContourCounter = 0; prevColumn = -1; int totalsum = 0;
                for (int i = image2.cols-1; i >= 0; i--){
                    if (intens.size() >= 5){
                        intens.erase(intens.begin());
                    }
                    intens.push_back(image2.at<uchar>(j,i)); //intensity
                    double pointOnContour = cv::pointPolygonTest(largestContour,cv::Point2f(i,j),false);
                    // cv::circle(tempBGR, cv::Point2f(i,j), 2, cv::Scalar(255,0,0), 2);
                    if (pointOnContour == 0){
                        if (prevColumn == -1 || (prevColumn - i >= 2)){ // 8
                            cv::circle(tempBGR, cv::Point2f(i,j), 2, cv::Scalar(255,0,0), 2); // tempbgr
                            prevColumn = i;

                            for (int num : intens){
                                totalsum += num;
                                // std::cout << totalsum/intens.size() << std::endl;
                            }

                            if ((totalsum / intens.size()) < 170){
                                onContourCounter += 1;
                                cv::circle(tempBGR, cv::Point2f(i,j), 1, cv::Scalar(255,0,0), 2);
                            }
                            else{
                                if (onContourCounter > 2){
                                    onContourCounter -= 2;
                                }
                                else{
                                    onContourCounter = 0;
                                }
                            }
                            totalsum = 0;
                        }
                    }
                }
                intens.clear();
                if (onContourCounter >= 1){
                    intersec += 1;
                    if (intersec >= 5){
                        // std::cout << onContourCounter << std::endl;
                        // cv::imshow("image2", image2);
                        // cv::waitKey(0);
                        // cv::destroyAllWindows();
                        return std::vector<cv::Point>();
                    }
                    // break;
                }
            }
        }
    }

    if (persp == 1 || persp == 2){
        if (contourArea < 30000){
            int pickrow = 30;
            for (int i = image2.rows-pickrow; i < image2.rows; i++){ // bilo 25,40
                for (int j = 0; j < image2.cols; j++){
                    if (image2.at<uchar>(i,j) == 255){
                        return std::vector<cv::Point>();
                    }
                }
            }
        }
        if (contourArea < 50000){
            for (int j=0; j<image2.cols; j++) {
                if (image2.at<uchar>(image2.rows-1,j) == 255){
                    return std::vector<cv::Point>();
                }
            }
        }
    }

    else {
        if (contourArea < 10000){
            for (int j = 0; j < image2.cols/4; j++){
                // cv::circle(image2,cv::Point2f(j, i),2,255,2);
                if (image2.at<uchar>(image2.rows/2,j) == 255){
                    // cv::circle(image2,cv::Point2f(492,131),2,255,2);
                    return std::vector<cv::Point>();
                }
            }         
        }
    }

    if (!includeInteriorPoints) {
        // If only the contour points are requested, return them immediately
        return largestContour;
    }

    // If includeInteriorPoints is true, proceed to fill the contour and find all points

    // Create a mask where the largest contour will be drawn
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

    cv::drawContours(mask, contours, -1, cv::Scalar(255), 2);

    // Find all points in the filled contour
    std::vector<cv::Point> allPointsInContour;
    for (int y = 0; y < mask.rows; ++y) {
        for (int x = 0; x < mask.cols; ++x) {
            if (mask.at<uchar>(y, x) == 255) { // If the mask pixel is part of the contour
                allPointsInContour.push_back(cv::Point(x, y));
            }
        }
    }
    return allPointsInContour;
}


vtkSmartPointer<vtkPolyData> rDICOM::ConvertContourToVTKPolyData(const std::vector<cv::Point>& contour) {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& pt : contour) {
        points->InsertNextPoint(pt.x, pt.y, 0); // Assuming 2D contour with z=0
    }

    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(contour.size());
    for (unsigned int i = 0; i < contour.size(); i++) {
        polyLine->GetPointIds()->SetId(i, i);
    }

    // Instead of inserting vtkPolyLine directly, use vtkIdList for the cell array
    vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
    for(unsigned int i = 0; i < contour.size(); i++) {
        idList->InsertNextId(i);
    }

    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    // Add the entire polyline at once using the id list
    cells->InsertNextCell(idList);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(cells);

    return polyData;
}

vtkSmartPointer<vtkPolyData> rDICOM::ExtractLargestContourFromSlice(vtkSmartPointer<vtkImageData> slice, int persp, int sliceNum) {
    cv::Mat image = ConvertVTKSliceToOpenCVMat(slice);
    std::vector<cv::Point> largestContour = FindLargestContour(image,false,persp,sliceNum);
    return ConvertContourToVTKPolyData(largestContour);
}

void rDICOM::ProcessAndReplaceAllSlices(vtkSmartPointer<vtkImageData> volume, int persp) {
    int dims[3];
    volume->GetDimensions(dims); // Get the dimensions of the volume data
    vtkSmartPointer<vtkMatrix4x4> resliceAxes = vtkSmartPointer<vtkMatrix4x4>::New();
    bool initializeXYZ = true;

    switch (persp) {
    case 0:
        // Setup the slicing orientation ----> AXIAL slicing!!!!
        resliceAxes->Identity();
        resliceAxes->SetElement(0,0,1);
        resliceAxes->SetElement(1,1,1);
        resliceAxes->SetElement(2,2,1);

        xyzList[4] = 0; xyzList[5] = dims[2];
        // Configure the matrix for axial slicing (this example assumes z slicing, adjust if needed)
        for (int z = 0; z < dims[2]; ++z) {
            double sliceSpacing = volume->GetSpacing()[2]; // Get the z-spacing
            double sliceOrigin = volume->GetOrigin()[2] + z * sliceSpacing; // Calculate the origin of the current slice

            // std::cout << sliceOrigin << " " << sliceSpacing << "\n";

            // Configure the resliceAxes to extract the correct slice
            resliceAxes->SetElement(2, 3, sliceOrigin);

            // std::cout << "ResliceAxes: " << *resliceAxes << endl;

            // Setup vtkImageReslice for slicing the volume
            vtkSmartPointer<vtkImageReslice> reslicer = vtkSmartPointer<vtkImageReslice>::New();
            reslicer->SetInputData(volume);
            reslicer->SetOutputDimensionality(2); // We want to extract 2D slices
            reslicer->SetResliceAxes(resliceAxes);
            reslicer->SetInterpolationModeToLinear(); // You can adjust the interpolation mode as needed

            // Perform the slicing
            reslicer->Update();

            // The output of the reslicer is a 2D vtkImageData object representing the slice
            vtkSmartPointer<vtkImageData> slice = reslicer->GetOutput();

            // Here you can process the slice, for example, extract the largest contour from the slice
            vtkSmartPointer<vtkPolyData> largestContourPolyData = ExtractLargestContourFromSlice(slice,persp,z);
            // Assuming largestContourPolyData is valid and has points
            vtkSmartPointer<vtkPoints> contourPoints = largestContourPolyData->GetPoints();

            int dims[3];
            volume->GetDimensions(dims);

            // std::cout << dims[2] << std::endl;

            double spacing[3];
            volume->GetSpacing(spacing);
            double origin[3];
            volume->GetOrigin(origin);

            if (contourPoints) {
                for (vtkIdType i = 0; i < contourPoints->GetNumberOfPoints(); i++) {
                    double p[3];
                    contourPoints->GetPoint(i, p);

                    // Convert image coordinates to world coordinates
                    std::array<double, 4> worldPoint;
                    worldPoint[0] = p[0] * spacing[0] + origin[0]; // X in mm # (x * spacing[0]) + origin[0];
                    worldPoint[1] = p[1] * spacing[1] + origin[1]; // Y in mm
                    worldPoint[2] = sliceOrigin; // Z in mm, assuming slice index corresponds directly to z-coordinate -- promjena
                    worldPoint[3] = 1; // Fourth dimension as specified
                    pointsVector.push_back(worldPoint);

                    if (initializeXYZ) {
                        xyzList[0] = worldPoint[0];
                        xyzList[1] = worldPoint[0];
                        xyzList[2] = worldPoint[1];
                        xyzList[3] = worldPoint[1];
                        xyzList[4] = 0;
                        xyzList[5] = dims[2];
                        initializeXYZ = false;
                    }
                    // Add to the vector
                    if (worldPoint[0] < xyzList[0]) {
                        xyzList[0] = worldPoint[0];
                    }
                    if (worldPoint[0] > xyzList[1]) {
                        xyzList[1] = worldPoint[0]; // worldPoint[0]
                    }
                    if (worldPoint[1] < xyzList[2]) {
                        xyzList[2] = worldPoint[1];
                    }
                    if (worldPoint[1] > xyzList[3]) {
                        xyzList[3] = worldPoint[1];
                    }
                }
            }
        }
        // std::cout << maxdist << std::endl;
        break;
    case 1:
        // Setup the slicing orientation ----> SAGITAL slicing!!!!
        resliceAxes->Identity();        

        resliceAxes->SetElement(0, 0, 0);
        resliceAxes->SetElement(0, 1, 0);
        resliceAxes->SetElement(0, 2, -1);   // Slicing along the X-axis, matrix
        resliceAxes->SetElement(1, 0, 1);
        resliceAxes->SetElement(1, 1, 0);
        resliceAxes->SetElement(1, 2, 0);
        resliceAxes->SetElement(2, 0, 0);
        resliceAxes->SetElement(2, 1, -1);
        resliceAxes->SetElement(2, 2, 0);


        for (int x = 0; x < dims[0]; ++x) {
            double sliceSpacing = volume->GetSpacing()[0]; // Get the z-spacing
            double sliceOrigin = volume->GetOrigin()[0] + x * sliceSpacing; // Calculate the origin of the current slice
            // std::cout << sliceOrigin << " " << sliceSpacing << "\n";

            // Configure the resliceAxes to extract the correct slice
            resliceAxes->SetElement(0, 3, sliceOrigin);

            // std::cout << "ResliceAxes: " << *resliceAxes << endl;

            // Setup vtkImageReslice for slicing the volume
            vtkSmartPointer<vtkImageReslice> reslicer = vtkSmartPointer<vtkImageReslice>::New();
            reslicer->SetInputData(volume);
            reslicer->SetOutputDimensionality(2); // We want to extract 2D slices
            reslicer->SetResliceAxes(resliceAxes);
            reslicer->SetInterpolationModeToLinear(); // You can adjust the interpolation mode as needed

            // Perform the slicing
            reslicer->Update();

            // The output of the reslicer is a 2D vtkImageData object representing the slice
            vtkSmartPointer<vtkImageData> slice = reslicer->GetOutput();

            // Here you can process the slice, for example, extract the largest contour from the slice
            vtkSmartPointer<vtkPolyData> largestContourPolyData = ExtractLargestContourFromSlice(slice,persp,x);
            // Assuming largestContourPolyData is valid and has points
            vtkSmartPointer<vtkPoints> contourPoints = largestContourPolyData->GetPoints();

            int dims[3];
            volume->GetDimensions(dims);
            double spacing[3];
            volume->GetSpacing(spacing);
            double origin[3];
            volume->GetOrigin(origin);

            if (contourPoints) {
                for (vtkIdType i = 0; i < contourPoints->GetNumberOfPoints(); i++) {
                    double p[3];
                    contourPoints->GetPoint(i, p);

                    // Convert image coordinates to world coordinates
                    std::array<double, 4> worldPoint;
                    worldPoint[0] = sliceOrigin;
                    worldPoint[1] = p[0] * spacing[1] + origin[1]; 
                    worldPoint[2] = -1*(p[1] * spacing[2] + origin[2])+dims[2]*spacing[2];
                    worldPoint[3] = 1; 
                    // Add to the vector
                    // std::cout << currdist << std::endl;
                    if ((xyzList[0] - border <= worldPoint[0] && xyzList[1] + border >= worldPoint[0] && xyzList[2] - border <= worldPoint[1] && xyzList[3] + border >= worldPoint[1] && xyzList[4] - border <= worldPoint[2] && xyzList[5] + border >= worldPoint[2])){
                        pointsVector.push_back(worldPoint);
                    }
                }
            }
        }
        break; 
    case 2:
        // Setup the slicing orientation ----> CORONAL slicing!!!!
        resliceAxes->Identity();

        resliceAxes->SetElement(0, 0, 1);
        resliceAxes->SetElement(0, 1, 0);
        resliceAxes->SetElement(0, 2, 0);   // Slicing along the Y-axis, matrix
        resliceAxes->SetElement(1, 0, 0);
        resliceAxes->SetElement(1, 1, 0);
        resliceAxes->SetElement(1, 2, 1);
        resliceAxes->SetElement(2, 0, 0);
        resliceAxes->SetElement(2, 1, -1);
        resliceAxes->SetElement(2, 2, 0);

        for (int y = 0; y < dims[1]; ++y) {
            double sliceSpacing = volume->GetSpacing()[1]; 
            double sliceOrigin = volume->GetOrigin()[1] + y * sliceSpacing; 

            // std::cout << sliceOrigin << " " << sliceSpacing << "\n";

            // Configure the resliceAxes to extract the correct slice
            resliceAxes->SetElement(1, 3, sliceOrigin);

            // std::cout << "ResliceAxes: " << *resliceAxes << endl;

            // Setup vtkImageReslice for slicing the volume
            vtkSmartPointer<vtkImageReslice> reslicer = vtkSmartPointer<vtkImageReslice>::New();
            reslicer->SetInputData(volume);
            reslicer->SetOutputDimensionality(2); // We want to extract 2D slices
            reslicer->SetResliceAxes(resliceAxes);
            reslicer->SetInterpolationModeToLinear(); // You can adjust the interpolation mode as needed

            // Perform the slicing
            reslicer->Update();

            // The output of the reslicer is a 2D vtkImageData object representing the slice
            vtkSmartPointer<vtkImageData> slice = reslicer->GetOutput();

            // Here you can process the slice, for example, extract the largest contour from the slice
            vtkSmartPointer<vtkPolyData> largestContourPolyData = ExtractLargestContourFromSlice(slice,persp,y);
            // Assuming largestContourPolyData is valid and has points
            vtkSmartPointer<vtkPoints> contourPoints = largestContourPolyData->GetPoints();

            int dims[3];
            volume->GetDimensions(dims);

            // std::cout << dims[0] << " " << dims[1] << " " << dims[2] <<"\n";

            double spacing[3];
            volume->GetSpacing(spacing);
            double origin[3];
            volume->GetOrigin(origin);

            if (contourPoints) {
                for (vtkIdType i = 0; i < contourPoints->GetNumberOfPoints(); i++) {
                    double p[3];
                    contourPoints->GetPoint(i, p);

                    // Convert image coordinates to world coordinates
                    std::array<double, 4> worldPoint;
                    worldPoint[0] = p[0] * spacing[0] + origin[0]; 
                    worldPoint[1] = sliceOrigin;// Y in mm
                    worldPoint[2] = -1*(p[1] * spacing[2] + origin[2])+dims[2]*spacing[2]; 
                    worldPoint[3] = 1; 

                    if ((xyzList[0] - border <= worldPoint[0] && xyzList[1] + border >= worldPoint[0] && xyzList[2] - border <= worldPoint[1] && xyzList[3] + border >= worldPoint[1] && xyzList[4] - border <= worldPoint[2] && xyzList[5] + border >= worldPoint[2]))
                        pointsVector.push_back(worldPoint);
                }
            }
        }
        break;
    }
}

// Function to process DICOM CT scan and extract skin surface
vtkSmartPointer<vtkPolyData> rDICOM::ExtractSkinSurfaceFromCT(const std::string& dicomFolderPath) {
    // Read DICOM data
    vtkSmartPointer<vtkDICOMImageReader> dicomReader = vtkSmartPointer<vtkDICOMImageReader>::New();
    dicomReader->SetDirectoryName(dicomFolderPath.c_str());
    dicomReader->Update();
    std::cout<<"Marker1"<<endl;

    // Resample the image for a higher/lower resolution if necessary
    double resolution = 0.2;
    vtkSmartPointer<vtkImageResample> resample = vtkSmartPointer<vtkImageResample>::New();
    resample->SetInputConnection(dicomReader->GetOutputPort());
    resample->SetAxisMagnificationFactor(0, resolution);
    resample->SetAxisMagnificationFactor(1, resolution);
    // Z-axis resolution might be adjusted separately if needed
    std::cout<<"Marker1"<<endl;

    // Extract the skin surface using an isocontour value typical for skin
    double isocontourValue = -500; // Hounsfield units for skin
    vtkSmartPointer<vtkContourFilter> contourFilter = vtkSmartPointer<vtkContourFilter>::New();
    contourFilter->SetInputConnection(resample->GetOutputPort());
    contourFilter->SetValue(0, isocontourValue);
    std::cout<<"Marker1"<<endl;
    // Decimate the mesh to reduce complexity
    double decimationFactor = 0.3;
    vtkSmartPointer<vtkDecimatePro> decimator = vtkSmartPointer<vtkDecimatePro>::New();
    decimator->SetInputConnection(contourFilter->GetOutputPort());
    decimator->SetTargetReduction(decimationFactor);
    decimator->SetPreserveTopology(true);
    std::cout<<"Marker1"<<endl;
    // Smooth the mesh to improve visual quality
    int numberOfSmoothingIterations = 50;
    vtkSmartPointer<vtkSmoothPolyDataFilter> smoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
    smoother->SetInputConnection(decimator->GetOutputPort());
    smoother->SetNumberOfIterations(numberOfSmoothingIterations);
    std::cout<<"Marker1"<<endl;
    // Recalculate normals for better lighting effects
    vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
    normals->SetInputConnection(smoother->GetOutputPort());
    normals->SetFeatureAngle(120);
    std::cout<<"Marker1"<<endl;
    // Extract the largest connected component to isolate the face/skin surface
    vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
    connectivityFilter->SetInputConnection(normals->GetOutputPort());
    connectivityFilter->SetExtractionModeToLargestRegion();
    connectivityFilter->Update();

    return connectivityFilter->GetOutput();
}

std::vector<std::vector<std::array<double, 4>>> rDICOM::getDICOMdata(std::string folder)
{
    //--------------To run extract surface using opencv
    vtkSmartPointer<vtkDICOMImageReader> reader = vtkSmartPointer<vtkDICOMImageReader>::New();
    reader->SetDirectoryName(folder.c_str());
    reader->Update();
    std::cout<<"Folder - "<<folder<<std::endl;

    vtkSmartPointer<vtkImageData> volume = reader->GetOutput();

    std::vector<std::vector<std::array<double, 4>>> voxelVector(3);
    for(int persp = 0; persp < 3; ++persp){
        pointsVector.clear();
        firstInit = true;
        ProcessAndReplaceAllSlices(volume, persp); // 0 = axial, 1 = sagital, 2 = coronal
        std::cout << "Iteration " << persp << " done\n";

        if (pointsVector.size()>1){
            voxelVector[persp].insert(voxelVector[persp].end(), pointsVector.begin(), pointsVector.end());
        }
        else{

            // Convert vtkImageData to vtkPolyData using vtkMarchingCubes or another suitable filter
            std::cout<<"Starting reconstruction with volume point size = "<<volume->GetNumberOfPoints()<<std::endl;
            vtkSmartPointer<vtkMarchingCubes> marchingCubes = vtkSmartPointer<vtkMarchingCubes>::New();
            marchingCubes->SetInputData(volume);
            marchingCubes->SetValue(-500, 200); // Example isovalue; adjust based on your data
            marchingCubes->Update();
            vtkSmartPointer<vtkPolyData> polyData = marchingCubes->GetOutput();
            std::cout<<"Reconstruction - DONE."<<std::endl;
            // Extract points from vtkPolyData
            //vtkPoints* points = polyData->GetPoints();

            // Decimation
            vtkSmartPointer<vtkDecimatePro> decimate = vtkSmartPointer<vtkDecimatePro>::New();
            decimate->SetInputData(polyData);
            decimate->SetTargetReduction(0.8); // Adjust this value to control decimation percentage
            decimate->PreserveTopologyOn(); // Preserve the topology of the original mesh
            decimate->Update();

            // The output of decimation is a simplified version of the original polyData
            vtkSmartPointer<vtkPolyData> simplifiedPolyData = decimate->GetOutput();
            std::cout << "Decimation - DONE." << std::endl;

            //Extract points from vtkPolyData
            vtkPoints* points = simplifiedPolyData->GetPoints();

            for(vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
                double p[3];
                points->GetPoint(i, p);
                // Assuming the value to be 1 for each vertex, as an example
                // voxelVector.push_back({p[0], p[1], p[2], 1.0}); 
            }
            // std::cout<<"VoxelVector = "<<voxelVector.size()<<std::endl;
            //-------------------- Visualization of the resulting surface mesh
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(simplifiedPolyData);

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);

            vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
            vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
            renderWindow->AddRenderer(renderer);
            vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
            renderWindowInteractor->SetRenderWindow(renderWindow);

            renderer->AddActor(actor);
            renderer->SetBackground(0.1, 0.2, 0.3); // Background color dark blue

            renderWindow->Render();
            renderWindowInteractor->Start();

            std::cout<<"Starting stl writer..."<<std::endl;
            // -------------------------------Create an STL writer and set the input data
            vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
            stlWriter->SetFileName("/home/jcvetic/Patient_registration_cveta/Patient_registration/output.stl");
            stlWriter->SetInputData(simplifiedPolyData);
            // Write the file
            stlWriter->Write();
            std::cout << "STL file has been saved as output.stl" << std::endl;
            //----------------------------------STL writer
        }

    }
    return voxelVector;
}
