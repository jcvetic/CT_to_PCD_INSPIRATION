#include "includes.h"
#include "visualizer.h"
// #include <vtkDICOMImageReader.h>


// void saveTimeToCSV(double elapsedTime, const std::string& filename = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/results/times.csv") {
//     std::ofstream file(filename, std::ios::app); // Open in append mode
//     if (file.is_open()) {
//         file << elapsedTime << "\n"; // Append new time to file
//         file.close();
//     } else {
//         std::cerr << "Error: Unable to open file " << filename << std::endl;
//     }
// }

// void getPixelSpacing(){
//     std::string putanja = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/data/new_2103";
//     double *pixelSpacing, prevPixelSpacing = 1.0;
    
//     for (const auto& entry : std::filesystem::directory_iterator(putanja)){
//         std::string folderPath = entry.path().string();

//         vtkSmartPointer<vtkDICOMImageReader> reader = vtkSmartPointer<vtkDICOMImageReader>::New();
//         reader->SetDirectoryName(folderPath.c_str());
//         reader->Update();

//         pixelSpacing = reader->GetPixelSpacing();

//         if (*pixelSpacing < prevPixelSpacing){
//             prevPixelSpacing = *pixelSpacing;
//         }
//     }

//     std::cout << "Minimum pixel spacing: " << prevPixelSpacing << std::endl;
// }

int main() {
    // getPixelSpacing();
    bool firstInit = true; bool directories = false; bool saveData = false;
    std::string basePath = "/home/jcvetic/INSPIRATION/Visualize_pointclouds/data/CQ500/thin/patient45";

    for (const auto& entry : std::filesystem::directory_iterator(basePath)){
        if (entry.is_directory()){
            directories = true;
            break;
        }
    }
    if (directories){
        std::vector<double> timeelapsed;
        for (const auto& entry : std::filesystem::directory_iterator(basePath)){
            if (entry.is_directory()){
                std::string folderPath = entry.path().string();
                std::filesystem::path path(folderPath);
                std::string patientName = path.filename().string();

                // const auto start{std::chrono::steady_clock::now()};
                visualizer vis1(folderPath);

                vis1.run(patientName, firstInit, saveData);
                // const auto finish{std::chrono::steady_clock::now()};
                // const std::chrono::duration<double> elapsed{finish - start};

                // timeelapsed.push_back(elapsed.count());
                // saveTimeToCSV(elapsed.count());

                // if (timeelapsed.size() == 40){
                //     std::cout << "Elapsed times: ";
                //     for (double time : timeelapsed) {
                //         std::cout << time << "s ";
                //     }
                // std::cout << std::endl;
                // }
                if (firstInit){
                    firstInit=!firstInit;
                }
            }
        }
    }
    else{
        visualizer vis1(basePath);
        vis1.run(basePath, firstInit, saveData);
    }

    // std::filesystem::path path(folderPath);
    // std::string patientName = path.filename().string();
    // visualizer vis1(folderPath);
    // vis1.run(patientName);
    return 0;
}
