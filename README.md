# Triplanar Point Cloud Reconstruction of Head Skin Surface from CT Images in Markerless Image-Guided Surgery



## Description

Accurate preoperative image processing in marker-less image-guided surgeries 1
is an important task. However, preoperative planning highly depends on the quality of 2
medical imaging data. In this study, a novel algorithm for outer skin layer extraction from 3
head computed tomography (CT) scans is presented and evaluated. Axial, sagittal, and 4
coronal slices are processed separately to generate spatial data. Each slice is thresholded 5
to create binary images, from which valid contours are extracted, reconstructed into uni- 6
planar point clouds, and merged into a single enriched triplanar point cloud. A two-step 7
downsampling process is applied, first at the uniplanar level and then after merging, using 8
a voxel size of 1 mm. Across two independent datasets with a total of 83 individuals, the 9
merged cloud approach yielded an average of 11.61% more unique points compared to 10
the axial cloud. The validity of the triplanar point cloud reconstruction was confirmed by 11
a root mean square (RMS) registration error of 0.848 ± 0.035 mm relative to the ground 12
truth models. These results establish the proposed algorithm as robust and accurate across 13
different CT scanners and acquisition parameters, supporting its potential integration into 14
patient registration for markerless image-guided surgeries.

### Dependencies

* Ubuntu 22.04.5 LTS, OpenCV 4.8.1., VTK 9.2.2., PCL 1.13.1.

### Executing program

* 1) in the main.cpp file, change the basepath to the valid path to a single patient CT scan
  2) setup the CmakeLists.txt file according to your setup
  3) open the terminal and execute the commands:
```
cmake --build . --config Debug -j$(nproc)
./(name of the project) - our example --> ./Visualize_pointclouds

```

## Authors
Jurica Cvetić

--> readme under development - WIP!
