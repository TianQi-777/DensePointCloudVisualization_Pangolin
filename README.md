# DensePointCloudVisualization_Pangolin
This demo is a simple 3D reconstruction of the room with an RGB-D camera.

## Additional Prerequisites for this project  
Besides,to build this project, you need the followings:  
**Pangolin**  
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

**Sophus**  
We use [Sophus](https://github.com/strasdat/Sophus) for Lie groups commonly used for 2d and 3d geometric problems. 
Dowload and install instructions can be found at: https://github.com/strasdat/Sophus.

**OpenCV**  
We use [OpenCV](http://opencv.org) to process images.

**Dataset**  
We use [TUM RGBD Dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#)(rgbd_dataset_freiburg1_desk2) to finish the test.

**C++11 or C++0x Compiler**  
We use the some functionalities of C++11.

## Build and Run
1. Modify some paths
   For examples:
   string strAssociationFilename = "XXX/rgbd_dataset_freiburg1_desk2/associate.txt";  
   string img_path = "XXX/rgbd_dataset_freiburg1_desk2/";  
   string txt_file_TUM = "XXX/CameraTrajectory.txt";  
2. cd XX/XX(include Pangolin_test.cpp ,CameraTrajectory.txt ,associate.txt and CMakeLists.txt)  
   mkdir build  
   cd build  
   cmake ..  
   make -j2  
   ./Pangolin_test
   
## Result
**Pangolin GUI:**
<div align=center>  
  
![](https://github.com/TianQi-777/TrackAlignmentWith_ICP/blob/master/images/Alignment.png)
</div>.
