# AG-Mapping

An application for 3D mapping tasks in agriculture uses object-based Visual SLAM 

# TODO List:

- [x] Create point cloud by ORB-SLAM3 (Pending release code)
- [x] Publish point cloud to ROS
- [ ] Explore enviroment
   
   - [x] Yolov5 TensorRT support
   - [x] OctoMap support
   - [x] 3D cuboid support
   - [ ] Turning object database
   	   - [x] Segment 3D bounding box
   	   - [x] Object filter (sometime crashed)
   	   - [ ] Improve Object filter
   - [x] Add ZED example  
   - [ ] Optimize mapping memory
   - [ ] Plant bounding box
   
 - [ ] Evaluate results
   - [ ] SLAM 
   - [ ] 3D Mapping

# Install 

1. ROS Noetic

- vision_msgs 
  - sudo apt-get install ros-noetic-vision-msgs
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping)
- [octomap_rviz_plugins](https://github.com/OctoMap/octomap_rviz_plugins)

2. ag_mapping

- CUDA 11.6 [[Link](https://developer.nvidia.com/cuda-11-6-0-download-archive)]
  - <i> Choose option and follow instructions </i>

- OpenCV 4.5.2 (With CUDA Build) [[Link](https://github.com/lacie-life/codecpp/blob/main/opencv_cuda.sh)]
  - <i>Note: Change CUDA_ARCH flag to your NVIDIA Device and OpenCV version</i>
  - <i>Note: If you have conflict with Ros opencv, remove them and install ros depends manual </i>

- PCL 1.8 [[Link](https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.0.zip)]
  - <i> Uncompress and build </i>

- Pangolin

```
sudo apt-get install -y libglew-dev

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin 

./scripts/install_prerequisites.sh --dry-run recommended

mkdir build && cd build
cmake .. or cmake .. -DPython_EXECUTABLE='/usr/bin/python3'
cmake --build .

sudo make install
```

- TensorRT 
- ZED SDK 
- Realsense SDK 

3. TensorRT with Yolov5 model (Replace for libtorch in Jetson series)

```
git clone -b v7.0 https://github.com/ultralytics/yolov5.git
# create conda envs and install requierments.txt for running gen_wts.py
# stupid scripts

git clone -b yolov5-v7.0 https://github.com/wang-xinyu/tensorrtx.git
cd yolov5/
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
cp [PATH-TO-TENSORRTX]/yolov5/gen_wts.py .
python gen_wts.py -w yolov5s.pt -o yolov5s.wts
# A file 'yolov5s.wts' will be generated.

cd [PATH-TO-TENSORRTX]/yolov5/
# Update kNumClass in src/config.h if your model is trained on custom dataset
mkdir build
cd build
cp [PATH-TO-ultralytics-yolov5]/yolov5s.wts . 
cmake ..
make

# Generate engine file (engine include 80 class of coco dataset)
./yolov5_det -s yolov5s.wts yolov5s.engine s
```

# Bug 

Pls refer links:

[PCL build Errors](https://blog.csdn.net/weixin_51925771/article/details/118405623)
 
[PointCloudMapping Errors](https://blog.csdn.net/hai_fellow_Z/article/details/123681382)

# Note

- Make -j4
- If log too long, use can use this command:

```
catkin build -j4 &> log.txt
```


 
 
 
