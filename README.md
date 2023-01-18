# AG-Mapping
An application for 3D mapping tasks in agriculture uses object-based Visual SLAM 

# Requirements

- Ubuntu 20.04
- Qt 5.15.2
- PCL 1.10
- OpenCV 4.2
- Pangolin
- Boost
- CUDA 11.6
- Libtorch 1.12.1+cu116 (cxx11-abi)
- ZED SDK (Option)
- Realsense SDK (Option)

# TODO

- [x] Test SLAM module with TUM RGB-D dataset
  - [x] Point Cloud viewer
  - [ ] Yolo test (remove/bounding box a class => object?)
  - [ ] Segment model => 3D bounding box
  - [ ] Optimize point cloud
- [ ] Test Qt Gui display
    - [ ] Point Cloud view
    - [x] Keyframe Trajectory view
    - [x] Map point view
