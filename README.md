# AG-Mapping
An application for 3D mapping tasks in agriculture uses object-based Visual SLAM 

# Enviroments

- Ubuntu 22.04
- RTX 3060
- CUDA 11.5
- cuDNN 8.7
- Libtorch 1.13

# Requierments

- OpenCV 4.2.0
- GTSAM 4.1.1
- [Gui lib](https://github.com/lacie-life/codecpp/blob/main/3D-tools-install.md) 

# Tasks

- [x] Refactor Odometry System
  - [x] VO Mode
  - [x] VIO Mode
- [ ] Visual trajectory estimated and ground-truth in 3D
- [ ] Model Semantic test
	- [x] YOLO
	- [ ] Segmentation model
- [ ] Quadric feature test
- [ ] VO with Quadric feature
- [ ] VIO with Quadric feature
- [ ] VPI test
- [ ] Quadric feature with VPI (???)

## Run
```
export LD_LIBRARY_PATH=/home/lin/local/lib/:$LD_LIBRARY_PATH
```
