# SHIELD

SHIELD, **S**pherical-Projection **H**ybrid-Frontier **I**ntegration for **E**fficient **L**i**D**AR-based UAV Exploration. **SHIELD** is a LiDAR-based UAV exploration framework which uses an outward spherical-projection ray-casting strategy to ensure flight safety and exploration efficiency in open areas with insufficient point cloud returns. It maintains an observation-quality occupancy map and performs ray-casting based on quality. A hybrid frontier method is used to tackle both the computational burden and the inconsistent quality of point clouds.



SHIELD achieves superior performance compared with the state-of-the-art methods.

<p align="center">
  <img src="Readme.assets/P1_720P.gif" alt="P1_720P" width="49%" />
  <img src="Readme.assets/P2_720P.gif" alt="P2_720P" width="49%" />
</p>

SHIELD was also tested through flight experiments in different types of real-world scenarios.

<p align="center">
  <img src="Readme.assets/P3_720P.gif" alt="P3_720P" width="32.5%" />
  <img src="Readme.assets/P4_720P.gif" alt="P4_720P" width="32.5%" />
  <img src="Readme.assets/P5_720P.gif" alt="P5_720P" width="32.5%" />
</p>

Please cite our paper if you use this project in your research:

```bibtex
@ARTICLE{shield,
  author={Feng, Liangtao and Liu, Zhenchang and Zhang, Feng and Ren, Xuefeng},
  journal={arXiv preprint arXiv:2512.23972},
  title={SHIELD: Spherical-Projection Hybrid-Frontier Integration for Efficient LiDAR-based Drone Exploration},
  year={2025},
  doi={10.48550/arXiv.2512.23972}
}
```






## Table of Contents

- [SHIELD](#shield)
  - [Table of Contents](#table-of-contents)
  - [Quick Start](#quick-start)
  - [Exploring Different Scenarios](#exploring-different-scenarios)
  - [Acknowledgements](#acknowledgements)

## Quick Start

**Test Environment**

- Ubuntu 20.04
- ROS Noetic
- C++17

**Clone Code**

```bash
cd
git clone https://github.com/LiuZhenchang/SHIELD.git
```

**Download PCD File**

Download simulation maps from  [Google Drive](https://drive.google.com/drive/folders/1A5Z1i0KR9-Gc7UP8QuEFMgSqcjj4GF5a?usp=sharing) or [Baidu Netdisk](https://pan.baidu.com/s/1vK-SEa1iA9xpRScj0l89ow?pwd=yink), create `rflysim_explore/explore_demo/src/MARSIM/map_generator/resource` if it doesn't exist, and move the downloaded maps to this folder.

```bash
mkdir -p ~/SHIELD/src/MARSIM/map_generator/resource
mv /path/to/downloaded/maps/*.pcd ~/SHIELD/src/MARSIM/map_generator/resource/
```

**Install  packages**

```bash
sudo apt-get install libglfw3-dev libglew-dev libarmadillo-dev libdw-dev
```

**install nlopt v2.7.1**

```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

**Install LKH-3**

```bash
wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz
tar xvfz LKH-3.0.6.tgz
cd LKH-3.0.6
make
sudo cp LKH /usr/local/bin
```

**Build**

```bash
cd ~/SHIELD
catkin_make
```





##  Exploring Different Scenarios

Run the bash script to start MARSIM, RVIZ and SHIELD.

```bash
cd ~/SHIELD/shell
bash Explore_marsim.sh
```

You can change the scene in `Explore_marsim.sh` to run different scenarios.

```
DEMO_NAME="community" # garage、cave、community、ruins
```



## Acknowledgements

This project is sponsored by Zhuoyi Intelligent Tech Co, Ltd.

We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:


[MARSIM](https://github.com/hku-mars/MARSIM)

[FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)

[RACER](https://github.com/Robotics-STAR-Lab/RACER)

We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.
