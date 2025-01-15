Dynamic Obstacle Removal for Grandtour Project
---

0. Prerequisite  
  Pointcloud of every single scan is extracted from rosbag by calling [Pointcloud Extract](https://github.com/iiisaac40/simple_ndt_slam_grandtour).   
  If not, do it first


1. clone the code
```sh
git clone https://github.com/iiisaac40/dynablox_grandtour.git
```

2. Navigate to the repo and build
```sh
cd dynablox_grandtour
cmake -B build && cmake --build build
```

3. Run the code
```sh
./build/dynablox_run  {Path to DATA Folder} {Path to Config File} {pcd timestamp}

Example
./build/dynablox_run ../11-03/extracted_pcd assets/config_grandtour.yaml 1730618341820832014 
```

**The following is the original README**

Dynablox
---

This is no ros version! Origin one please check fork repo.

pcd files are enough to run this program. Need transformed and pose in VIEWPOINT. 

Please reference our [DynamicMap benchmark](https://github.com/KTH-RPL/DynamicMap_Benchmark) for more detail on datasets.

For more depedenices or one Dockerfile for all, check: 

### Dependencies

1. ROS-full: for boost and Eigen or you can install these two manually

2. glog gflag (only for print)
    ```sh
    sh -c "$(wget -O- https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/Dockerfiles/latest_glog_gflag.sh)"
    ```

3. yaml-cpp. Please set the FLAG, check this issue if you want to know more: https://github.com/jbeder/yaml-cpp/issues/682, [TOOD inside the CMakeLists.txt](https://github.com/jbeder/yaml-cpp/issues/566)

    If you install in Ubuntu 22.04, please check [this commit:jbeder/yaml-cpp](https://github.com/jbeder/yaml-cpp/commit/c86a9e424c5ee48e04e0412e9edf44f758e38fb9) which is the version could build in 22.04

    ```sh
    cd ${Tmp_folder}
    git clone --branch yaml-cpp-0.6.0 --single-branch https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp
    env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -Bbuild
    cmake --build build --config Release
    sudo cmake --build build --config Release --target install
    ```

    An issue mentioned here, and I needed set 0.6.0 version to get rid of error, check [this comment](https://stackoverflow.com/a/75293221/9281669)

### Build and Run

Data example (Teaser data on KITTI sequence 00 only 384.8MB):
```bash
wget https://zenodo.org/records/10886629/files/00.zip
unzip 00.zip -d ${data_path, e.g. /home/kin/data}
```

```bash
cmake -B build && cmake --build build

./build/dynablox_run /home/kin/data/00 assets/config.yaml -1
```

Demo effect:
![demo](./assets/imgs/demo.png)

### Cite Papers

This work is refactored during our DynamicMap benchmark and DUFOMap, please cite our work if you use this code version. 

Please also cite original work by clicking to the fork on top (core method).

```
@inproceedings{zhang2023benchmark,
  author={Zhang, Qingwen and Duberg, Daniel and Geng, Ruoyu and Jia, Mingkai and Wang, Lujia and Jensfelt, Patric},
  booktitle={IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)}, 
  title={A Dynamic Points Removal Benchmark in Point Cloud Maps}, 
  year={2023},
  pages={608-614},
  doi={10.1109/ITSC57777.2023.10422094}
}
@article{daniel2024dufomap,
  author={Duberg, Daniel and Zhang, Qingwen and Jia, Mingkai and Jensfelt, Patric},
  journal={IEEE Robotics and Automation Letters}, 
  title={{DUFOMap}: Efficient Dynamic Awareness Mapping}, 
  year={2024},
  volume={9},
  number={6},
  pages={5038-5045},
  doi={10.1109/LRA.2024.3387658}
}
@article{schmid2023dynablox,
  title={Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments},
  author={Schmid, Lukas, and Andersson, Olov, and Sulser, Aurelio, and Pfreundschuh, Patrick, and Siegwart, Roland},
  booktitle={IEEE Robotics and Automation Letters (RA-L)},
  year={2023},
  volume={8},
  number={10},
  pages={6259 - 6266},
  doi={10.1109/LRA.2023.3305239}
}
```