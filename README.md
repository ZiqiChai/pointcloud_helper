# pointcloud_helper

This repo contains the following contents:

- Multi-View RGBD data fusion with Robot Pose data and Eye-hand-calibration data.
- Some PCL pointcloud process helpers, including ply to pcd transorm, as well as pcd to ply transform, and so on.

```bash
mkdir build
cd build
cmake ..
make
```

Put the `data` folder under `build` folder, the `data` folder contains:

- XXX.ply files
- names.txt
- robot_Cart_Pose.txt
- eyehand.txt

run:

```bash
cd data
../merge_pointcloud_icp ./names.txt ./robot_Cart_Pose.txt ./eyehand.txt
```

An example data.tar file is attached.

Warning:

- *all txt file should leave no blank line at the end of the file*
- *pay attention to all the path, or modify the path according to your need*
