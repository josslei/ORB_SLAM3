# Requirements

**Operating System:**
 - WSL on Windows 10
 - Ubuntu 20.04
 - VcXsrv

**ROS Version:** ROS Noetic Ninjemys

**Packages to install using apt:**
 - libboost1.71-dev
 - libvtk6.3 (default for ROS Noetic Ninjemys)

**Libraries to compile from sources:**

| Libraries | Version |
|-----------|---------|
| Eigen     | 3.3.0   |
| OpenCV    | 4.2.0   |
| Pangolin  | 0.8     |
| PCL       | 1.12.1  |

**NOTE:** For Pangolin, specifically, it is the commit on [Mon May 30 15:12:01 2022 +0100] [20792418d83334c6f89fb0b9c91ce5b71c30f4db]

**NOTE:** For PCL, specifically, it is the commit on [Wed Aug 10 10:07:05 2022 +0200] [a92da8b793cda3567b89ce86b2f9ed9215e1918d]

# Build & Run

1. Set Xwindow display
```bash
# To display GUI when running WSL, set the display for Xwindow
# To check the IP of your WSL, invoke [ipconfig] under Windows PowerShell
#
# Temporary setting
$ export DISPLAY=${your_wsl_ip}:0.0
# Or append it to .bashrc
$ echo "export DISPLAY=${your_wsl_ip}:0.0" >> ~/.bashrc
```

2. Config ROS
```bash
# Make sure the built binary executable path is in $ROS_PACKAGE_PATH
#
# Temporary setting
$ export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${your_path_to_repository}/Environments/ScenePerception/SLAM/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3
# Or append it to .bashrc
$ echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${your_path_to_repository}/Environments/ScenePerception/SLAM/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3" >> ~/.bashrc
```

3. Build ORB_SLAM3 and ROS examples
```bash
$ cd ORB_SLAM3
$ ./build.sh
$ ./build_ros.sh
```

4. Running RGB-D example with point cloud
```bash
# Create a new terminal to start an ROS daemon
$ roscore
# Start RGB-D example
$ rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt ./d435_with-point-cloud.yaml
# Create a new terminal to feed your data
$ rosbag play path_to_your_data.bag
```

# Post Running

You will receive `ORB_SLAM3/KeyFrameTrajectory.txt` that is originally provided by ORB_SLAM3. The point cloud will be
defaultingly saved to `ORB_SLAM3/PointCloud.pcd`; you can change this by editing
`ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc`.

NOTE: You can change the resolution of the point cloud by editing `PointCloudMapping.Resolution` in
`ORB_SLAM3/d435_with-point-cloud.yaml`
