#~/usr/bin/bash

cd build
make -j6 ORB_SLAM3
cd ../Examples_old/ROS/ORB_SLAM3/build
make RGBD
cd ../../../../
rosrun ORB_SLAM3 RGBD Vocabulary/ORBvoc.txt d435_with-point-cloud.yaml

