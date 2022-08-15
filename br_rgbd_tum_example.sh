#!/usr/bin/sh

# clean
rm ./Examples/RGB-D/rgbd_tum

cd build
#cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6 rgbd_tum
cd ..

echo Running TUM example...
#gdb --args ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1_WithPointCloud.yaml /mnt/ext/dataset-slam/rgbd_dataset_freiburg1_room/ ./Examples/RGB-D/associations/fr1_room.txt
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1_WithPointCloud.yaml /mnt/ext/dataset-slam/rgbd_dataset_freiburg1_room/ ./Examples/RGB-D/associations/fr1_room.txt > log.txt 2> log_err.txt
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1_WithPointCloud.yaml /mnt/ext/dataset-slam/rgbd_dataset_freiburg1_room/ ./Examples/RGB-D/associations/fr1_room.txt
