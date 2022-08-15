#!/usr/bin/sh

cd build
make rgbd_lab
cd ..
sh ./run_rgbd_tum_example.sh
