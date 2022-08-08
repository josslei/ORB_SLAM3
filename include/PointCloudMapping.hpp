/**
* This file is part of ORB-SLAM3
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/
//
// Created by Joss Lei.
// Referred to Xiang Gao <gao.xiang.thu@gmail.com>, Xiaobainixi <741299292@qq.com>
// - Xiang Gao
//     - X. Gao, T. Zhang, Y. Liu, Q. Yan. Shijue SLAM shisi jiang: Conglilun daoshijian [Fourteen chapters of visual
//     SLAM: from theories to practice]. Publishing House of Electronics Industry, Beijing.
//     - ORB_SLAM2 With Point Cloud Map, by X. Gao. https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map
// - Xiaobainixi [https://github.com/xiaobainixi]
//     - ORB-SLAM2, RGBD Dense Map, https://github.com/xiaobainixi/ORB-SLAM2_RGBD_DENSE_MAP
//     NOTE: This code includes comments for ORB-SLAM2 and methods that handles loop closing.
//

#ifndef ORB_SLAM3_POINTCLOUDMAPPING_HPP
#define ORB_SLAM3_POINTCLOUDMAPPING_HPP

#include "System.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp>
#include <condition_variable>
#include <mutex>
#include <string>

namespace ORB_SLAM3 {

class PointCloudFragment
{
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
public:
    PointCloud::Ptr pc;

    Eigen::Isometry3d T;
    int pc_id;
}; // PointCloudFragment

class PointCloudMapping {
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double depth_map_factor, double resolution = 0.04, double meank = 50, double thresh = 1);
    void savePointCloud(const std::string &filename);
    void shutdown();

    void insertKeyframe(KeyFrame *kf, cv::Mat color, cv::Mat depth, int idk, std::vector<KeyFrame*> vpKFs);
    void loopClosingUpdate();
    void viewer();

    int loop_count;
    bool loop_busy;
    std::mutex loop_busy_mutex;

private:
    PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth);

    double depth_map_factor;
    double resolution;
    double meank;
    double thresh;
    pcl::VoxelGrid<PointT> voxel;
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;

    PointCloud::Ptr global_map;
    std::mutex global_map_mutex;

    std::shared_ptr<std::thread>  viewer_thread;

    bool shutdown_flag;
    std::mutex shutdown_flag_mutex;

    std::condition_variable keyframe_update;
    std::mutex keyframe_update_mutex;

    std::vector<PointCloudFragment> point_cloud;
    // data to generate point clouds
    std::vector<KeyFrame*>  current_vpKFs;
    std::vector<KeyFrame*>  keyframes;
    std::mutex              keyframe_mutex;
    uint16_t                last_keyframe_size;

    bool cloud_busy;
    std::mutex cloud_busy_mutex;

}; // PointCloudMapping

} // ORB_SLAM3

#endif //ORB_SLAM3_POINTCLOUDMAPPING_HPP
