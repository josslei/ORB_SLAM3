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

#include "PointCloudMapping.hpp"

namespace ORB_SLAM3 {
    PointCloudMapping::PointCloudMapping(double depth_map_factor_, double resolution_, double meank_, double thresh_)
    {
        this->depth_map_factor = depth_map_factor_;
        this->resolution = resolution_;
        this->meank = meank_;
        this->thresh = thresh_;
        statistical_filter.setMeanK(this->meank);
        statistical_filter.setStddevMulThresh(this->thresh);
        voxel.setLeafSize( this->resolution, this->resolution, this->resolution);

        this->global_map = std::make_shared<PointCloud>();
        this->viewer_thread = std::make_shared<thread>(std::bind(&PointCloudMapping::viewer, this));

        this->shutdown_flag         = false;
        this->point_cloud           = std::vector<PointCloudFragment>();
        this->keyframes             = std::vector<KeyFrame*>();
        this->last_keyframe_size    = 0;

        this->current_vpKFs         = std::vector<KeyFrame*>();
        this->loop_busy             = false;
        this->cloud_busy            = false;
        this->loop_count            = 0;

        Verbose::PrintMess("PointCloudMapping module. Successively initiated.\n", Verbose::VERBOSITY_NORMAL);
    }

    void PointCloudMapping::savePointCloud(const std::string &filename)
    {
        std::unique_lock<std::mutex> lck_global_map(this->global_map_mutex);
        PointCloud::Ptr tmp(new PointCloud);
        this->voxel.setInputCloud(this->global_map);
        this->voxel.filter(*tmp);
        pcl::io::savePCDFileBinary(filename, *(tmp));
        Verbose::PrintMess("PCD file saved to \"" + filename + "\"\n", Verbose::VERBOSITY_NORMAL);

        Verbose::PrintMess("Saving keyframse to ~/Keyframs/", Verbose::VERBOSITY_QUIET);
        std::ofstream fposes("/home/josslei/Keyframes/poses.txt");
        for (int i = 0; i < this->kf_colors.size(); ++i)
        {
            // save color and depth images
            cv::imwrite(std::string("/home/josslei/Keyframes/color/") + std::to_string(i) + string(".png"), this->kf_colors[i]);
            cv::imwrite(std::string("/home/josslei/Keyframes/depth/") + std::to_string(i) + string(".png"), this->kf_depths[i]);
            // save poses
            Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(this->keyframes[i]->GetPoseInverse());
            fposes << T.matrix() << "\n\n";
            //this->current_vpKFs[i]->GetPose()
        }
    }

    void PointCloudMapping::shutdown()
    {
        {
            std::unique_lock<std::mutex> lck(this->shutdown_flag_mutex);
            this->shutdown_flag = true;
            this->keyframe_update.notify_one();
        }
        this->viewer_thread->join();
        Verbose::PrintMess("PointCloudMapping module. Successively shutdown.\n", Verbose::VERBOSITY_NORMAL);
    }

    void PointCloudMapping::insertKeyframe(KeyFrame *kf, cv::Mat color, cv::Mat depth, int idk, std::vector<KeyFrame*> vpKFs)
    {
        Verbose::PrintMess("Keyframe received, idk = " + std::to_string(idk) + ", keyframe id = " + std::to_string(kf->mnId),
                           Verbose::VERBOSITY_NORMAL);
        std::unique_lock<std::mutex> lck(this->keyframe_mutex);
        this->keyframes.push_back(kf);
        this->current_vpKFs = vpKFs;
        this->kf_colors.push_back(color);
        this->kf_depths.push_back(depth);

        PointCloudFragment pointcloudfrg;
        pointcloudfrg.pc_id = idk;
        pointcloudfrg.T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
        pointcloudfrg.pc = this->generatePointCloud(kf, color, depth);
        this->point_cloud.push_back(pointcloudfrg);

        //cout << "==========\nKeyfram id: " << kf->mnId << "\nPTM Matrix:\n"<< pointcloudfrg.T.inverse().matrix()
        //     << "\nORB_SLAM# Matrix:\n" << kf->GetPoseInverse().matrix()
        //     << "\n=========\n";

        this->keyframe_update.notify_one();
    }

    void PointCloudMapping::loopClosingUpdate()
    {
        {
            std::unique_lock<std::mutex> lck_cloud_busy(this->cloud_busy_mutex);
            std::unique_lock<std::mutex> lck_loop_busy(this->loop_busy_mutex);
            if (!this->cloud_busy)
            {
                lck_cloud_busy.unlock();
                this->loop_busy = true;
                Verbose::PrintMess("Start looping map points.\n", Verbose::VERBOSITY_NORMAL);
                PointCloud::Ptr tmp1(new PointCloud);
                for (int i = 0; i < this->current_vpKFs.size(); ++i)
                {
                    for (int j = 0; j < this->point_cloud.size(); ++j)
                    {
                        if(this->point_cloud[j].pc_id == this->current_vpKFs[i]->mnFrameId)
                        {
                            // Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(this->keyframes[i]->GetPose());
                            Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(this->current_vpKFs[i]->GetPose());
                            PointCloud::Ptr cloud(new PointCloud);
                            pcl::transformPointCloud(*this->point_cloud[j].pc, *cloud, T.inverse().matrix());
                            *tmp1 += *cloud;
                            continue;
                        }
                    }
                }
                Verbose::PrintMess("Looping done.\n", Verbose::VERBOSITY_NORMAL);
                voxel.setInputCloud(tmp1);
                voxel.filter(*(this->global_map));
                this->loop_busy = false;
                ++(this->loop_count);
            }
        }
    }

    void PointCloudMapping::viewer()
    {
        pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
        //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        while (true)
        {
            {
                std::unique_lock<std::mutex> lck_shutdown(this->shutdown_flag_mutex);
                if (this->shutdown_flag)
                    break;
            }
            {
                std::unique_lock<std::mutex> lck_keyframe_update(this->keyframe_update_mutex);
                Verbose::PrintMess("Waiting for keyframe update...\n", Verbose::VERBOSITY_NORMAL);
                this->keyframe_update.wait(lck_keyframe_update);
                Verbose::PrintMess("Received keyframe update signal.\n", Verbose::VERBOSITY_NORMAL);
            }

            // Keyframe is updated
            size_t N = 0;
            {
                std::unique_lock<std::mutex> lck(this->keyframe_mutex);
                N = this->keyframes.size();
                // N = this->current_vpKFs.size();
            }
            {
                std::unique_lock<std::mutex> lck_loop_busy(this->loop_busy_mutex);
                if (this->loop_busy)
                    continue;
            }
            {
                std::unique_lock<std::mutex> lck_cloud_busy(this->cloud_busy_mutex);
                this->cloud_busy = true;
                for (size_t i = this->last_keyframe_size; i < N; ++i) {
                    PointCloud::Ptr p(new PointCloud);
                    pcl::transformPointCloud(*(this->point_cloud)[i].pc, *p, this->point_cloud[i].T.inverse().matrix());
                    *(this->global_map) += *p;
                }
                // Depth filter and statistical removal
                PointCloud::Ptr tmp1(new PointCloud);
                statistical_filter.setInputCloud(this->global_map);
                statistical_filter.filter(*tmp1);

                // voxel.setInputCloud(tmp1);
                // voxel.filter(*(this->global_map));
                this->global_map->swap(*tmp1);

                //viewer.showCloud(this->point_cloud[this->point_cloud.size() - 1].pc);
                viewer.showCloud(this->global_map);
                Verbose::PrintMess("Showing global map, size = " + std::to_string(N) + ", " + std::to_string(this->global_map->points.size()) + '\n',
                                   Verbose::VERBOSITY_NORMAL);
                this->last_keyframe_size = N;
                this->cloud_busy = false;
            }
        }
    }

    void PointCloudMapping::regeneratePointCloud()
    {
        cout << "Starting to regenerate...\n";
        this->global_map = std::make_shared<PointCloud>();

        //vector<KeyFrame*> vpKFs = this->current_vpKFs;
        vector<KeyFrame*> vpKFs = this->keyframes;
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if(pKF->isBad())
                continue;
            if(i % 5 != 0)
                continue;

            Sophus::SE3f Twc = pKF->GetPoseInverse();

            PointCloudFragment pointcloudfrg;
            pointcloudfrg.pc = this->generatePointCloud(pKF, this->kf_colors[i], this->kf_depths[i]);

            PointCloud::Ptr p(new PointCloud);
            pcl::transformPointCloud(*(this->point_cloud)[i].pc, *p, Twc.matrix());
            *(this->global_map) += *p;
        }
        PointCloud::Ptr tmp1(new PointCloud);
        statistical_filter.setInputCloud(this->global_map);
        statistical_filter.filter(*tmp1);
        this->global_map->swap(*tmp1);

        /*
        // The first keyframe as the origin
        auto Two = this->current_vpKFs[0]->GetPoseInverse();

        // Iterate all Keyframes

        // List of keyframes
        auto lRit = mpTracker->mlpReferences.begin();//list<ORB_SLAM3::KeyFrame*>::iterator;
        // List of relative poses
        auto lit = mpTracker->mlRelativeFramePoses.begin();
        auto lend = mpTracker->mlRelativeFramePoses.end();
        cout << "Iteration start...\n";
        cout << this->kf_colors.size() << '\n';
        for (int i = 0; lit != lend; ++lit, ++lRit, ++i)
        {
            cout << "Index: " << i << '\n';
            KeyFrame *pKF = *lRit;
            Sophus::SE3f Trw;
            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while(pKF->isBad())
            {
                cout << "Keyframe " << pKF->mnId << " is bad!\n";
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }
            cout << "Stage 1\n";
            Trw = Trw * pKF->GetPose() * Two;
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            if (i >= this->kf_colors.size())
                continue;

            cout << "Stage 2\n";
            PointCloudFragment pointcloudfrg;
            pointcloudfrg.pc = this->generatePointCloud(pKF, this->kf_colors[i], this->kf_depths[i]);

            cout << "Stage 3\n";
            PointCloud::Ptr p(new PointCloud);
            pcl::transformPointCloud(*(this->point_cloud)[i].pc, *p, Twc.matrix());
            cout << "Stage 4\n";
            *(this->global_map) += *p;
            cout << "<<<<<<<<<<<<<<\ni: " << i << "\n<<<<<<<<<<<<<<\n";
        }
        // Depth filter and statistical removal
        PointCloud::Ptr tmp1(new PointCloud);
        statistical_filter.setInputCloud(this->global_map);
        statistical_filter.filter(*tmp1);
        this->global_map->swap(*tmp1);
        */
    }

    PointCloudMapping::PointCloud::Ptr PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth)
    {
        //cv::imshow("Color", color);
        //cv::imshow("Depth", depth);
        PointCloud::Ptr tmp(new PointCloud());
        for (int m = 0; m < depth.rows; ++m)
        {
            for (int n = 0; n < depth.cols; ++n)
            {
                unsigned short d_raw = depth.ptr<unsigned short>(m)[n];
                double d = (double)d_raw / this->depth_map_factor;
                if (d < 0.01 || d > 5)
                    continue;
                PointT p;
                p.z = d;
                p.x = (n - kf->cx) * p.z / kf->fx;
                p.y = (m - kf->cy) * p.z / kf->fy;

                p.b = color.ptr<uchar>(m)[n * 3];
                p.g = color.ptr<uchar>(m)[n * 3 + 1];
                p.r = color.ptr<uchar>(m)[n * 3 + 2];

                tmp->points.push_back(p);
            }
        }
        return tmp;
    }
} // ORB_SLAM3
