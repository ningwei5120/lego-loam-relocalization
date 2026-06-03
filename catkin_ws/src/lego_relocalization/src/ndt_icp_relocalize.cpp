#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <thread>
#include <deque>

namespace lego_relocalization {

using PointType = pcl::PointXYZI;

enum LocalizeState {
    IDLE = 0,
    INITIALIZING = 1,
    TRACKING = 2,
    LOST = 3
};

class NdtIcpRelocalize {
public:
    NdtIcpRelocalize() : nh_("~"), state_(IDLE), has_velocity_(false),
                         local_map_valid_(false), lost_count_(0),
                         frame_count_(0), success_count_(0),
                         default_init_applied_(false) {
        initParams();
        initROS();
        loadGlobalMap();
        idle_start_time_ = ros::Time::now();
    }

    void spin() {
        std::thread map_thread(&NdtIcpRelocalize::publishMapThread, this);
        ros::WallRate rate(processing_rate_);  // WallRate: not blocked when sim time is paused
        while (ros::ok()) {
            ros::spinOnce();
            process();
            rate.sleep();
        }
        map_thread.join();
    }

private:
    void initParams() {
        nh_.param<std::string>("pcd_map_path", pcd_map_path_,
            "/home/hong/lego/map_results/finalCloud.pcd");
        nh_.param<std::string>("lidar_topic", lidar_topic_, "/velodyne_points");
        nh_.param<std::string>("initialpose_topic", initialpose_topic_, "/initialpose");
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<std::string>("lidar_frame", lidar_frame_, "camera_init");
        nh_.param<std::string>("odom_topic", odom_topic_, "/relocalization/odometry");

        nh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.4f);
        nh_.param<float>("ndt_resolution", ndt_resolution_, 1.0f);
        nh_.param<float>("ndt_epsilon", ndt_epsilon_, 0.01f);
        nh_.param<float>("icp_max_corr_dist", icp_max_corr_dist_, 30.0f);
        nh_.param<int>("icp_max_iterations", icp_max_iterations_, 100);
        nh_.param<float>("icp_fitness_threshold", icp_fitness_threshold_, 8.0f);
        nh_.param<float>("processing_rate", processing_rate_, 10.0f);

        // New params for performance and robustness
        nh_.param<float>("local_map_radius", local_map_radius_, 30.0f);
        nh_.param<bool>("use_local_map", use_local_map_, true);
        nh_.param<int>("tracking_icp_max_iter", tracking_icp_max_iter_, 20);
        nh_.param<float>("tracking_fitness_mult", tracking_fitness_mult_, 10.0f);
        nh_.param<int>("auto_reinit_max_lost", auto_reinit_max_lost_, 5);
        nh_.param<float>("auto_reinit_radius", auto_reinit_radius_, 20.0f);
        nh_.param<float>("max_tracking_jump", max_tracking_jump_, 10.0f);
        nh_.param<float>("tracking_icp_max_corr_dist", tracking_icp_max_corr_dist_, 5.0f);
        nh_.param<float>("reinit_fitness_threshold", reinit_fitness_threshold_, 15.0f);

        // Default initial pose settings
        nh_.param<bool>("use_default_initial_pose", use_default_initial_pose_, false);
        nh_.param<float>("default_init_x", default_init_x_, 0.0f);
        nh_.param<float>("default_init_y", default_init_y_, 0.0f);
        nh_.param<float>("default_init_z", default_init_z_, 0.0f);
        nh_.param<float>("default_init_roll", default_init_roll_, 0.0f);
        nh_.param<float>("default_init_pitch", default_init_pitch_, 0.0f);
        nh_.param<float>("default_init_yaw", default_init_yaw_, 0.0f);
        nh_.param<float>("default_init_timeout", default_init_timeout_, 5.0f);

        current_pose_ = Eigen::Matrix4f::Identity();
        initial_guess_ = Eigen::Matrix4f::Identity();
        prev_pose_ = Eigen::Matrix4f::Identity();
        velocity_transform_ = Eigen::Matrix4f::Identity();
        default_initial_guess_ = Eigen::Matrix4f::Identity();

        if (use_default_initial_pose_) {
            buildPoseMatrix(default_init_x_, default_init_y_, default_init_z_,
                           default_init_roll_, default_init_pitch_, default_init_yaw_,
                           default_initial_guess_);
            ROS_INFO("Default initial pose enabled: [%.2f, %.2f, %.2f] RPY:[%.2f, %.2f, %.2f] (timeout: %.1fs)",
                     default_init_x_, default_init_y_, default_init_z_,
                     default_init_roll_, default_init_pitch_, default_init_yaw_,
                     default_init_timeout_);
        }

        ROS_INFO("Parameters:");
        ROS_INFO("  voxel_leaf_size: %.2f", voxel_leaf_size_);
        ROS_INFO("  ndt_resolution: %.2f", ndt_resolution_);
        ROS_INFO("  icp_max_corr_dist: %.2f", icp_max_corr_dist_);
        ROS_INFO("  icp_max_iterations: %d", icp_max_iterations_);
        ROS_INFO("  icp_fitness_threshold: %.2f", icp_fitness_threshold_);
        ROS_INFO("  local_map_radius: %.1f", local_map_radius_);
        ROS_INFO("  use_local_map: %s", use_local_map_ ? "true" : "false");
        ROS_INFO("  tracking_icp_max_iter: %d", tracking_icp_max_iter_);
    }

    void initROS() {
        sub_lidar_ = nh_.subscribe(lidar_topic_, 2, &NdtIcpRelocalize::lidarCallback, this);
        sub_initialpose_ = nh_.subscribe(initialpose_topic_, 1, &NdtIcpRelocalize::initialposeCallback, this);
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
        pub_aligned_ = nh_.advertise<sensor_msgs::PointCloud2>("/relocalization/aligned_cloud", 1);
        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/relocalization/global_map", 1);
        pub_local_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/relocalization/local_map", 1);
    }

    void loadGlobalMap() {
        pcl::PointCloud<PointType>::Ptr cloud_raw(new pcl::PointCloud<PointType>());
        if (pcl::io::loadPCDFile<PointType>(pcd_map_path_, *cloud_raw) == -1) {
            ROS_ERROR("Failed to load PCD map: %s", pcd_map_path_.c_str());
            ros::shutdown();
            return;
        }

        ROS_INFO("Loaded raw global map: %zu points", cloud_raw->points.size());

        // Reverse coordinate swap: LeGO-LOAM featureAssociation applies (x,y,z)->(y,z,x)
        for (size_t i = 0; i < cloud_raw->points.size(); ++i) {
            float xs = cloud_raw->points[i].x;
            float ys = cloud_raw->points[i].y;
            float zs = cloud_raw->points[i].z;
            cloud_raw->points[i].x = zs;
            cloud_raw->points[i].y = xs;
            cloud_raw->points[i].z = ys;
        }

        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.setInputCloud(cloud_raw);
        voxel.filter(*global_map_);

        ROS_INFO("Downsampled global map: %zu points (leaf=%.2f)",
                 global_map_->points.size(), voxel_leaf_size_);

        // Compute map bounds
        if (!global_map_->empty()) {
            float min_x = 1e9, max_x = -1e9;
            float min_y = 1e9, max_y = -1e9;
            float min_z = 1e9, max_z = -1e9;
            for (const auto& pt : global_map_->points) {
                min_x = std::min(min_x, pt.x); max_x = std::max(max_x, pt.x);
                min_y = std::min(min_y, pt.y); max_y = std::max(max_y, pt.y);
                min_z = std::min(min_z, pt.z); max_z = std::max(max_z, pt.z);
            }
            ROS_INFO("Global map bounds: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
                     min_x, max_x, min_y, max_y, min_z, max_z);
        }
    }

    void extractLocalMap(const Eigen::Vector3f& center) {
        local_map_->clear();
        local_map_->reserve(50000);

        float r = local_map_radius_;
        float min_x = center(0) - r, max_x = center(0) + r;
        float min_y = center(1) - r, max_y = center(1) + r;
        float min_z = center(2) - r, max_z = center(2) + r;

        for (const auto& pt : global_map_->points) {
            if (pt.x >= min_x && pt.x <= max_x &&
                pt.y >= min_y && pt.y <= max_y &&
                pt.z >= min_z && pt.z <= max_z) {
                local_map_->points.push_back(pt);
            }
        }
        local_map_->width = local_map_->points.size();
        local_map_->height = 1;
        local_map_->is_dense = true;

        local_map_valid_ = (local_map_->points.size() >= 100);
        if (!local_map_valid_) {
            ROS_WARN_THROTTLE(5.0, "Local map too small: %zu points at [%.1f, %.1f, %.1f]",
                              local_map_->points.size(), center(0), center(1), center(2));
        }
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_cloud_);
        latest_cloud_msg_ = *msg;
        has_new_cloud_ = true;
    }

    static void buildPoseMatrix(float x, float y, float z,
                                float roll, float pitch, float yaw,
                                Eigen::Matrix4f& out) {
        out = Eigen::Matrix4f::Identity();
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation = (yawAngle * pitchAngle * rollAngle).matrix();
        out.block<3,3>(0,0) = rotation;
        out(0,3) = x;
        out(1,3) = y;
        out(2,3) = z;
    }

    void initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_pose_);

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        buildPoseMatrix(x, y, z, roll, pitch, yaw, initial_guess_);

        ROS_INFO("Received initial pose: [%.2f, %.2f, %.2f] RPY:[%.2f, %.2f, %.2f]",
                 x, y, z, roll, pitch, yaw);

        state_ = INITIALIZING;
        lost_count_ = 0;
        default_init_applied_ = true;
        // Reset idle timer so manual input takes precedence
        idle_start_time_ = ros::Time::now();
    }

    Eigen::Matrix4f predictPose(float dt) {
        if (!has_velocity_ || dt <= 0) {
            return current_pose_;
        }
        // Exponential prediction: T_pred = T_curr * (T_prev^{-1} * T_curr)^n
        // For simplicity, linear extrapolation in SE(3) using velocity transform
        // T_pred = T_curr * exp(log(velocity_transform_) * dt_factor)
        // Simpler: T_pred = T_curr * velocity_transform_ (assuming unit time step)
        // Even simpler for small dt: approximate with constant velocity
        return current_pose_ * velocity_transform_;
    }

    void updateVelocity(const ros::Time& current_time) {
        if (prev_time_.isZero()) {
            prev_time_ = current_time;
            prev_pose_ = current_pose_;
            return;
        }
        float dt = (current_time - prev_time_).toSec();
        if (dt > 0.01f && dt < 2.0f) {
            velocity_transform_ = prev_pose_.inverse() * current_pose_;
            has_velocity_ = true;
        }
        prev_time_ = current_time;
        prev_pose_ = current_pose_;
    }

    void process() {
        // Check default initial pose timeout even when no new cloud arrives
        if (state_ == IDLE && use_default_initial_pose_ && !default_init_applied_) {
            float elapsed = (ros::Time::now() - idle_start_time_).toSec();
            if (elapsed >= default_init_timeout_) {
                ROS_INFO("Timeout (%.1fs) reached. Using default initial pose: [%.2f, %.2f, %.2f] yaw=%.2f",
                         default_init_timeout_, default_init_x_, default_init_y_, default_init_z_, default_init_yaw_);
                initial_guess_ = default_initial_guess_;
                state_ = INITIALIZING;
                default_init_applied_ = true;
            }
        }

        if (!has_new_cloud_) return;

        sensor_msgs::PointCloud2 cloud_msg;
        {
            std::lock_guard<std::mutex> lock(mtx_cloud_);
            cloud_msg = latest_cloud_msg_;
            has_new_cloud_ = false;
        }

        pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloud_msg, *cloud_in);

        if (cloud_in->empty()) {
            ROS_WARN_THROTTLE(5.0, "Empty input cloud");
            return;
        }

        // Downsample input
        pcl::PointCloud<PointType>::Ptr cloud_ds(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.setInputCloud(cloud_in);
        voxel.filter(*cloud_ds);

        frame_count_++;
        Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
        bool success = false;
        float fitness_score = 999.0f;

        if (state_ == IDLE) {
            // Default initial pose timeout is checked at the start of process()
            ROS_INFO_THROTTLE(5.0, "Waiting for /initialpose to start relocalization...");
            return;
        }
        else if (state_ == INITIALIZING) {
            success = ndtIcpAlign(cloud_ds, initial_guess_, result_pose, fitness_score);
            if (success) {
                ROS_INFO("Initialization succeed! Fitness: %.4f", fitness_score);
                state_ = TRACKING;
                current_pose_ = result_pose;
                extractLocalMap(current_pose_.block<3,1>(0,3));
                updateVelocity(cloud_msg.header.stamp);
                success_count_++;
            } else {
                ROS_WARN("Initialization failed! Fitness: %.4f. Please provide a new initial pose.", fitness_score);
                state_ = IDLE;
                idle_start_time_ = ros::Time::now();
                default_init_applied_ = false;
                return;
            }
        }
        else if (state_ == TRACKING) {
            // Predict pose using constant velocity model
            float dt = 0.1f; // assume ~10Hz
            if (!prev_time_.isZero()) {
                dt = (cloud_msg.header.stamp - prev_time_).toSec();
            }
            Eigen::Matrix4f predicted_pose = predictPose(dt);

            // Optionally update local map if we've moved too far
            Eigen::Vector3f current_pos = current_pose_.block<3,1>(0,3);
            Eigen::Vector3f predicted_pos = predicted_pose.block<3,1>(0,3);
            float move_dist = (predicted_pos - current_pos).norm();

            if (use_local_map_ && (move_dist > local_map_radius_ * 0.3f || !local_map_valid_)) {
                extractLocalMap(predicted_pos);
                ROS_INFO_THROTTLE(2.0, "Updated local map: %zu points", local_map_->points.size());
            }

            // DEBUG: always use global map for tracking to diagnose local map issue
            pcl::PointCloud<PointType>::Ptr target_map = global_map_;
            ROS_DEBUG("Using global map for tracking: %zu points", target_map->points.size());

            success = icpAlign(cloud_ds, target_map, predicted_pose, result_pose, fitness_score, tracking_icp_max_iter_, tracking_icp_max_corr_dist_);

            float tracking_threshold = icp_fitness_threshold_ * tracking_fitness_mult_;

            if (!success || fitness_score > tracking_threshold) {
                lost_count_++;
                ROS_WARN("Tracking lost! Fitness: %.4f (threshold: %.2f). Lost count: %d/%d",
                         fitness_score, tracking_threshold, lost_count_, auto_reinit_max_lost_);

                if (lost_count_ >= auto_reinit_max_lost_) {
                    ROS_WARN("Max lost count reached. Switching to LOST state.");
                    state_ = LOST;
                    has_velocity_ = false;
                }
                return;
            }

            // Check for large jump
            float pose_jump = (result_pose.block<3,1>(0,3) - current_pose_.block<3,1>(0,3)).norm();
            if (pose_jump > max_tracking_jump_) {
                ROS_WARN("Large pose jump detected: %.2f m. Rejecting this frame.", pose_jump);
                lost_count_++;
                if (lost_count_ >= auto_reinit_max_lost_) {
                    state_ = LOST;
                }
                return;
            }

            lost_count_ = 0;
            current_pose_ = result_pose;
            updateVelocity(cloud_msg.header.stamp);
            success_count_++;
        }
        else if (state_ == LOST) {
            lost_count_++;
            ROS_WARN_THROTTLE(5.0, "Localization lost. Waiting for /initialpose... (lost frames: %d)", lost_count_);

            // If default initial pose is enabled, try it after some lost frames
            if (use_default_initial_pose_ && lost_count_ >= auto_reinit_max_lost_) {
                ROS_INFO("Auto reinitialization attempt using default initial pose...");
                success = ndtIcpAlign(cloud_ds, default_initial_guess_, result_pose, fitness_score, reinit_fitness_threshold_);
                if (success) {
                    ROS_INFO("Auto reinitialization succeed with default pose! Fitness: %.4f", fitness_score);
                    state_ = TRACKING;
                    current_pose_ = result_pose;
                    extractLocalMap(current_pose_.block<3,1>(0,3));
                    updateVelocity(cloud_msg.header.stamp);
                    lost_count_ = 0;
                    success_count_++;
                    return;
                }
                // Fall through to last-pose-based reinitialization if default fails
            }

            // Auto reinitialization attempt using last known pose
            if (lost_count_ >= auto_reinit_max_lost_ && lost_count_ % auto_reinit_max_lost_ == 0) {
                ROS_INFO("Auto reinitialization attempt using last pose...");
                // Add some random perturbation around last pose
                Eigen::Matrix4f reinit_guess = current_pose_;
                // Small noise in position
                reinit_guess(0,3) += ((rand() % 100) - 50) / 50.0f * auto_reinit_radius_;
                reinit_guess(1,3) += ((rand() % 100) - 50) / 50.0f * auto_reinit_radius_;

                success = ndtIcpAlign(cloud_ds, reinit_guess, result_pose, fitness_score, reinit_fitness_threshold_);
                if (success) {
                    ROS_INFO("Auto reinitialization succeed! Fitness: %.4f", fitness_score);
                    state_ = TRACKING;
                    current_pose_ = result_pose;
                    extractLocalMap(current_pose_.block<3,1>(0,3));
                    updateVelocity(cloud_msg.header.stamp);
                    lost_count_ = 0;
                    success_count_++;
                } else {
                    ROS_WARN("Auto reinitialization failed. Fitness: %.4f", fitness_score);
                    return;
                }
            } else {
                return;
            }
        }

        publishResult(cloud_msg.header.stamp, cloud_ds);

        // Periodic stats
        if (frame_count_ % 100 == 0) {
            ROS_INFO("Stats: frames=%d, successes=%d, rate=%.1f%%",
                     frame_count_, success_count_, 100.0f * success_count_ / frame_count_);
        }
    }

    bool ndtIcpAlign(const pcl::PointCloud<PointType>::Ptr& source,
                     const Eigen::Matrix4f& initial_guess,
                     Eigen::Matrix4f& result,
                     float& fitness_score,
                     float custom_threshold = -1.0f) {
        ros::WallTime t_start = ros::WallTime::now();

        // Stage 1: NDT coarse alignment
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(ndt_epsilon_);
        ndt.setResolution(ndt_resolution_);
        ndt.setInputSource(source);
        ndt.setInputTarget(global_map_);

        pcl::PointCloud<PointType>::Ptr ndt_result(new pcl::PointCloud<PointType>());
        ndt.align(*ndt_result, initial_guess);

        if (!ndt.hasConverged()) {
            ROS_WARN("NDT did not converge");
        }

        // Stage 2: ICP fine alignment
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
        icp.setMaximumIterations(icp_max_iterations_);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);  // Disable RANSAC: structured LiDAR points don't benefit from it
        icp.setInputSource(source);
        icp.setInputTarget(global_map_);

        pcl::PointCloud<PointType>::Ptr icp_result(new pcl::PointCloud<PointType>());
        icp.align(*icp_result, ndt.getFinalTransformation());

        result = icp.getFinalTransformation();
        fitness_score = icp.getFitnessScore();

        ros::WallTime t_end = ros::WallTime::now();
        float threshold = (custom_threshold > 0) ? custom_threshold : icp_fitness_threshold_;
        ROS_INFO("NDT+ICP: converged=%s fitness=%.4f threshold=%.2f time=%.3fs",
                 icp.hasConverged() ? "yes" : "no",
                 fitness_score, threshold,
                 (t_end - t_start).toSec());

        return icp.hasConverged() && fitness_score < threshold;
    }

    bool icpAlign(const pcl::PointCloud<PointType>::Ptr& source,
                  const pcl::PointCloud<PointType>::Ptr& target,
                  const Eigen::Matrix4f& initial_guess,
                  Eigen::Matrix4f& result,
                  float& fitness_score,
                  int max_iterations,
                  float max_corr_dist = -1.0f) {
        ros::WallTime t_start = ros::WallTime::now();

        float corr_dist = (max_corr_dist > 0) ? max_corr_dist : icp_max_corr_dist_;
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(corr_dist);
        icp.setMaximumIterations(max_iterations);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);  // Disable RANSAC: structured LiDAR points don't benefit from it
        icp.setInputSource(source);
        icp.setInputTarget(target);

        pcl::PointCloud<PointType>::Ptr icp_result(new pcl::PointCloud<PointType>());
        icp.align(*icp_result, initial_guess);

        result = icp.getFinalTransformation();
        fitness_score = icp.getFitnessScore();

        ros::WallTime t_end = ros::WallTime::now();
        ROS_DEBUG("ICP: converged=%s fitness=%.4f time=%.3fs iter=%d",
                  icp.hasConverged() ? "yes" : "no",
                  fitness_score,
                  (t_end - t_start).toSec(),
                  max_iterations);

        return icp.hasConverged();
    }

    void publishMapThread() {
        publishGlobalMapOnce();
        ros::WallRate rate(0.2); // every 5 seconds
        while (ros::ok()) {
            rate.sleep();
            publishGlobalMapOnce();
            if (use_local_map_ && local_map_valid_) {
                publishLocalMapOnce();
            }
        }
    }

    void publishGlobalMapOnce() {
        if (global_map_->empty()) return;
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        ros::Time stamp = ros::Time::now();
        if (stamp.isZero()) {
            ros::WallTime wt = ros::WallTime::now();
            stamp = ros::Time(wt.sec, wt.nsec);
        }
        map_msg.header.stamp = stamp;
        map_msg.header.frame_id = map_frame_;
        pub_map_.publish(map_msg);
        ROS_INFO_ONCE("Global map published (%dx%d, frame=%s)",
                      map_msg.width, map_msg.height, map_msg.header.frame_id.c_str());
    }

    void publishLocalMapOnce() {
        if (local_map_->empty()) return;
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*local_map_, map_msg);
        ros::Time stamp = ros::Time::now();
        if (stamp.isZero()) {
            ros::WallTime wt = ros::WallTime::now();
            stamp = ros::Time(wt.sec, wt.nsec);
        }
        map_msg.header.stamp = stamp;
        map_msg.header.frame_id = map_frame_;
        pub_local_map_.publish(map_msg);
    }

    void publishResult(const ros::Time& stamp, const pcl::PointCloud<PointType>::Ptr& aligned_cloud) {
        Eigen::Matrix3f rot = current_pose_.block<3,3>(0,0);
        Eigen::Vector3f trans = current_pose_.block<3,1>(0,3);

        Eigen::Vector3f euler = rot.eulerAngles(2, 1, 0); // ZYX order: yaw, pitch, roll
        double yaw = euler(0);
        double pitch = euler(1);
        double roll = euler(2);

        // Publish Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = lidar_frame_;
        odom.pose.pose.position.x = trans(0);
        odom.pose.pose.position.y = trans(1);
        odom.pose.pose.position.z = trans(2);

        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        pub_odom_.publish(odom);

        // Publish TF
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(trans(0), trans(1), trans(2)));
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, map_frame_, lidar_frame_));

        // Publish aligned cloud for visualization
        // Transform the raw input cloud to map frame using current_pose_
        // so that it visually aligns with the global map in RViz.
        pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*aligned_cloud, *cloud_in_map, current_pose_);
        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(*cloud_in_map, aligned_msg);
        aligned_msg.header.stamp = stamp;
        aligned_msg.header.frame_id = map_frame_;
        pub_aligned_.publish(aligned_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_initialpose_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_aligned_;
    ros::Publisher pub_map_;
    ros::Publisher pub_local_map_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::string pcd_map_path_;
    std::string lidar_topic_;
    std::string initialpose_topic_;
    std::string map_frame_;
    std::string lidar_frame_;
    std::string odom_topic_;

    float voxel_leaf_size_;
    float ndt_resolution_;
    float ndt_epsilon_;
    float icp_max_corr_dist_;
    int icp_max_iterations_;
    float icp_fitness_threshold_;
    float processing_rate_;

    float local_map_radius_;
    bool use_local_map_;
    int tracking_icp_max_iter_;
    float tracking_fitness_mult_;
    int auto_reinit_max_lost_;
    float auto_reinit_radius_;
    float max_tracking_jump_;
    float tracking_icp_max_corr_dist_;
    float reinit_fitness_threshold_;

    pcl::PointCloud<PointType>::Ptr global_map_{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr local_map_{new pcl::PointCloud<PointType>()};
    bool local_map_valid_;

    std::mutex mtx_cloud_;
    std::mutex mtx_pose_;
    sensor_msgs::PointCloud2 latest_cloud_msg_;
    bool has_new_cloud_ = false;

    LocalizeState state_;
    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f initial_guess_;
    Eigen::Matrix4f prev_pose_;
    ros::Time prev_time_;
    Eigen::Matrix4f velocity_transform_;
    bool has_velocity_;

    // Default initial pose support
    bool use_default_initial_pose_;
    float default_init_x_, default_init_y_, default_init_z_;
    float default_init_roll_, default_init_pitch_, default_init_yaw_;
    float default_init_timeout_;
    Eigen::Matrix4f default_initial_guess_;
    ros::Time idle_start_time_;
    bool default_init_applied_;

    int lost_count_;
    int frame_count_;
    int success_count_;
};

} // namespace lego_relocalization

int main(int argc, char** argv) {
    ros::init(argc, argv, "ndt_icp_relocalize");
    ROS_INFO("Starting NDT+ICP 3D Relocalization Node (optimized v2)...");
    lego_relocalization::NdtIcpRelocalize node;
    node.spin();
    return 0;
}
