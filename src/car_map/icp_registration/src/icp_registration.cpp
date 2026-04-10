#include "icp_registration/icp_registration.hpp"

#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>
#include <cmath>

namespace icp
{

    IcpNode::IcpNode(const rclcpp::NodeOptions &options)
        : Node("icp_registration", options),
          rough_iter_(10),
          refine_iter_(5),
          stop_thread_(false),
          is_ready_(false)
    {
        cloud_in_ = std::make_shared<PointCloudXYZI>();

        // ---------------- 参数读取 ----------------
        pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
        pointcloud_topic_ = this->declare_parameter("pointcloud_topic", std::string("/livox/lidar/pointcloud"));
        map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
        odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("odom"));
        laser_frame_id_ = this->declare_parameter("laser_frame_id", std::string("body"));

        double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
        double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
        thresh_ = this->declare_parameter("thresh", 0.15);
        tf_rate_ = this->declare_parameter("tf_rate", 50);

        xy_offset_ = this->declare_parameter("xy_offset", 0.2);
        yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
        yaw_resolution_ = this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;

        initial_x_ = this->declare_parameter<double>("init_pose.x", 0.0);
        initial_y_ = this->declare_parameter<double>("init_pose.y", 0.0);
        initial_z_ = this->declare_parameter<double>("init_pose.z", 0.0);
        initial_qx_ = this->declare_parameter<double>("init_pose.qx", 0.0);
        initial_qy_ = this->declare_parameter<double>("init_pose.qy", 0.0);
        initial_qz_ = this->declare_parameter<double>("init_pose.qz", 0.0);
        initial_qw_ = this->declare_parameter<double>("init_pose.qw", 1.0);

        RCLCPP_INFO(this->get_logger(), ">>> [ICPParams] Configuration <<<");
        RCLCPP_INFO(this->get_logger(), "  > PCD Path: %s", pcd_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  > Topic:    %s", pointcloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  > Frames:   Map=[%s] -> Odom=[%s] -> Laser=[%s]",
                    map_frame_id_.c_str(), odom_frame_id_.c_str(), laser_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  > Filter:   Rough Leaf=%.2f m, Refine Leaf=%.2f m",
                    rough_leaf_size, refine_leaf_size);
        RCLCPP_INFO(this->get_logger(), "  > Thresh:   Score Threshold=%.4f", thresh_);
        RCLCPP_INFO(this->get_logger(), "  > Search:   XY Offset=%.2f m", xy_offset_);
        RCLCPP_INFO(this->get_logger(), "  > Search:   Yaw Offset=%.4f rad, Resolution=%.4f rad",
                    yaw_offset_, yaw_resolution_);
        RCLCPP_INFO(this->get_logger(), "  > InitPose: x=%.2fm, y=%.2fm, z=%.2fm, qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f",
                    initial_x_, initial_y_, initial_z_, initial_qx_, initial_qy_, initial_qz_, initial_qw_);
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");

        // ---------------- 加载与处理地图 ----------------
        if (!std::filesystem::exists(pcd_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
            throw std::runtime_error("Invalid pcd path");
        }

        pcl::PCDReader reader;
        auto cloud = std::make_shared<PointCloudXYZI>();
        reader.read(pcd_path_, *cloud);
        pcl::toROSMsg(*cloud, map_ros_msg_);

        // 滤波与法向量计算
        voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size, refine_leaf_size);
        voxel_refine_filter_.setInputCloud(cloud);
        voxel_refine_filter_.filter(*cloud);
        refine_map_ = addNorm(cloud);

        auto filtered_point_rough = std::make_shared<PointCloudXYZI>();
        voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size, rough_leaf_size);
        voxel_rough_filter_.setInputCloud(cloud);
        voxel_rough_filter_.filter(*filtered_point_rough);
        rough_map_ = addNorm(filtered_point_rough);

        icp_rough_.setMaximumIterations(rough_iter_);
        icp_rough_.setInputTarget(rough_map_);
        icp_refine_.setMaximumIterations(refine_iter_);
        icp_refine_.setInputTarget(refine_map_);

        // ---------------- 订阅与发布 ----------------
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                pointcloudCallback(msg);
            });

        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
            {
                initialPoseCallback(msg);
            });

        // 状态发布器 (latching = true)
        status_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/localization_ready",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/icp_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        map_ros_msg_.header.frame_id = map_frame_id_;
        map_pub_->publish(map_ros_msg_);

        // ---------------- TF 初始化 ----------------
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            map_to_odom_.header.frame_id = map_frame_id_;
            map_to_odom_.child_frame_id = odom_frame_id_;
            map_to_odom_.transform.translation.x = initial_x_;
            map_to_odom_.transform.translation.y = initial_y_;
            map_to_odom_.transform.translation.z = initial_z_;
            map_to_odom_.transform.rotation.w = initial_qw_;
            map_to_odom_.transform.rotation.x = initial_qx_;
            map_to_odom_.transform.rotation.y = initial_qy_;
            map_to_odom_.transform.rotation.z = initial_qz_;
        }

        tf_publisher_thread_ = std::make_unique<std::thread>([this]()
                                                             {
            rclcpp::Rate rate(tf_rate_);
            while (rclcpp::ok() && !stop_thread_) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    map_to_odom_.header.stamp = this->now();
                    tf_broadcaster_->sendTransform(map_to_odom_);
                }
                
                std_msgs::msg::Bool status_msg;
                status_msg.data = is_ready_.load();
                status_pub_->publish(status_msg);

                rate.sleep();
            } });

        startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            [this]()
            { this->startupTimerCallback(); });

        RCLCPP_INFO(this->get_logger(), ">>> ICP Node Initialized. Initial Fake TF Published. Waiting... <<<");
    }

    IcpNode::~IcpNode()
    {
        stop_thread_ = true;
        if (tf_publisher_thread_ && tf_publisher_thread_->joinable())
        {
            tf_publisher_thread_->join();
        }
    }

    void IcpNode::startupTimerCallback()
    {
        startup_timer_->cancel(); // 只执行一次

        RCLCPP_INFO(this->get_logger(), "--- Triggering Auto-Localization ---");

        Eigen::Vector3d pos(initial_x_, initial_y_, initial_z_);
        Eigen::Quaterniond q(initial_qw_, initial_qx_, initial_qy_, initial_qz_);

        // 尝试定位
        bool ok = performLocalization(pos, q);
        if (!ok)
        {
            RCLCPP_WARN(this->get_logger(), "Auto-localization failed. Please use 2D Pose Estimate in RViz.");
        }
    }

    void IcpNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        pcl::fromROSMsg(*msg, *cloud_in_);
    }

    void IcpNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Manual Initial Pose.");
        Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

        performLocalization(pos, q);
    }

    // [核心函数] 执行定位逻辑
    bool IcpNode::performLocalization(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q_init)
    {
        // 1. 重置状态
        is_ready_.store(false);

        // 2. 检查数据
        auto current_cloud = std::make_shared<PointCloudXYZI>();
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (cloud_in_->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Cloud empty! Cannot localize.");
                return false;
            }
            pcl::copyPointCloud(*cloud_in_, *current_cloud);
        }

        // 开始计时
        auto start_time = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(this->get_logger(), "Starting ICP... Init Guess: x=%.2f, y=%.2f", pos.x(), pos.y());

        // 3. 构建初始矩阵
        Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
        initial_guess.block<3, 3>(0, 0) = q_init.toRotationMatrix();
        initial_guess.block<3, 1>(0, 3) = pos;

        // 4. 执行多重 ICP
        Eigen::Matrix4d map_to_laser = multiAlignSync(current_cloud, initial_guess);

        if (!success_)
        {
            RCLCPP_ERROR(this->get_logger(), "ICP Failed to converge.");
            return false;
        }

        // 5. 计算 map -> odom (需要查询 laser -> odom)
        Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
        try
        {
            // 等待 TF (最多1秒)
            auto transform = tf_buffer_->lookupTransform(
                laser_frame_id_, odom_frame_id_, tf2::TimePointZero, std::chrono::seconds(1));

            Eigen::Vector3d t(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            Eigen::Quaterniond q_lo(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);

            laser_to_odom.block<3, 3>(0, 0) = q_lo.toRotationMatrix();
            laser_to_odom.block<3, 1>(0, 3) = t;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TF Lookup Error (Laser->Odom): %s", ex.what());
            return false;
        }

        Eigen::Matrix4d result = map_to_laser * laser_to_odom;

        // 6. 更新 TF 数据 (加锁)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            map_to_odom_.transform.translation.x = result(0, 3);
            map_to_odom_.transform.translation.y = result(1, 3);
            map_to_odom_.transform.translation.z = result(2, 3);

            Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
            Eigen::Quaterniond q_res(rotation);

            map_to_odom_.transform.rotation.w = q_res.w();
            map_to_odom_.transform.rotation.x = q_res.x();
            map_to_odom_.transform.rotation.y = q_res.y();
            map_to_odom_.transform.rotation.z = q_res.z();
        }

        // 7. 设置成功标志位
        is_ready_.store(true);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "Localization Success! Time cost: %ld ms. Score: %.4f", duration.count(), score_);

        return true;
    }

    Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess)
    {
        // 定义一个 Lambda 表达式，用于将旋转矩阵转换为欧拉角 (Roll, Pitch, Yaw)
        static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d
        {
            double roll = std::atan2(rot(2, 1), rot(2, 2));
            double pitch = asin(-rot(2, 0));
            double yaw = std::atan2(rot(1, 0), rot(0, 0));
            return Eigen::Vector3d(roll, pitch, yaw);
        };

        // 重置成功标志
        success_ = false;
        // 从初始猜测矩阵中提取平移向量
        Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
        // 从初始猜测矩阵中提取旋转矩阵
        Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
        // 将旋转矩阵转换为 RPY 欧拉角
        Eigen::Vector3d rpy = rotate2rpy(rotation);

        // 预计算 Roll 和 Pitch 的旋转轴 (因为我们在搜索时只改变 Yaw)
        Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());

        // 存储所有候选位姿的容器
        std::vector<Eigen::Matrix4f> candidates;
        Eigen::Matrix4f temp_pose;

        // 计算 Yaw 轴搜索的步数。
        // 例如范围是 30 度，分辨率是 10 度，则 steps = 3
        int yaw_steps = 0;
        if (yaw_resolution_ > 1e-4)
        {
            yaw_steps = std::round(yaw_offset_ / yaw_resolution_);
        }

        // 打印搜索范围信息
        RCLCPP_INFO(this->get_logger(), "Searching with yaw_steps: %d", yaw_steps);

        // --- 生成候选点 ---
        // i, j 控制 XY 平面的位移：-1, 0, 1 倍的 xy_offset_
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                // k 控制 Yaw 角的旋转：-yaw_steps 到 +yaw_steps
                for (int k = -yaw_steps; k <= yaw_steps; k++)
                {
                    // 计算当前候选位置 (在初始位置基础上偏移)
                    Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_, xyz(2));
                    // 计算当前候选 Yaw 角
                    Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_, Eigen::Vector3f::UnitZ());

                    // 构建候选变换矩阵
                    temp_pose.setIdentity();
                    // 组合旋转：Roll * Pitch * (新的Yaw)
                    temp_pose.block<3, 3>(0, 0) = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
                    temp_pose.block<3, 1>(0, 3) = pos;
                    // 加入候选列表
                    candidates.push_back(temp_pose);
                }
            }
        }

        // 准备用于配准的输入源点云 (雷达数据)
        PointCloudXYZI::Ptr rough_source(new PointCloudXYZI);
        PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);

        // 对雷达点云进行滤波
        voxel_rough_filter_.setInputCloud(source);
        voxel_rough_filter_.filter(*rough_source); // 粗匹配用稀疏点云
        voxel_refine_filter_.setInputCloud(source);
        voxel_refine_filter_.filter(*refine_source); // 精匹配用较密点云

        // 计算雷达点云的法向量 (因为使用了 Point-to-Plane ICP)
        PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
        PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
        // 用于存储配准后点云的临时容器
        PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

        // 初始化最佳结果
        Eigen::Matrix4f best_rough_transform = init_guess.cast<float>();
        double best_rough_score = 1000.0; // 初始分数设大一点
        bool rough_converge = false;      // 是否收敛标志

        // --- 第一阶段：粗匹配 ---
        // 遍历所有候选位姿
        for (Eigen::Matrix4f &init_pose : candidates)
        {
            // 设置输入源
            icp_rough_.setInputSource(rough_source_norm);
            // 执行 ICP，传入初始猜测 init_pose
            icp_rough_.align(*align_point, init_pose);

            // 如果这次 ICP 没收敛，跳过
            if (!icp_rough_.hasConverged())
                continue;

            // 获取匹配得分 (Fitness Score)，越小越好
            double rough_score = icp_rough_.getFitnessScore();
            // 如果得分比当前最好的还要好
            if (rough_score < best_rough_score)
            {
                best_rough_score = rough_score;
                rough_converge = true;
                // 记录下这个变换矩阵
                best_rough_transform = icp_rough_.getFinalTransformation();
            }
        }

        // 如果所有候选都没收敛，或者最好的得分也很差 (超过 2 倍阈值)
        if (!rough_converge || best_rough_score > 2.0 * thresh_)
        {
            RCLCPP_WARN(this->get_logger(), "Rough match failed. Best Score: %f", best_rough_score);
            return Eigen::Matrix4d::Zero(); // 返回零矩阵表示失败
        }

        // --- 第二阶段：精匹配 ---
        // 使用精细的点云和粗匹配得到的最佳位置
        icp_refine_.setInputSource(refine_source_norm);
        icp_refine_.align(*align_point, best_rough_transform);
        score_ = icp_refine_.getFitnessScore();

        // 如果精匹配没收敛，或者得分超过阈值
        if (!icp_refine_.hasConverged() || score_ > thresh_)
        {
            RCLCPP_WARN(this->get_logger(), "Refine match failed. Score: %f > Thresh: %f", score_, thresh_);
            return Eigen::Matrix4d::Zero();
        }

        // 标记成功
        success_ = true;
        RCLCPP_INFO(this->get_logger(), "ICP Converged! Score: %f", score_);
        // 返回最终的变换矩阵 (double 精度)
        return icp_refine_.getFinalTransformation().cast<double>();
    }

    PointCloudXYZIN::Ptr IcpNode::addNorm(PointCloudXYZI::Ptr cloud)
    {
        // 创建存储法向量的指针
        auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
        // 创建 KD-Tree 用于加速近邻搜索
        auto searchTree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        searchTree->setInputCloud(cloud);

        // 创建法向量估计器
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
        normalEstimator.setInputCloud(cloud);
        normalEstimator.setSearchMethod(searchTree);
        normalEstimator.setKSearch(10); // 设置 K 近邻搜索数量为 10 (取周围 10 个点拟合平面计算法向量)
        normalEstimator.compute(*normals);

        // 将原始点云 (XYZI) 和法向量 (Normal) 拼接成新的点云格式 (XYZINormal)
        auto out = std::make_shared<PointCloudXYZIN>();
        pcl::concatenateFields(*cloud, *normals, *out);
        return out;
    }

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
