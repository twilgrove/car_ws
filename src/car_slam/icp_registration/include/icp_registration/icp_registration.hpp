#ifndef ICP_REGISTRATION_HPP
#define ICP_REGISTRATION_HPP

#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <atomic>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp> // 状态消息
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// pcl
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

namespace icp
{

  using PointType = pcl::PointXYZINormal;
  using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
  using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

  class IcpNode : public rclcpp::Node
  {
  public:
    IcpNode(const rclcpp::NodeOptions &options);
    ~IcpNode();

  private:
    // --- 回调函数 ---
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // 自动启动的回调
    void startupTimerCallback();

    // 封装的核心定位逻辑 (带计时和返回值)
    bool performLocalization(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q_init);

    // --- 算法辅助函数 ---
    static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);
    Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess);

    // --- ROS 通信 ---
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    // [新增] 状态发布者
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

    // --- 定时器 ---
    rclcpp::TimerBase::SharedPtr startup_timer_; // 用于自动启动

    // --- 数据存储 ---
    sensor_msgs::msg::PointCloud2 map_ros_msg_;
    PointCloudXYZI::Ptr cloud_in_;
    PointCloudXYZIN::Ptr refine_map_;
    PointCloudXYZIN::Ptr rough_map_;

    // TF 数据 (受互斥锁保护)
    geometry_msgs::msg::TransformStamped map_to_odom_;
    int tf_rate_; // TF 发布频率

    // --- TF 工具 ---
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- 线程控制 ---
    std::atomic<bool> stop_thread_;
    std::unique_ptr<std::thread> tf_publisher_thread_;
    std::mutex mutex_;       // 保护 map_to_odom_
    std::mutex cloud_mutex_; // 保护 cloud_in_

    // --- 算法对象 ---
    pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;
    pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
    pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;

    // --- 参数 ---
    std::filesystem::path pcd_path_;
    // 初始位姿参数
    double initial_x_, initial_y_, initial_z_;
    double initial_qx_, initial_qy_, initial_qz_, initial_qw_;

    std::string pointcloud_topic_;
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string laser_frame_id_;

    int rough_iter_;
    int refine_iter_;
    bool success_;
    double score_;
    double thresh_;
    double xy_offset_;
    double yaw_offset_;
    double yaw_resolution_;

    std::atomic<bool> is_ready_;
  };

} // namespace icp

#endif // ICP_REGISTRATION_HPP