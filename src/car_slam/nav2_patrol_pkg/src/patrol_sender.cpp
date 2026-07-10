#include <memory>
#include <vector>
#include <string>
#include <cmath> // 用于 sin, cos

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp" // 引入 Bool 消息类型

class PatrolSender : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  PatrolSender() : Node("patrol_sender"), goal_sent_(false) // 初始化标志位为 false
  {
    // --- 1. 读取 YAML 参数的核心逻辑 ---
    this->declare_parameter("num_waypoints", 0);
    int num_waypoints = this->get_parameter("num_waypoints").as_int();

    RCLCPP_INFO(this->get_logger(), "读取到配置：共有 %d 个航点", num_waypoints);

    for (int i = 0; i < num_waypoints; ++i)
    {
      std::string param_name = "waypoint_" + std::to_string(i);
      this->declare_parameter(param_name, std::vector<double>{});
      std::vector<double> wp_data = this->get_parameter(param_name).as_double_array();

      if (wp_data.size() != 3)
      {
        RCLCPP_WARN(this->get_logger(), "参数 %s 格式错误！必须是 [x, y, yaw]。跳过。", param_name.c_str());
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = wp_data[0];
      pose.pose.position.y = wp_data[1];
      pose.pose.position.z = 0.0;

      double yaw = wp_data[2];
      pose.pose.orientation.z = sin(yaw / 2.0);
      pose.pose.orientation.w = cos(yaw / 2.0);

      waypoints_.push_back(pose);
      RCLCPP_INFO(this->get_logger(), "加载航点 %d: x=%.2f, y=%.2f, yaw=%.2f", i, wp_data[0], wp_data[1], yaw);
    }
    // -------------------------------------

    // 创建 Action Client
    client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(
        this, "follow_waypoints");

    // 【优化】创建订阅者，监听 /localization_ready
    localization_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/localization_ready", 1,
        std::bind(&PatrolSender::localization_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "节点已启动，正在等待 /localization_ready 为 true...");
  }

  // 【优化】回调函数
  void localization_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // 如果任务已经发送过，直接返回，避免重复发送
    if (goal_sent_)
    {
      return;
    }

    // 检查定位是否就绪
    if (msg->data)
    {
      RCLCPP_INFO(this->get_logger(), "接收到定位就绪信号 (/localization_ready = true)，准备发送航点...");

      // 标记为已发送，防止下一次回调重复触发
      goal_sent_ = true;

      // 执行发送逻辑
      send_goal();
    }
    else
    {
      // 可以在这里打印日志，但建议使用 throttle 防止刷屏
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "等待定位就绪中...");
    }
  }

  void send_goal()
  {
    if (waypoints_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "没有读取到任何航点，无法执行巡逻！");
      rclcpp::shutdown(); // 没航点直接退出
      return;
    }

    // 等待 Action Server 上线
    RCLCPP_INFO(this->get_logger(), "正在连接导航服务器...");
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server 'follow_waypoints' 连接超时，请检查 Nav2 是否启动");
      // 如果连接失败，重置标志位，允许下次收到信号再次尝试
      goal_sent_ = false;
      return;
    }

    auto goal_msg = FollowWaypoints::Goal();

    // 填充航点
    for (auto &pose : waypoints_)
    {
      pose.header.stamp = this->now();
      goal_msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "正在发送 %zu 个航点任务...", goal_msg.poses.size());

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&PatrolSender::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&PatrolSender::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "服务器拒绝了航点任务！");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "服务器已接收任务，开始巡逻...");
    }
  }

  void result_callback(const GoalHandleFollowWaypoints::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "巡逻任务圆满完成！");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "巡逻任务被中止");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "巡逻任务被取消");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "未知结果代码");
      break;
    }
    rclcpp::shutdown();
  }

  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;

  // 【优化】新增订阅者
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

  // 【优化】状态标志位，防止重复发送
  bool goal_sent_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}