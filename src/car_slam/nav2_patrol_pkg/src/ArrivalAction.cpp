#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_patrol_pkg
{

  class WaitAtWaypoint : public nav2_core::WaypointTaskExecutor
  {
  public:
    // 【修改3】定义 Action 类型为 Wait
    using WaitAction = nav2_msgs::action::Wait;
    using ActionClient = rclcpp_action::Client<WaitAction>;

    // 构造函数，默认等待 3.0 秒
    WaitAtWaypoint() : wait_duration_(3.0), is_enabled_(true) {}

    void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name) override
    {
      auto node = parent.lock();
      if (!node)
      {
        throw std::runtime_error("Failed to lock node in WaitAtWaypoint plugin");
      }

      node_weak_ = parent;
      logger_ = node->get_logger();

      // 【修改4】声明等待时间参数
      node->declare_parameter(plugin_name + ".wait_duration", 3.0);
      node->get_parameter(plugin_name + ".wait_duration", wait_duration_);

      // 【修改5】创建 wait 动作客户端 (对应 behavior_server 中的 wait)
      action_client_ = rclcpp_action::create_client<WaitAction>(node, "wait");

      RCLCPP_INFO(logger_, "插件初始化完成，设定航点等待时间: %.2f 秒", wait_duration_);
    }

    bool processAtWaypoint(
        const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
        const int &curr_waypoint_index) override
    {
      if (!is_enabled_)
        return true;

      auto node = node_weak_.lock();
      if (!node)
      {
        RCLCPP_ERROR(logger_, "节点指针失效！");
        return false;
      }

      RCLCPP_INFO(logger_, "到达航点 %d，开始等待 %.1f 秒...", curr_waypoint_index, wait_duration_);

      // 检查 /wait 服务是否上线
      if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
      {
        RCLCPP_ERROR(logger_, "Wait 服务没上线 (请检查 behavior_server)，跳过！");
        return false;
      }

      // 【修改6】构建 Wait 目标
      auto goal_msg = WaitAction::Goal();
      // 转换 double 时间到 msg Duration
      goal_msg.time.sec = static_cast<int32_t>(wait_duration_);
      goal_msg.time.nanosec = static_cast<uint32_t>((wait_duration_ - goal_msg.time.sec) * 1e9);

      // 发送目标
      auto goal_handle_future = action_client_->async_send_goal(goal_msg);

      // 阻塞等待接收
      if (rclcpp::spin_until_future_complete(
              node->get_node_base_interface(), goal_handle_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(logger_, "发送 Wait 请求失败");
        return false;
      }

      auto goal_handle = goal_handle_future.get();
      if (!goal_handle)
      {
        RCLCPP_ERROR(logger_, "服务器拒绝了 Wait 请求");
        return false;
      }

      // 等待执行结果 (也就是等待那 3 秒过去)
      auto result_future = action_client_->async_get_result(goal_handle);

      auto status = rclcpp::spin_until_future_complete(
          node->get_node_base_interface(), result_future);

      if (status != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(logger_, "等待动作中途失败或被取消");
        return false;
      }

      RCLCPP_INFO(logger_, "等待结束，继续前往下一个航点！");
      return true;
    }

  protected:
    double wait_duration_; // 等待时长
    bool is_enabled_;
    rclcpp::Logger logger_{rclcpp::get_logger("WaitAtWaypoint")};
    ActionClient::SharedPtr action_client_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_weak_;
  };

} // namespace nav2_patrol_pkg

// 【修改7】导出新类名
PLUGINLIB_EXPORT_CLASS(nav2_patrol_pkg::WaitAtWaypoint, nav2_core::WaypointTaskExecutor)