#include <chrono>
#include <cstring>
#include <array>
#include <memory>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <serial/serial.h>

using namespace std::chrono_literals;

class ChassisNode : public rclcpp::Node
{
public:
    ChassisNode() : Node("chassis_node")
    {
        // 1. 订阅 cmd_vel
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1,
            std::bind(&ChassisNode::cmdVelCallback, this, std::placeholders::_1));

        // 2. 初始化串口参数
        port_name_ = this->declare_parameter<std::string>("port", "/dev/wheeltec_car"); // 建议改为你的软连接名字
        int baudrate = this->declare_parameter<int>("baudrate", 115200);

        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        serial_.setPort(port_name_);
        serial_.setBaudrate(baudrate);
        serial_.setTimeout(to);

        // 尝试首次连接
        attemptReconnect();

        // 3. 定时器主循环
        timer_ = this->create_wall_timer(
            50ms, std::bind(&ChassisNode::updateLoop, this));

        RCLCPP_INFO(this->get_logger(), "底盘节点启动完成");
    }

    ~ChassisNode()
    {
        if (serial_.isOpen())
        {
            v_x_cmd_.store(0.0f);
            v_y_cmd_.store(0.0f);
            v_z_cmd_.store(0.0f);
            flag_start_ = 0;
            try {
                sendData(); // 尝试最后发送一次停止
            } catch(...) {}
            serial_.close();
        }
    }

private:
    // 增加一个重连函数
    void attemptReconnect()
    {
        try
        {
            // 确保先关闭，清除无效的句柄
            if (serial_.isOpen())
            {
                serial_.close();
            }
            
            // 重新打开
            serial_.setPort(port_name_); // 确保指向正确的软连接路径
            serial_.open();

            if (serial_.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "串口重连成功: %s", port_name_.c_str());
                // 重连成功后，发送几帧激活指令
                for (int j = 0; j < 3; ++j)
                {
                    sendData(); 
                    rclcpp::sleep_for(10ms); // 稍微延时，不要阻塞太久
                }
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "串口连接失败，正在重试... 错误: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "串口连接发生未知错误");
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx = msg->linear.x;
        float vy = -msg->linear.y;
        float vz = msg->angular.z*57.29578f;

        // 简单限幅
        if (vx > 1.1f) vx = 1.1f; else if (vx < -1.1f) vx = -1.1f;
        if (vy > 1.1f) vy = 1.1f; else if (vy < -1.1f) vy = -1.1f;
        if (vz > 3.1f*57.29578f) vz = 3.1f*57.29578f; else if (vz < -3.1f*57.29578f) vz = -3.1f*57.29578f;

        v_x_cmd_.store(vx);
        v_y_cmd_.store(vy);
        v_z_cmd_.store(vz);

        flag_mode_ = 1;
        flag_start_ = (vx == 0.0f && vy == 0.0f && vz == 0.0f) ? 0 : 1;
    }

    void updateLoop()
    {
        // 关键修改：每次循环先检查串口状态
        if (!serial_.isOpen())
        {
            attemptReconnect();
            return; // 如果还没连上，这次循环就先不发数据了，防止报错刷屏
        }

        // 20Hz 频率发送
        sendData();

        t_log_++;
        if (t_log_ >= 4) // 约 0.2s 打印一次日志 (20Hz / 50ms interval? Wait, 50ms is 20Hz. So every 4 calls is 0.2s? No. 50ms=20Hz. log every 4 times = 200ms = 5Hz log)
        {
            t_log_ = 0;
            // 只有连接正常才打印这个，避免干扰错误日志
            if (serial_.isOpen()) {
                 RCLCPP_INFO(this->get_logger(), "指令 -> Vx: %.2f, Vy: %.2f, Vth: %.2f | 状态: %s",
                        v_x_cmd_.load(), v_y_cmd_.load(), v_z_cmd_.load(),
                        (flag_start_ ? "运行" : "停止"));
            }
        }
    }

    void sendData()
    {
        // 如果串口没开，直接返回，交给 updateLoop 去重连
        if (!serial_.isOpen()) return;

        const uint8_t len = 12;
        uint8_t tbuf[53] = {0}; 

        Data_US_[0] = static_cast<float>(flag_start_);
        Data_US_[1] = static_cast<float>(flag_mode_);
        Data_US_[2] = v_x_cmd_.load();
        Data_US_[3] = v_y_cmd_.load();
        Data_US_[4] = v_z_cmd_.load();

        for (uint8_t i = 0; i < len; ++i)
        {
            unsigned char *p = reinterpret_cast<unsigned char *>(&Data_US_[i]);
            tbuf[4 * i + 4] = *(p + 3);
            tbuf[4 * i + 5] = *(p + 2);
            tbuf[4 * i + 6] = *(p + 1);
            tbuf[4 * i + 7] = *(p + 0);
        }

        tbuf[0] = 0xAA;
        tbuf[1] = 0xAA;
        tbuf[2] = 0xF1;
        tbuf[3] = len * 4;

        tbuf[len * 4 + 4] = 0;
        for (uint8_t i = 0; i < (len * 4 + 4); ++i)
        {
            tbuf[len * 4 + 4] += tbuf[i];
        }

        try
        {
            serial_.write(tbuf, len * 4 + 5);
        }
        catch (serial::IOException &e)
        {
            // 关键修改：捕获异常后，立即关闭串口！
            // 这样下一次 updateLoop 就会检测到 !isOpen() 从而触发重连
            RCLCPP_ERROR(this->get_logger(), "串口发送IO异常，连接断开！正在关闭以准备重连...");
            serial_.close(); 
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "串口发送未知异常，正在关闭...");
            serial_.close();
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
    std::string port_name_; // 保存端口名用于重连

    std::array<float, 12> Data_US_{};
    std::atomic<float> v_x_cmd_{0.0f};
    std::atomic<float> v_y_cmd_{0.0f};
    std::atomic<float> v_z_cmd_{0.0f};

    uint8_t flag_start_ = 0;
    uint8_t flag_mode_ = 1;
    int t_log_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
