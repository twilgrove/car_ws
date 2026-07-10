#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kDefaultAngularScale = 1.0; // RT-Thread wm_set expects rad/s.

enum VehicleStatusCode : std::uint16_t
{
    kVehicleStatusOk = 0,
    kVehicleStatusRpmsgNotReady = 1,
    kVehicleStatusRttStopped = 2,
    kVehicleStatusStatusParseError = 3,
    kVehicleStatusCanError = 4,
    kVehicleStatusCommError = 5,
};

double clampValue(double value, double limit)
{
    if (limit <= 0.0)
        return value;
    return std::max(-limit, std::min(limit, value));
}

std::string trimReply(std::string text)
{
    const auto nul_pos = text.find('\0');
    if (nul_pos != std::string::npos)
        text.resize(nul_pos);

    const auto colon = text.find(": ");
    if (colon != std::string::npos && text.rfind("from=", 0) == 0)
        text.erase(0, colon + 2);

    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ' || text.back() == '\t'))
        text.pop_back();

    while (!text.empty() && (text.front() == '\n' || text.front() == '\r' || text.front() == ' ' || text.front() == '\t'))
        text.erase(text.begin());

    return text;
}

bool parseBatteryMv(const std::string &text, int &battery_mv)
{
    const auto pos = text.find("BAT=");
    unsigned int volts = 0;
    unsigned int millivolts = 0;

    if (pos == std::string::npos)
        return false;

    if (std::sscanf(text.c_str() + pos + 4, "%u.%u", &volts, &millivolts) != 2)
        return false;

    battery_mv = static_cast<int>(volts * 1000U + millivolts);
    return true;
}

bool parseLongKey(const std::string &text, const std::string &key, long &value)
{
    const auto pos = text.find(key);
    char *end = nullptr;

    if (pos == std::string::npos)
        return false;

    value = std::strtol(text.c_str() + pos + key.size(), &end, 0);
    return end && end != text.c_str() + pos + key.size();
}
} // namespace

class ChassisNode : public rclcpp::Node
{
public:
    ChassisNode() : Node("chassis_node")
    {
        loadParameters();

        status_code_pub_ = this->create_publisher<std_msgs::msg::UInt16>("vehicle_status_code", 10);
        status_text_pub_ = this->create_publisher<std_msgs::msg::String>("vehicle_status_text", 10);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ChassisNode::cmdVelCallback, this, std::placeholders::_1));

        last_cmd_time_ = this->now();
        configureRuntime();

        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / control_rate_hz_)),
            std::bind(&ChassisNode::controlLoop, this));

        status_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / status_rate_hz_)),
            std::bind(&ChassisNode::statusLoop, this));

        RCLCPP_INFO(this->get_logger(),
                    "RK3588 RPMsg chassis node started: path=%s control=%.1fHz status=%.1fHz scale=(%.3f, %.3f)",
                    rpmsg_path_.c_str(), control_rate_hz_, status_rate_hz_, linear_scale_, angular_scale_);
    }

    ~ChassisNode() override
    {
        std::string reply;
        transactExpect("SET 0 0 0", reply, "OK SET");
        if (auto_start_)
            transactExpect("STOP", reply, "OK STOP");
    }

private:
    void loadParameters()
    {
        rpmsg_path_ = this->declare_parameter<std::string>("rpmsg_path", "/proc/rpmsg_rtt");
        control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 100.0);
        status_rate_hz_ = this->declare_parameter<double>("status_rate_hz", 5.0);
        command_timeout_sec_ = this->declare_parameter<double>("command_timeout_sec", 0.5);
        reply_wait_ms_ = this->declare_parameter<int>("reply_wait_ms", 1);
        auto_start_ = this->declare_parameter<bool>("auto_start", true);
        linear_scale_ = this->declare_parameter<double>("linear_scale", 30.0);
        angular_scale_ = this->declare_parameter<double>("angular_scale", kDefaultAngularScale);
        max_linear_mps_ = this->declare_parameter<double>("max_linear_mps", 1.1);
        max_angular_rps_ = this->declare_parameter<double>("max_angular_rps", 3.1);
        startup_mode_ = this->declare_parameter<std::string>("startup_mode", "PID");
        pwm_min_ = this->declare_parameter<double>("pwm_min", 220.0);
        pwm_gain_ = this->declare_parameter<double>("pwm_gain", 120.0);
        pwm_max_ = this->declare_parameter<double>("pwm_max", 4000.0);
        control_rate_hz_ = std::max(1.0, control_rate_hz_);
        status_rate_hz_ = std::max(1.0, status_rate_hz_);
        reply_wait_ms_ = std::max(1, reply_wait_ms_);
    }

    void configureRuntime()
    {
        std::string reply;
        bool ping_ok = false;

        for (int attempt = 0; attempt < 3; attempt++)
        {
            drainReplies();
            if (transactExpect("PING", reply, "rpmsg-echo"))
            {
                ping_ok = true;
                break;
            }
            rclcpp::sleep_for(20ms);
        }

        if (!ping_ok)
        {
            RCLCPP_WARN(this->get_logger(), "RT-Thread RPMsg is not ready yet: %s", rpmsg_path_.c_str());
            publishVehicleStatus(kVehicleStatusRpmsgNotReady, "rpmsg ping failed");
            return;
        }

        rclcpp::sleep_for(5ms);
        drainReplies();

        ready_.store(true);
        sendConfigCommand("MODE " + startup_mode_);
        sendConfigCommand("PWMMIN " + formatNumber(pwm_min_));
        sendConfigCommand("PWMGAIN " + formatNumber(pwm_gain_));
        sendConfigCommand("PWMMAX " + formatNumber(pwm_max_));

        if (auto_start_)
            sendConfigCommand("START");
    }

    void sendConfigCommand(const std::string &cmd)
    {
        std::string reply;
        const std::string token = cmd.substr(0, cmd.find(' '));
        if (!transactExpect(cmd, reply, "OK " + token))
        {
            RCLCPP_WARN(this->get_logger(), "RPMsg config command failed: %s", cmd.c_str());
            ready_.store(false);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RTT <= %s | => %s", cmd.c_str(), reply.c_str());
    }

    static std::string formatNumber(double value)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(3);
        oss << value;
        return oss.str();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double vx_mps = clampValue(msg->linear.x, max_linear_mps_);
        const double vy_mps = clampValue(msg->linear.y, max_linear_mps_);
        const double wz_rps = clampValue(msg->angular.z, max_angular_rps_);

        vx_rtt_.store(vx_mps * linear_scale_);
        vy_rtt_.store(-vy_mps * linear_scale_);
        omega_rtt_.store(wz_rps * angular_scale_);
        last_cmd_time_ = this->now();
    }

    void controlLoop()
    {
        if (!ready_.load())
        {
            configureRuntime();
            return;
        }

        const double cmd_age_sec = (this->now() - last_cmd_time_).seconds();
        const bool timed_out = cmd_age_sec > command_timeout_sec_;
        const double vx = timed_out ? 0.0 : vx_rtt_.load();
        const double vy = timed_out ? 0.0 : vy_rtt_.load();
        const double omega = timed_out ? 0.0 : omega_rtt_.load();

        std::ostringstream cmd;
        cmd.setf(std::ios::fixed);
        cmd.precision(3);
        cmd << "SET " << vx << " " << vy << " " << omega;

        std::string reply;
        if (!transactExpect(cmd.str(), reply, "OK SET"))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "RT-Thread control command reply timeout, will retry bootstrap");
            ready_.store(false);
            publishVehicleStatus(kVehicleStatusCommError, "control command reply timeout");
            return;
        }

        log_counter_++;
        if (log_counter_ >= static_cast<int>(control_rate_hz_))
        {
            log_counter_ = 0;
            RCLCPP_INFO(this->get_logger(), "cmd_vel -> SET %.3f %.3f %.3f%s",
                        vx, vy, omega, timed_out ? " (timeout safe stop)" : "");
        }
    }

    void statusLoop()
    {
        if (!ready_.load())
        {
            publishVehicleStatus(kVehicleStatusRpmsgNotReady, "rpmsg not ready");
            return;
        }

        std::string reply;
        if (!transactExpect("STATUS", reply, "BOOT="))
        {
            publishVehicleStatus(kVehicleStatusCommError, "STATUS reply timeout");
            return;
        }

        publishVehicleStatusFromReply(reply);
    }

    void publishVehicleStatus(std::uint16_t code, const std::string &text)
    {
        std_msgs::msg::UInt16 code_msg;
        std_msgs::msg::String text_msg;

        code_msg.data = code;
        text_msg.data = text;

        status_code_pub_->publish(code_msg);
        status_text_pub_->publish(text_msg);

        if (code != last_status_code_)
        {
            last_status_code_ = code;
            if (code == kVehicleStatusOk)
                RCLCPP_INFO(this->get_logger(), "vehicle status OK: %s", text.c_str());
            else
                RCLCPP_WARN(this->get_logger(), "vehicle status code=%u: %s", code, text.c_str());
        }
    }

    void publishVehicleStatusFromReply(const std::string &status)
    {
        long boot = 0;
        long init = 0;
        long run = 0;
        long err = 0;
        long txerr = 0;
        long rxerr = 0;
        std::ostringstream text;

        if (!parseLongKey(status, "BOOT=", boot) ||
            !parseLongKey(status, "INIT=", init) ||
            !parseLongKey(status, "RUN=", run))
        {
            publishVehicleStatus(kVehicleStatusStatusParseError, "STATUS missing BOOT/INIT/RUN");
            return;
        }

        parseLongKey(status, "ERR=", err);
        parseLongKey(status, "TXERR=", txerr);
        parseLongKey(status, "RXERR=", rxerr);
        parseBatteryMv(status, last_battery_mv_);

        text << "boot=" << boot
             << " init=" << init
             << " run=" << run
             << " can_err=" << err
             << " txerr=" << txerr
             << " rxerr=" << rxerr
             << " bat_mv=" << last_battery_mv_;

        if (init == 0 || run == 0)
        {
            publishVehicleStatus(kVehicleStatusRttStopped, text.str());
            return;
        }

        if (err != 0 || txerr != 0 || rxerr != 0)
        {
            publishVehicleStatus(kVehicleStatusCanError, text.str());
            return;
        }

        publishVehicleStatus(kVehicleStatusOk, text.str());
    }

    bool transact(const std::string &cmd, std::string &reply, bool expect_reply)
    {
        std::lock_guard<std::mutex> lock(rpmsg_mutex_);

        if (!writeCommand(cmd))
            return false;

        if (!expect_reply)
            return true;

        rclcpp::sleep_for(std::chrono::milliseconds(reply_wait_ms_));
        return readReply(reply);
    }

    bool transactExpect(const std::string &cmd, std::string &reply, const std::string &needle)
    {
        std::lock_guard<std::mutex> lock(rpmsg_mutex_);

        if (!writeCommand(cmd))
            return false;

        rclcpp::sleep_for(std::chrono::milliseconds(reply_wait_ms_));
        for (int i = 0; i < 4; i++)
        {
            std::string candidate;
            if (!readReply(candidate))
                return false;
            if (candidate.find(needle) != std::string::npos)
            {
                reply = candidate;
                return true;
            }
        }

        return false;
    }

    bool writeCommand(const std::string &cmd)
    {
        const int fd = ::open(rpmsg_path_.c_str(), O_WRONLY | O_CLOEXEC);
        if (fd < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "open(%s) for write failed: %s", rpmsg_path_.c_str(), std::strerror(errno));
            return false;
        }

        const std::string line = cmd + "\n";
        const char *ptr = line.c_str();
        ssize_t left = static_cast<ssize_t>(line.size());
        while (left > 0)
        {
            const ssize_t n = ::write(fd, ptr, static_cast<size_t>(left));
            if (n <= 0)
            {
                RCLCPP_WARN(this->get_logger(), "write(%s) failed: %s", rpmsg_path_.c_str(), std::strerror(errno));
                ::close(fd);
                return false;
            }
            ptr += n;
            left -= n;
        }

        ::close(fd);
        return true;
    }

    bool readReply(std::string &reply)
    {
        const int fd = ::open(rpmsg_path_.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK);
        std::array<char, 1024> buf{};
        ssize_t n = -1;

        if (fd < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "open(%s) for read failed: %s", rpmsg_path_.c_str(), std::strerror(errno));
            return false;
        }

        for (int i = 0; i < 20; i++)
        {
            n = ::read(fd, buf.data(), buf.size() - 1U);
            if (n > 0)
                break;
            if (n == 0)
                break;
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                break;
            rclcpp::sleep_for(2ms);
        }

        ::close(fd);

        if (n <= 0)
            return false;

        buf[static_cast<size_t>(n)] = '\0';
        reply = trimReply(std::string(buf.data(), static_cast<size_t>(n)));
        return !reply.empty();
    }

    void drainReplies()
    {
        std::lock_guard<std::mutex> lock(rpmsg_mutex_);
        const int fd = ::open(rpmsg_path_.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK);
        std::array<char, 1024> buf{};

        if (fd < 0)
            return;

        for (int i = 0; i < 32; i++)
        {
            const ssize_t n = ::read(fd, buf.data(), buf.size());
            if (n <= 0)
                break;
        }

        ::close(fd);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_code_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_text_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::mutex rpmsg_mutex_;
    std::string rpmsg_path_;
    std::string startup_mode_;

    std::atomic<bool> ready_{false};
    std::atomic<double> vx_rtt_{0.0};
    std::atomic<double> vy_rtt_{0.0};
    std::atomic<double> omega_rtt_{0.0};
    rclcpp::Time last_cmd_time_;

    double control_rate_hz_ = 100.0;
    double status_rate_hz_ = 5.0;
    double command_timeout_sec_ = 0.5;
    double linear_scale_ = 10.0;
    double angular_scale_ = kDefaultAngularScale;
    double max_linear_mps_ = 1.1;
    double max_angular_rps_ = 3.1;
    double pwm_min_ = 220.0;
    double pwm_gain_ = 120.0;
    double pwm_max_ = 4000.0;
    int reply_wait_ms_ = 1;
    int log_counter_ = 0;
    int last_battery_mv_ = 0;
    std::uint16_t last_status_code_ = std::numeric_limits<std::uint16_t>::max();
    bool auto_start_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
