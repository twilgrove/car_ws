#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用 odom 的 xy 位移差分算实际速度, 并和 cmd_vel 指令速度对比, 用于标定底盘 linear_scale。

用法 (容器内, source 好 ROS 后):
    python3 /car_ws/speed_check.py
    # 指定别的 odom 话题:
    python3 /car_ws/speed_check.py --ros-args -p odom_topic:=/odom

标定:
    发一个恒定 cmd (比如 0.3 m/s), 等实测稳定, 看 "实测/cmd" 比值 r。
    新 linear_scale = 旧 linear_scale / r
    (例: 命令0.3 实测0.15 -> r=0.5 -> linear_scale 30 应改成 60)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SpeedCheck(Node):
    def __init__(self):
        super().__init__("speed_check")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("print_rate", 2.0)   # 打印频率 Hz
        self.declare_parameter("window", 1.0)        # 位移差分平均窗口 秒

        odom_topic = self.get_parameter("odom_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        self.window = self.get_parameter("window").value

        # best_effort 订阅: reliable / best_effort 发布都能收到, 最兼容
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, odom_topic, self.odom_cb, qos)
        self.create_subscription(Twist, cmd_topic, self.cmd_cb, qos)

        self.prev_x = self.prev_y = self.prev_t = None
        self.win = []               # [(t, dt, dist), ...]
        self.meas_speed = 0.0       # 位移差分实测线速度
        self.twist_speed = 0.0      # odom.twist 里的线速度 (交叉验证)
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0

        self.create_timer(1.0 / self.get_parameter("print_rate").value, self.report)
        self.get_logger().info(f"listening: odom={odom_topic}  cmd={cmd_topic}")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.twist_speed = math.hypot(vx, vy)

        if self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-4:
                dist = math.hypot(x - self.prev_x, y - self.prev_y)
                self.win.append((t, dt, dist))
                self.win = [w for w in self.win if t - w[0] <= self.window]
                sdt = sum(w[1] for w in self.win)
                sd = sum(w[2] for w in self.win)
                self.meas_speed = sd / sdt if sdt > 0 else 0.0
        self.prev_x, self.prev_y, self.prev_t = x, y, t

    def cmd_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

    def report(self):
        cmd_speed = math.hypot(self.cmd_vx, self.cmd_vy)
        ratio = self.meas_speed / cmd_speed if cmd_speed > 1e-3 else float("nan")
        self.get_logger().info(
            f"cmd={cmd_speed:5.3f}m/s(vx={self.cmd_vx:+.2f} vy={self.cmd_vy:+.2f} wz={self.cmd_wz:+.2f}) | "
            f"实测(位移差)={self.meas_speed:5.3f}m/s | odom.twist={self.twist_speed:5.3f}m/s | "
            f"实测/cmd={ratio:4.2f}"
        )


def main():
    rclpy.init()
    node = SpeedCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
