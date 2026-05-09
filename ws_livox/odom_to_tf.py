#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomFromImu(Node):
    def __init__(self):
        super().__init__("odom_from_imu")

        # 【输入：订阅 Livox IMU】
        self.imu_sub = self.create_subscription(
            Imu, "/livox/imu", self.imu_callback, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # 雷达外参
        self.LIDAR_X = 0.1
        self.LIDAR_Y = 0.0
        self.LIDAR_Z = 0.2

        # 积分用变量
        self.yaw = 0.0
        self.last_time = None

        self.get_logger().info("✅ IMU 输入 → odom → base_link 已启动")

    def imu_callback(self, msg):
        # 【计算 1：获取时间差】
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now

        # 【计算 2：从 IMU 获取角速度（绕Z轴转 = yaw）】
        wz = msg.angular_velocity.z

        # 【计算 3：积分 → 得到当前角度】
        self.yaw += wz * dt

        # 角度归一化
        while self.yaw > math.pi: self.yaw -= 2*math.pi
        while self.yaw < -math.pi: self.yaw += 2*math.pi

        # ==============================================
        # 【输出：发布 odom → base_link】
        # ==============================================
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # 位置暂时给 0（纯IMU只能算角度，位置需要雷达）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 【关键：IMU 算出的角度】
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFromImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
