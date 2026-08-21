#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcia_vision_interfaces.msg import Trajectory
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import numpy as np
import math 

class BallisticSystem:
    def __init__(self):
        # 物理参数（需根据实际情况校准）
        self.v0 = 26.0  # 初始速度 (m/s)
        self.mass = 0.003  # 子弹质量 (kg)
        self.radius = 0.01  # 子弹半径 (m)
        self.Cd = 2.7  # 阻力系数（典型球形弹丸）
        self.rho = 1.225  # 空气密度 (kg/m³)
        self.g = 9.80665  # 重力加速度 (m/s²)

        # 坐标系参数（单位：米）
        self.camera_offset = np.array([0, 0.0, 0])  # [dx, dy, dz]
        self.R = np.eye(3)  # 假设摄像头与枪管无旋转

        # 弹道计算参数
        self.dt = 0.01  # 积分步长
        self.max_flight_time = 2.0

        # 预计算弹丸截面积
        self.A = math.pi * self.radius ** 2

    def _transform_coordinates(self, camera_point):
        """将摄像头坐标转换为枪口坐标系"""
        return camera_point - self.camera_offset

    def _dynamics(self, v):
        """计算空气阻力加速度"""
        drag_force = 0.5 * self.rho * self.Cd * self.A * v ** 2
        return drag_force / self.mass


    def visualize(self, trajectory, target_cam):
        plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置字体为 SimHei
        plt.rcParams['axes.unicode_minus'] = False

        """可视化弹道轨迹和参数"""
        target_gun = self._transform_coordinates(np.array(target_cam))

        plt.figure(figsize=(15, 6))

        # 弹道轨迹
        plt.subplot(1, 3, 1)
        plt.plot(trajectory[:, 1], trajectory[:, 2], label='实际弹道')

        # plt.scatter(target_gun[2], abs(target_gun[1]), c='red', label='目标位置')
        plt.scatter(target_gun[2], 0, c='red', label='目标位置')


        # 绘制理论抛物线（无空气阻力）
        t = np.linspace(0, np.max(trajectory[:, 0]), 5)
        x_theory = self.v0 * np.cos(np.radians(45)) * t
        y_theory = self.v0 * np.sin(np.radians(45)) * t - 0.5 * self.g * t ** 2
        # plt.plot(x_theory, y_theory, '--', label='理论抛物线（无阻力）')

        plt.xlabel('水平距离 (m)')
        plt.ylabel('高度 (m)')
        plt.title('弹道轨迹对比')
        plt.legend()
        plt.grid(True)
        
        # 速度衰减曲线
        plt.subplot(1, 3, 2)
        plt.plot(trajectory[:, 0], trajectory[:, 3])
        plt.xlabel('时间 (s)')
        plt.ylabel('速度 (m/s)')
        plt.title('子弹速度衰减曲线')
        plt.grid(True)

        # 弹道曲线
        plt.subplot(1, 3, 3)
        plt.plot(trajectory[:, 0], trajectory[:, 2])
        plt.xlabel('时间 (s)')
        plt.ylabel('速度 (m/s)')
        plt.title('子弹速度衰减曲线')
        plt.grid(True)

        plt.tight_layout()
        plt.show()



class TrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_subscriber')
        self.subscription = self.create_subscription(
            Trajectory,
            'trajectory_topic',  # 确保与C++代码中的话题名称一致
            self.trajectory_callback,
            10)
        self.subscription  # 防止未使用的变量警告
        self.ballisticSystem = BallisticSystem()
        self.init = 1

    def trajectory_callback(self, msg):
        self.get_logger().info('Received trajectory:')
        trajectory = []

        for i in range(len(msg.time)):
            self.get_logger().info(f"Time: {msg.time[i]}, X: {msg.x[i]}, Y: {msg.y[i]}, Z: {msg.z[i]}, Speed: {msg.speed[i]}")
            self.get_logger().info(f"target_x: {msg.target_gun_x}, target_y: {msg.target_gun_y}, target_z: {msg.target_gun_z}")
            trajectory.append(np.array([msg.time[i], msg.x[i], msg.y[i], msg.speed[i]]))
        
        trajectory = np.array(trajectory)
        
        target_position = np.array([msg.target_gun_x, msg.target_gun_y, msg.target_gun_z])

        # if self.init == 1:
        self.ballisticSystem.visualize(trajectory, target_position)
        self.init = 0

def main(args=None):
    rclpy.init(args=args)
    trajectory_subscriber = TrajectorySubscriber()
    rclpy.spin(trajectory_subscriber)
    trajectory_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
