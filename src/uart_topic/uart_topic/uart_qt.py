#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QTableWidget, QTableWidgetItem,
    QHeaderView, QDoubleSpinBox, QLineEdit
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

# 导入自定义消息（确保 uart_topic 包已编译并位于 PYTHONPATH）
from uart_msgs.msg import Recive, Send


class RosQtNode(Node):
    """ROS2 节点，负责订阅和发布"""
    def __init__(self):
        super().__init__('qt_ui_node')
        # 订阅接收数据
        self.subscription = self.create_subscription(
            Recive,
            '/recive_data',
            self.recive_callback,
            10
        )
        # 发布控制指令
        self.publisher = self.create_publisher(Send, '/send_cmd', 10)

        # 存储最新接收的数据（供界面读取）
        self.latest_data = None

    def recive_callback(self, msg):
        self.latest_data = msg

    def publish_command(self, speed, angular_velocity):
        """发布速度/角速度指令"""
        msg = Send()
        msg.target_speed = float(speed)
        msg.steer_angle = float(angular_velocity)
        self.publisher.publish(msg)


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ROS2 IMU 数据显示与控制")
        self.setGeometry(100, 100, 700, 500)

        self.init_ui()

        # 定时器：每 50ms 调用一次 ROS spin_once，并更新界面
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ---------- 数据显示 ----------
        data_group = QGroupBox("传感器数据 (来自 /recive_data)")
        data_layout = QVBoxLayout()
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(2)
        self.data_table.setHorizontalHeaderLabels(["字段", "数值"])
        self.data_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        data_layout.addWidget(self.data_table)
        data_group.setLayout(data_layout)
        main_layout.addWidget(data_group)

        # ---------- 控制指令发送 ----------
        send_group = QGroupBox("控制指令 (发布到 /send_cmd)")
        send_layout = QVBoxLayout()

        # 输入行
        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel("目标速度:"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(-0.15, 0.15)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setValue(0.0)
        input_layout.addWidget(self.speed_spin)

        input_layout.addWidget(QLabel("角速度(rad/s):"))
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-0.45, 0.45)
        self.angle_spin.setSingleStep(0.05)
        self.angle_spin.setDecimals(3)
        self.angle_spin.setValue(0.0)
        input_layout.addWidget(self.angle_spin)

        self.send_btn = QPushButton("发送指令")
        self.send_btn.clicked.connect(self.send_command)
        input_layout.addWidget(self.send_btn)

        send_layout.addLayout(input_layout)

        # 发送预览
        preview_layout = QHBoxLayout()
        preview_layout.addWidget(QLabel("发送字节 (预览):"))
        self.send_preview = QLineEdit()
        self.send_preview.setReadOnly(True)
        font = QFont("Courier New")
        self.send_preview.setFont(font)
        preview_layout.addWidget(self.send_preview)
        send_layout.addLayout(preview_layout)

        send_group.setLayout(send_layout)
        main_layout.addWidget(send_group)

        # 初始显示空数据
        self.update_table(None)

    def update_table(self, data):
        """更新表格显示，data 为 recive 消息或 None"""
        if data is None:
            rows = [("x (位置)", "---"),
                    ("y (位置)", "---"),
                    ("vx (速度)", "---"),
                    ("vy (速度)", "---"),
                    ("陀螺仪 Z (dps)", "---"),
                    ("偏航角 (deg)", "---"),
                    ("俯仰角 (deg)", "---"),
                    ("横滚角 (deg)", "---")]
        else:
            rows = [
                ("x (位置)", f"{data.x:.3f}"),
                ("y (位置)", f"{data.y:.3f}"),
                ("vx (速度)", f"{data.vx:.3f}"),
                ("vy (速度)", f"{data.vy:.3f}"),
                ("陀螺仪 Z (dps)", f"{data.gyro_z_dps:.4f}"),
                ("偏航角 (deg)", f"{data.yaw_deg:.3f}"),
                ("俯仰角 (deg)", f"{data.pitch_deg:.3f}"),
                ("横滚角 (deg)", f"{data.roll_deg:.3f}"),
            ]
        self.data_table.setRowCount(len(rows))
        for row, (field, value) in enumerate(rows):
            self.data_table.setItem(row, 0, QTableWidgetItem(field))
            self.data_table.setItem(row, 1, QTableWidgetItem(value))

    def update_ui(self):
        """定时调用 ROS spin_once 并刷新显示"""
        rclpy.spin_once(self.ros_node, timeout_sec=0)

        # 读取最新数据并更新表格
        if self.ros_node.latest_data is not None:
            self.update_table(self.ros_node.latest_data)

    def send_command(self):
        """发送控制指令"""
        speed = self.speed_spin.value()
        angular_velocity = self.angle_spin.value()
        # 发布到 ROS2
        self.ros_node.publish_command(speed, angular_velocity)

        # 预览（模仿之前串口发送的十六进制，仅为显示，实际编码在串口节点内进行）
        # 这里我们显示将要发送的数据，但不生成实际串口帧，只做提示
        preview = f"speed={speed:.2f}, angular={angular_velocity:.3f} rad/s"
        self.send_preview.setText(preview)

        # 也可显示为十六进制（如要模仿，可调用串口节点的打包函数，但为了简洁，此处略）


def main(args=None):
    rclpy.init(args=args)

    # 创建 ROS2 节点
    ros_node = RosQtNode()

    # 创建 Qt 应用
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # 运行 Qt 事件循环（ROS spin 由定时器驱动）
    try:
        sys.exit(app.exec_())
    finally:
        # 清理
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
