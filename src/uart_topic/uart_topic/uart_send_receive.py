#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from uart_msgs.msg import Recive, Send   # 自定义消息
import serial
import struct
from typing import Optional, List, Dict, Any


# ==================== 串口通信类（仅 IMU 协议） ====================
class SerialComm:
    def __init__(self, port: str = '/dev/ttyS1', baudrate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._buffer = b''

    def open(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            return True
        except Exception as e:
            print(f"打开串口失败: {e}")
            return False

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
        self._buffer = b''

    def is_open(self) -> bool:
        return self.serial is not None and self.serial.is_open

    def write(self, data: bytes) -> int:
        if not self.is_open():
            raise RuntimeError("串口未打开")
        return self.serial.write(data)

    def read_all(self) -> bytes:
        if not self.is_open():
            raise RuntimeError("串口未打开")
        return self.serial.read(self.serial.in_waiting)

    def read_into_buffer(self) -> int:
        if not self.is_open():
            return 0
        data = self.read_all()
        if data:
            self._buffer += data
        return len(data)

    # ---------- UART 协议解析（固定 35 字节） ----------
    @staticmethod
    def parse_frame(frame: bytes) -> Dict[str, float]:
        if len(frame) < 35:
            raise ValueError("帧长度不足（需35字节）")
        if frame[0] != 0xAA or frame[1] != 0x55:
            raise ValueError("无效帧头")
        try:
            x, y, vx, vy, gyro_z, yaw, pitch, roll = \
                struct.unpack('<ffffffff', frame[2:34])
            # checksum = frame[34]   # 不校验也可
        except Exception as e:
            raise ValueError(f"解包失败: {e}")
        return {
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
            "gyro_z_dps": gyro_z,
            "yaw_deg": yaw,
            "pitch_deg": pitch,
            "roll_deg": roll,
        }

    def extract_frames(self) -> List[Dict[str, float]]:
        results = []
        while True:
            idx = self._buffer.find(b'\xAA\x55')
            if idx == -1:
                break
            frame_len = 35
            if len(self._buffer) < idx + frame_len:
                break
            frame = self._buffer[idx:idx + frame_len]
            try:
                parsed = self.parse_frame(frame)
                results.append(parsed)
            except ValueError:
                self._buffer = self._buffer[idx + 2:]
                continue
            self._buffer = self._buffer[idx + frame_len:]
        return results

    # ---------- 发送控制指令（11 字节） ----------
    def build_command(self, target_speed: float, steer_angle: float) -> bytes:
        header = b'\xAA\x55'
        data = struct.pack('<ff', target_speed, steer_angle)
        frame_without_checksum = header + data
        checksum = sum(frame_without_checksum) & 0xFF
        return frame_without_checksum + bytes([checksum])

    def send_command(self, target_speed: float, steer_angle: float) -> bool:
        if not self.is_open():
            return False
        frame = self.build_command(target_speed, steer_angle)
        try:
            self.write(frame)
            return True
        except Exception:
            return False


# ==================== ROS2 节点 ====================
class SerialRosNode(Node):
    def __init__(self):
        super().__init__('uart_topic_node')   # 节点名可自定义

        # 声明参数
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 50.0)  # Hz

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        rate = self.get_parameter('publish_rate').value

        # 打开串口
        self.serial = SerialComm(port, baudrate)
        if not self.serial.open():
            self.get_logger().error(f"无法打开串口 {port}")
            raise RuntimeError("串口打开失败")
        self.get_logger().info(f"串口 {port} 已打开，IMU 协议")

        # 发布者：自定义 recive 消息
        self.recive_pub = self.create_publisher(Recive, '/recive_data', 10)

        # 订阅者：自定义 send 消息
        self.send_sub = self.create_subscription(Send, '/send_cmd', self.send_callback, 10)

        # 定时器：读取串口并发布
        self.timer = self.create_timer(1.0 / rate, self.read_and_publish)

        self.get_logger().info("节点初始化完成")

    def send_callback(self, msg: Send):
        """收到控制指令时发送到串口"""
        if self.serial.is_open():
            success = self.serial.send_command(msg.target_speed, msg.steer_angle)
            if not success:
                self.get_logger().warn("发送指令失败")
        else:
            self.get_logger().warn("串口未打开，无法发送指令")

    def read_and_publish(self):
        if not self.serial.is_open():
            return

        self.serial.read_into_buffer()
        parsed_list = self.serial.extract_frames()
        for data in parsed_list:
            self.publish_recive(data)

    def publish_recive(self, data: Dict[str, float]):
        """填充自定义 recive 消息并发布"""
        msg = Recive()
        msg.x = data['x']
        msg.y = data['y']
        msg.vx = data['vx']
        msg.vy = data['vy']
        msg.gyro_z_dps = data['gyro_z_dps']
        msg.yaw_deg = data['yaw_deg']
        msg.pitch_deg = data['pitch_deg']
        msg.roll_deg = data['roll_deg']
        self.recive_pub.publish(msg)
        self.get_logger().debug(
            f"发布: x={msg.x:.3f}, y={msg.y:.3f}, vx={msg.vx:.3f}, vy={msg.vy:.3f}, "
            f"gyro={msg.gyro_z_dps:.3f}, yaw={msg.yaw_deg:.2f}, "
            f"pitch={msg.pitch_deg:.2f}, roll={msg.roll_deg:.2f}"
        )

    def destroy_node(self):
        self.serial.close()
        super().destroy_node()


# ==================== 主函数 ====================
def main(args=None):
    rclpy.init(args=args)
    node = SerialRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
