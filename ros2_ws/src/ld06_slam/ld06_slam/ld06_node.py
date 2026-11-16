#!/usr/bin/env python3
"""ROS 2 Node publishing sensor_msgs/LaserScan from an LD06 LiDAR over serial."""

from __future__ import annotations

import math
import sys
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

PKG_HEADER = 0x54
PKG_VER_LEN = 0x2C
POINTS_PER_PACKET = 12
PACKET_SIZE = 47

CRC_TABLE: Tuple[int, ...] = (
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
)


class LD06Node(Node):
    """ROS 2 publisher for LD06 LiDAR scans."""

    def __init__(self) -> None:
        super().__init__("ld06_node")

        # Declare parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("min_confidence", 100)
        self.declare_parameter("num_samples", 450)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.range_min = self.get_parameter("range_min").get_parameter_value().double_value
        self.range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self.min_confidence = self.get_parameter("min_confidence").get_parameter_value().integer_value
        self.num_samples = self.get_parameter("num_samples").get_parameter_value().integer_value

        self.publisher = self.create_publisher(LaserScan, "scan", 10)

        # Connect serial
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to LD06 LiDAR on {self.port} at {self.baudrate} baud.")
        except serial.SerialException as exc:
            self.get_logger().error(f"Failed to open serial port {self.port}: {exc}")
            sys.exit(1)

        # Buffer for current 360-degree sweep
        self.current_scan: List[Tuple[float, float, int]] = []
        self.last_angle: float = 0.0
        self.scan_start_time = self.get_clock().now()

        # Timer for polling serial port at high frequency (~200Hz)
        self.timer = self.create_timer(0.005, self.read_and_publish)

    @staticmethod
    def verify_crc(packet: bytes) -> bool:
        crc = 0
        for byte in packet[:46]:
            crc = CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc == packet[46]

    def read_and_publish(self) -> None:
        if not self.serial or not self.serial.is_open:
            return

        while self.serial.in_waiting >= PACKET_SIZE:
            header = self.serial.read(1)
            if not header or header[0] != PKG_HEADER:
                continue

            ver_len = self.serial.read(1)
            if not ver_len or ver_len[0] != PKG_VER_LEN:
                continue

            rest = self.serial.read(PACKET_SIZE - 2)
            if len(rest) != PACKET_SIZE - 2:
                continue

            packet = bytes([PKG_HEADER, PKG_VER_LEN]) + rest
            if not self.verify_crc(packet):
                continue

            # Decode angles
            start_angle = (packet[4] | (packet[5] << 8)) / 100.0
            end_angle = (packet[42] | (packet[43] << 8)) / 100.0

            diff_angle = (end_angle - start_angle) % 360.0
            step_angle = diff_angle / (POINTS_PER_PACKET - 1)

            # Check for revolution wrap-around (360 -> 0)
            if start_angle < self.last_angle and self.last_angle > 300.0 and start_angle < 60.0:
                self.publish_laserscan()
                self.current_scan.clear()
                self.scan_start_time = self.get_clock().now()

            self.last_angle = start_angle

            # Accumulate points
            for i in range(POINTS_PER_PACKET):
                offset = 6 + i * 3
                dist_m = (packet[offset] | (packet[offset + 1] << 8)) / 1000.0
                conf = packet[offset + 2]
                angle_deg = (start_angle + i * step_angle) % 360.0
                self.current_scan.append((angle_deg, dist_m, conf))

    def publish_laserscan(self) -> None:
        if not self.current_scan:
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.scan_start_time.to_msg()
        scan_msg.header.frame_id = self.frame_id

        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / self.num_samples
        scan_msg.scan_time = 0.1  # 10 Hz
        scan_msg.time_increment = scan_msg.scan_time / self.num_samples
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Initialize uniform range bins
        ranges = [float("inf")] * self.num_samples
        intensities = [0.0] * self.num_samples

        for angle_deg, dist_m, conf in self.current_scan:
            if self.range_min <= dist_m <= self.range_max and conf >= self.min_confidence:
                # Bin index
                rad = math.radians(angle_deg) % (2.0 * math.pi)
                idx = int(rad / scan_msg.angle_increment)
                if 0 <= idx < self.num_samples:
                    ranges[idx] = dist_m
                    intensities[idx] = float(conf)

        scan_msg.ranges = ranges
        scan_msg.intensities = intensities

        self.publisher.publish(scan_msg)

    def destroy_node(self) -> None:
        if hasattr(self, "serial") and self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LD06Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
