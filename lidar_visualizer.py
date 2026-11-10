#!/usr/bin/env python3
"""LD06 LiDAR real-time visualizer using Pygame.

Decodes the proprietary LD06 47-byte serial protocol, converts polar coordinates
to cartesian metrics, and renders a live radar display.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import sys
from typing import Deque, Iterator, List, Optional, Tuple

import pygame
import serial

# ==============================================================================
# LD06 Protocol Constants
# ==============================================================================
PKG_HEADER = 0x54
PKG_VER_LEN = 0x2C
POINTS_PER_PACKET = 12
PACKET_SIZE = 47

# Precomputed CRC8 lookup table (polynomial 0x4D)
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


@dataclass(slots=True)
class LidarMeasurement:
    """Individual LiDAR sample point."""
    angle_deg: float
    distance_m: float
    confidence: int

    @property
    def cartesian(self) -> Tuple[float, float]:
        """Convert polar measurement to cartesian (x, y) in sensor coordinates."""
        rad = math.radians(self.angle_deg)
        return self.distance_m * math.cos(rad), self.distance_m * math.sin(rad)


class LD06Driver:
    """Serial communication handler and frame decoder for the LD06 LiDAR."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 230400, timeout: float = 0.1) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None

    def connect(self) -> None:
        """Open the serial connection."""
        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self) -> None:
        """Close the serial connection cleanly."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    @staticmethod
    def _verify_crc(packet: bytes) -> bool:
        """Validate packet integrity against LD06 CRC8."""
        crc = 0
        for byte in packet[:46]:
            crc = CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc == packet[46]

    @staticmethod
    def _decode_packet(packet: bytes) -> List[LidarMeasurement]:
        """Extract measurement samples from a verified 47-byte frame."""
        start_angle = (packet[4] | (packet[5] << 8)) / 100.0
        end_angle = (packet[42] | (packet[43] << 8)) / 100.0

        diff_angle = (end_angle - start_angle) % 360.0
        step_angle = diff_angle / (POINTS_PER_PACKET - 1)

        measurements: List[LidarMeasurement] = []
        for i in range(POINTS_PER_PACKET):
            offset = 6 + i * 3
            distance_mm = packet[offset] | (packet[offset + 1] << 8)
            confidence = packet[offset + 2]
            angle = (start_angle + i * step_angle) % 360.0
            measurements.append(LidarMeasurement(angle, distance_mm / 1000.0, confidence))

        return measurements

    def read_measurements(self) -> Iterator[LidarMeasurement]:
        """Synchronize with the serial stream and yield decoded measurements."""
        if not self._serial or not self._serial.is_open:
            return

        while self._serial.in_waiting >= PACKET_SIZE:
            header = self._serial.read(1)
            if not header or header[0] != PKG_HEADER:
                continue

            ver_len = self._serial.read(1)
            if not ver_len or ver_len[0] != PKG_VER_LEN:
                continue

            rest = self._serial.read(PACKET_SIZE - 2)
            if len(rest) != PACKET_SIZE - 2:
                continue

            packet = bytes([PKG_HEADER, PKG_VER_LEN]) + rest
            if self._verify_crc(packet):
                for measurement in self._decode_packet(packet):
                    yield measurement


class LidarVisualizer:
    """Pygame-based live radar display for LD06 point cloud."""

    # UI Theme
    COLOR_BG = (16, 20, 30)
    COLOR_GRID = (45, 55, 75)
    COLOR_TEXT = (210, 220, 235)
    COLOR_POINTS = (0, 255, 175)
    COLOR_ROBOT = (255, 80, 80)

    def __init__(
        self,
        driver: LD06Driver,
        window_size: int = 800,
        scale: float = 90.0,
        buffer_size: int = 480,
        min_distance: float = 0.05,
        max_distance: float = 12.0,
        min_confidence: int = 120,
    ) -> None:
        self.driver = driver
        self.window_size = window_size
        self.center = (window_size // 2, window_size // 2)
        self.scale = scale
        self.min_dist = min_distance
        self.max_dist = max_distance
        self.min_conf = min_confidence

        self.points_buffer: Deque[Tuple[int, int]] = deque(maxlen=buffer_size)

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((window_size, window_size))
        pygame.display.set_caption("LD06 LiDAR Real-time Radar")
        self.font = pygame.font.SysFont("monospace", 13, bold=True)
        self.clock = pygame.time.Clock()

    def _world_to_screen(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Project metric sensor coordinates to display pixel coordinates."""
        cx, cy = self.center
        return int(cx + x_m * self.scale), int(cy - y_m * self.scale)

    def _render_grid(self) -> None:
        """Render radar range rings and center crosshairs."""
        cx, cy = self.center

        # Range rings at 1m increments (up to 4m)
        for meter in range(1, 5):
            radius = int(meter * self.scale)
            pygame.draw.circle(self.screen, self.COLOR_GRID, (cx, cy), radius, 1)
            lbl = self.font.render(f"{meter}m", True, self.COLOR_TEXT)
            self.screen.blit(lbl, (cx + radius + 4, cy - 10))

        # Central axes
        pygame.draw.line(self.screen, self.COLOR_GRID, (cx, 0), (cx, self.window_size), 1)
        pygame.draw.line(self.screen, self.COLOR_GRID, (0, cy), (self.window_size, cy), 1)

        # Sensor origin indicator
        pygame.draw.circle(self.screen, self.COLOR_ROBOT, (cx, cy), 5)

    def _render_hud(self) -> None:
        """Render telemetry metrics overlay."""
        fps = int(self.clock.get_fps())
        metrics = [
            f"Points : {len(self.points_buffer)}",
            f"FPS    : {fps}",
            f"Scale  : {self.scale:.0f} px/m",
            f"Port   : {self.driver.port}",
        ]
        for idx, text in enumerate(metrics):
            surface = self.font.render(text, True, self.COLOR_TEXT)
            self.screen.blit(surface, (15, 15 + idx * 20))

    def run(self) -> None:
        """Main acquisition and rendering loop."""
        self.driver.connect()
        running = True

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False

                # Process all newly available measurements
                for sample in self.driver.read_measurements():
                    if (
                        self.min_dist <= sample.distance_m <= self.max_dist
                        and sample.confidence >= self.min_conf
                    ):
                        x, y = sample.cartesian
                        sx, sy = self._world_to_screen(x, y)
                        if 0 <= sx < self.window_size and 0 <= sy < self.window_size:
                            self.points_buffer.append((sx, sy))

                # Render frame
                self.screen.fill(self.COLOR_BG)
                self._render_grid()

                for px, py in self.points_buffer:
                    pygame.draw.circle(self.screen, self.COLOR_POINTS, (px, py), 2)

                self._render_hud()
                pygame.display.flip()
                self.clock.tick(120)

        finally:
            self.driver.close()
            pygame.quit()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LD06 LiDAR Real-time Radar Visualizer")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=230400, help="Baud rate (default: 230400)")
    parser.add_argument("--scale", type=float, default=90.0, help="Display scale in px/m (default: 90)")
    parser.add_argument("--window-size", type=int, default=800, help="Window resolution in px (default: 800)")
    parser.add_argument("--min-conf", type=int, default=120, help="Minimum point confidence (default: 120)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    driver = LD06Driver(port=args.port, baudrate=args.baud)
    visualizer = LidarVisualizer(
        driver=driver,
        window_size=args.window_size,
        scale=args.scale,
        min_confidence=args.min_conf,
    )
    visualizer.run()


if __name__ == "__main__":
    main()
