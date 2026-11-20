#!/usr/bin/env python3
"""Simulated RTK base station for one UAV: publishes mavros/gps_rtk/rtk_baseline as the F9P would (started by the spawner for --enable-rtk)."""

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from mavros_msgs.msg import RTKBaseline
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

FLOAT_NOISE_GAIN = 15.0  # noise before the fix (float solution), relative to the fixed one


def baseline_ned_mm(antenna_x, antenna_y, antenna_z, x, y, z, noise=0.0):
    """NED vector from the base antenna to the UAV (both in ENU), in mm, with gaussian noise [m std]."""
    def n():
        return random.gauss(0.0, noise) if noise > 0 else 0.0
    return (round((y - antenna_y + n()) * 1000.0),
            round((x - antenna_x + n()) * 1000.0),
            round((antenna_z - z + n()) * 1000.0))


class RtkBase(Node):

    def __init__(self):
        super().__init__('rtk_base')
        self.antenna_topic = self.declare_parameter('antenna_topic', 'rtk/odom').value
        self.noise = self.declare_parameter('noise', 0.02).value
        self.fix_delay = self.declare_parameter('fix_delay', 5.0).value
        self.base = [self.declare_parameter(p, 0.0).value for p in ('base_x', 'base_y', 'base_ground_z')]
        self.pos, self.tripod, self.t0 = None, None, None

        self.create_subscription(Odometry, self.antenna_topic, self.on_antenna, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, '/rtk_base/tripod_height', self.on_tripod, latched)
        self.pub = self.create_publisher(RTKBaseline, 'mavros/gps_rtk/rtk_baseline', 10)
        self.create_timer(0.2, self.publish)

    def on_antenna(self, msg):
        self.pos = msg.pose.pose.position
        if self.t0 is None:
            self.t0 = self.get_clock().now()

    def on_tripod(self, msg):
        self.tripod = msg.data

    def publish(self):
        if self.pos is None or self.tripod is None:
            self.get_logger().info(f'waiting for {self.antenna_topic} and /rtk_base/tripod_height', throttle_duration_sec=2.0)
            return
        now = self.get_clock().now()
        fixed = (now - self.t0).nanoseconds * 1e-9 >= self.fix_delay
        sigma = self.noise if fixed else self.noise * FLOAT_NOISE_GAIN
        msg = RTKBaseline()
        msg.header.stamp = now.to_msg()
        msg.rtk_health = 1 if fixed else 0
        msg.iar_num_hypotheses = msg.rtk_health
        msg.baseline_coords_type = RTKBaseline.RTK_BASELINE_COORDINATE_SYSTEM_NED
        bx, by, bz = self.base
        msg.baseline_a_mm, msg.baseline_b_mm, msg.baseline_c_mm = baseline_ned_mm(
            bx, by, bz + self.tripod, self.pos.x, self.pos.y, self.pos.z, sigma)
        msg.accuracy = max(1, round(sigma * 1000.0))
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(RtkBase())


if __name__ == '__main__':
    # self-check: a UAV 10 m up, 2 m north, 3 m east of a 1.5 m tripod is 8.5 m above the antenna
    assert baseline_ned_mm(0, 0, 1.5, 3.0, 2.0, 10.0) == (2000, 3000, -8500)
    random.seed(0)
    c = [baseline_ned_mm(0, 0, 1.5, 0, 0, 0, 0.02)[2] - 1500 for _ in range(2000)]
    assert 18.0 < (sum(x * x for x in c) / len(c)) ** 0.5 < 22.0
    print('self-check ok')
