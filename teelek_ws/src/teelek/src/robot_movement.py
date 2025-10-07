#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist

def mm_to_cm(distance: float) -> float:
    return distance / 10.0

class RackDistance(Node):
    def __init__(self):
        super().__init__("rack_distance_node")

        # ------------------------------
        # Publishers
        # ------------------------------
        self.pub_rack_distance = self.create_publisher(Float32, "/teelek/rack_distance", 10)
        self.pub_robot_distance = self.create_publisher(Float32, "/teelek/robot_distance", 10)

        # ------------------------------
        # Subscribers
        # ------------------------------
        self.create_subscription(Float32MultiArray, "/teelek/debug/encoder_wheels",
                                 self.teelek_encoder_wheels, 10)
        self.create_subscription(Twist, "/teelek/debug/encoder",
                                 self.teelek_encoder_rack, 10)

        # ------------------------------
        # Timer ส่งข้อมูลทุก 0.1s
        # ------------------------------
        self.sent_data_timer = self.create_timer(0.1, self.sendData)

        # ------------------------------
        # ตัวแปรสะสม
        # ------------------------------
        self.robot_distance = 0.0
        self.rack_distance = 0.0

        # เก็บ tick ล่าสุดเพื่อคำนวณ delta
        self.prev_wheel_tick = [0, 0, 0, 0]
        self.prev_rack_tick = 0

    # ------------------------------
    # Callback encoder wheels
    # ------------------------------
    def teelek_encoder_wheels(self, msg: Float32MultiArray):
        tick_per_revolution = 541.0168 * 11 * 4
        diameter = 130  # mm
        circumference = math.pi * diameter  # mm

        ticks = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
        delta_ticks = [ticks[i] - self.prev_wheel_tick[i] for i in range(4)]
        self.prev_wheel_tick = ticks

        avg_delta = sum(delta_ticks) / 4.0
        self.robot_distance += mm_to_cm((avg_delta / tick_per_revolution) * circumference)

    # ------------------------------
    # Callback encoder rack
    # ------------------------------
    def teelek_encoder_rack(self, msg: Twist):
        tick_per_revolution = 60
        diameter = 26  # mm
        circumference = math.pi * diameter  # mm

        delta_tick = msg.linear.z - self.prev_rack_tick
        self.prev_rack_tick = msg.linear.z

        self.rack_distance += mm_to_cm((delta_tick / tick_per_revolution) * circumference)

    # ------------------------------
    # ส่งข้อมูล
    # ------------------------------
    def sendData(self):
        rack_msg = Float32()
        rack_msg.data = float(self.rack_distance)
        self.pub_rack_distance.publish(rack_msg)

        robot_msg = Float32()
        robot_msg.data = float(self.robot_distance)
        self.pub_robot_distance.publish(robot_msg)


def main():
    rclpy.init()
    node = RackDistance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
