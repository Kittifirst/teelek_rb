#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy import qos

def mm_to_cm(distance : float) -> float:
    return distance / 10.0

class RackDistance(Node):
    def __init__(self):
        super().__init__("rack_distance")

        # Publisher: rack_distance
        self.pub_move = self.create_publisher(Float32, "/teelek/rack_distance", qos_profile=qos.qos_profile_system_default)

        # Subscribe encoder_tick
        self.create_subscription(Twist, '/teelek/debug/encoder', self.teelek_encoder, qos_profile=qos.qos_profile_default)

    
        # Timer to send data every 0.1s``
        self.sent_data_timer = self.create_timer(0.1, self.sendData)

        self.rack_distance : float = 0.0


    def teelek_encoder(self, msg):
        tick_per_revolution = 60
        diameter = 26  # mm
        circumference = math.pi * diameter  # mm

        self.rack_distance += mm_to_cm((msg.linear.z / tick_per_revolution) * circumference)

    def sendData(self):
        rack_distance_msg = Float32()
        rack_distance_msg.data = float(self.rack_distance)
    
        self.pub_move.publish(rack_distance_msg)


def main():
    rclpy.init()
    node = RackDistance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
