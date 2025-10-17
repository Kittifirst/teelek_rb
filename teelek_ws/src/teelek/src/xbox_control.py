#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy import qos

class Gamepad:
    def __init__(self):
        # Axes:--------------------------------------------------------
        self.lx = 0.0                   # 0: Left Stick X
        self.ly = 0.0                   # 1: Left Stick Y
        self.l2 = 0.0                   # 2: Left Trigger
        self.rx = 0.0                   # 3: Right Stick X
        self.ry = 0.0                   # 4: Right Stick Y
        self.r2 = 0.0                   # 5: Right Trigger
        self.dpadLeftRight = 0.0        # 6: Dpad Left/Right
        self.dpadUpDown = 0.0           # 7: Dpad Up/Down

        # Buttons:-------------------------------------------------------
        self.button_a = 0.0             # 0: A
        self.button_b = 0.0             # 1: B
        self.button_x = 0.0             # 2: X
        self.button_y = 0.0             # 3: Y
        self.lb = 0.0                   # 4: LB
        self.rb = 0.0                   # 5: RB
        self.button_view = 0.0          # 6: View (-)
        self.button_menu = 0.0          # 7: Menu (+) -> ใช้ toggle encoder
        self.button_xbox = 0.0          # 8: Xbox/Logo
        self.ls_press = 0.0             # 9: LS Press
        self.rs_press = 0.0             # 10: RS Press

class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        # Publisher: wheel move
        self.pub_move = self.create_publisher(Twist, "/teelek/cmd_move", qos_profile=qos.qos_profile_system_default)
        # Publisher: load motor_l
        self.pub_loadleft = self.create_publisher(Twist, "/teelek/cmd_loadleft", qos_profile=qos.qos_profile_system_default)
        # Publisher: load motor_r
        self.pub_loadright = self.create_publisher(Twist, "/teelek/cmd_loadright", qos_profile=qos.qos_profile_system_default)
        # Publisher: servo
        self.pub_servo = self.create_publisher(Twist, "/teelek/cmd_servo", qos_profile=qos.qos_profile_system_default)
        # Publisher: encoder
        self.pub_encoder = self.create_publisher(Twist, "/teelek/cmd_resetencoder", qos_profile=qos.qos_profile_system_default)
        # Subscribe joystick
        self.create_subscription(Joy, '/joy', self.joy, qos_profile=qos.qos_profile_sensor_data)

        # Gamepad object
        self.gamepad = Gamepad()

        # Max speed constants
        self.maxspeed = 1023.0
        self.maxloadspeed = 650.0

        # Encoder
        self.resetencoder = 1.0

        # Timer to send data every 0.1s
        self.sent_data_timer = self.create_timer(0.1, self.sendData)

    def joy(self, msg):
        # ---------------- Axes ----------------
        self.gamepad.lx = float(msg.axes[0] * -1)           # Left Stick X
        self.gamepad.ly = float(msg.axes[1])                # Left Stick Y
        self.gamepad.l2 = float((msg.axes[2] + 1) / 2)     # LT 0..1
        self.gamepad.rx = float(msg.axes[3] * -1)          # Right Stick X
        self.gamepad.ry = float(msg.axes[4])               # Right Stick Y
        self.gamepad.r2 = float((msg.axes[5] + 1) / 2)     # RT 0..1
        self.gamepad.dpadLeftRight  = float(msg.axes[6])   # Dpad Left/Right
        self.gamepad.dpadUpDown     = float(msg.axes[7])   # Dpad Up/Down

        # ---------------- Buttons ----------------
        self.gamepad.button_a      = float(msg.buttons[0])  # A
        self.gamepad.button_b      = float(msg.buttons[1])  # B
        self.gamepad.button_x      = float(msg.buttons[2])  # X
        self.gamepad.button_y      = float(msg.buttons[3])  # Y
        self.gamepad.lb            = float(msg.buttons[4])  # LB
        self.gamepad.rb            = float(msg.buttons[5])  # RB
        self.gamepad.button_view   = float(msg.buttons[6])  # View (-)
        self.gamepad.button_menu   = float(msg.buttons[7])  # Menu (+)
        self.gamepad.button_xbox   = float(msg.buttons[8])  # Xbox/Logo
        self.gamepad.ls_press      = float(msg.buttons[9])  # LS Press
        self.gamepad.rs_press      = float(msg.buttons[10]) # RS Press

        # Reset toggles if Xbox button pressed
        if self.gamepad.button_xbox:
            self.gamepad.reset_toggles()

    def sendData(self):
        cmd_vel_move = Twist()
        cmd_loadleft = Twist()
        cmd_loadright = Twist()
        cmd_servo = Twist()
        cmd_encoder = Twist()

        # Wheel movement
        cmd_vel_move.linear.x = float(self.gamepad.ly * self.maxspeed)
        cmd_vel_move.linear.y = float(self.gamepad.lx * self.maxspeed * -1)
        cmd_vel_move.angular.z = float(self.gamepad.rx * self.maxspeed * -1)

        # Load motor control using Dpad Left/Right
        # cmd_loadleft.linear.x = float(self.gamepad.lb * self.maxloadspeed * -1) 
        cmd_loadleft.linear.x = float(self.gamepad.dpadLeftRight * self.maxloadspeed * 1) 
        # cmd_loadleft.linear.x = float(self.gamepad.l2  * -self.maxloadspeed)
        # cmd_loadright.linear.x = float(self.gamepad.rb  * self.maxloadspeed * -1)
        cmd_loadright.linear.x = float(self.gamepad.dpadUpDown  * self.maxloadspeed * 1)
        # cmd_loadright.linear.x = float(self.gamepad.r2  * self.maxloadspeed)

        # servo
        if not hasattr(self, 'servo_last_value'):
            self.servo_last_value = 800.0 

        if self.gamepad.button_a:
            self.servo_last_value = 800.0
        elif self.gamepad.button_y:
            self.servo_last_value = 1400.0
        cmd_servo.angular.x = self.servo_last_value
        # Encoder Setpoint
        cmd_encoder.linear.x = float(self.gamepad.button_menu * self.resetencoder)
        
        # Publish
        self.pub_move.publish(cmd_vel_move)
        self.pub_loadleft.publish(cmd_loadleft)
        self.pub_loadright.publish(cmd_loadright)
        self.pub_servo.publish(cmd_servo)
        self.pub_encoder.publish(cmd_encoder)

def main():
    rclpy.init()
    node = Joystick()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
