import pygame
import serial
import time

# กำหนด Serial Port ของ ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # รอให้ ESP32 พร้อม

# เริ่ม pygame สำหรับอ่านจอย
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("ไม่พบจอย")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

def send_command(cmd):
    ser.write((cmd + '\n').encode())
    print("ส่งคำสั่ง:", cmd)

try:
    while True:
        pygame.event.pump()  # อัปเดตสถานะจอย

        x_axis = joystick.get_axis(0)  # แกน X (-1 ซ้าย, 1 ขวา)
        y_axis = joystick.get_axis(1)  # แกน Y (-1 หน้า, 1 หลัง)

        # Deadzone
        if abs(x_axis) < 0.2: x_axis = 0
        if abs(y_axis) < 0.2: y_axis = 0

        if y_axis < 0:
            send_command("forward")
        elif y_axis > 0:
            send_command("backward")
        elif x_axis < 0:
            send_command("left")
        elif x_axis > 0:
            send_command("right")
        else:
            send_command("stop")

        time.sleep(0.1)  # delay เพื่อไม่ให้ส่งเร็วเกิน
except KeyboardInterrupt:
    send_command("stop")
    ser.close()
    pygame.quit()
