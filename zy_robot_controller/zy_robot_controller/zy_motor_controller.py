#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import os

class ZyBaseMotorController(Node):
    SERIAL_PORT = '/dev/ttyACM0'
    PWD_MIN = 10 # Minimum PWD supported by base
    LINEAR_TO_PWD = 100 # if linear.x = 1m/s then set speed (pwd) to 100
    ANGULAR_TO_PWD = 90 # if angular.z = 1 rad/s then set speed (pwd) to 100
    def __init__(self):
        super().__init__("zy_motor_controller")
        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found:" + self.SERIAL_PORT + " Zy_motor_controller not started!")
            rclpy.shutdown()
        self.ser = serial.Serial(self.SERIAL_PORT,9600) # open serial port
        self.send_serial('') # Send to clear out any noise in the serial buffer
        self.twist_subscriber = self.create_subscription(
            Twist, "/base/cmd_vel", self.send_cmd_vel, 10)
        self.get_logger().info("Zy motor controller has started ...")
    
    def send_cmd_vel(self,msg):
        self.get_logger().info("Twist: Linear velocity: %f Angular velocity: %f" % (msg.linear.x, msg.angular.z))
        if msg.linear.x != 0:
            # We are making a linear movement
            direction = 1
            # Set pwd and adjust to limits
            if msg.linear.x < 0:
                direction = -1
            pwd = int(msg.linear.x * self.LINEAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN:
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100:
                pwd = 100 * direction
            self.get_logger().info("sending serial:" + "RV" + "%+4d" % pwd)
            self.send_serial("RV" + "%+4d" % pwd)
        elif msg.angular.z !=0:
            # We are making a turn
            direction = 1
            # Set pwd and adjust to limits
            if msg.angular.x < 0:
                direction = -1
            pwd = int(msg.linear.x + self.ANGULAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN:
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100:
                pwd = 100 * direction
            self.get_logger().info("sending serial:" + "TV" + "%+4d" % pwd)
            self.send_serial("TV" + "%+4d" % pwd)
        else:
            self.get_logger().info("sending serial:" + "SP")
            self.send_serial("SP")
    def send_serial(self, send):
        self.ser.write((send + "\n").encode())

def main(args=None):
    rclpy.init(args=args)
    node = ZyBaseMotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
