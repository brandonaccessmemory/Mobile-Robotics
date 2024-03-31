#!/usr/bin/env python
from math import pi, sqrt, atan2, cos, sin
from rclpy.node import Node 
import rclpy
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import threading 

class Controller:
    # u(t)=kpe(t)+kd(e(t)−e(t−Δt))
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        # error is the the reference point - actual output / previous error 
        error = self.set_point - current_value 
        P_term = self.Kp * error 
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D


class Turtlebot3():
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.get_logger().info("Motion sensor bot initiated")

        self.rate = self.node.create_rate(10)
        t = threading.Thread(target = rclpy.spin, args=(self.node,), daemon=True)
        t.start() 

        # laser detector will update this variable 
        self.position = 0.5
        self.measurements = [0] * 5
        # publish to turtlebot 
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        # subscribe to laser_scan 
        self.scan_sub = self.node.create_subscription(LaserScan, "scan", self.laser_callback, 10)
        self.i = 0

        try:
            self.run()
        except KeyboardInterrupt:
            print('Interrupted')
        finally: 
            self.node.destroy_node() 
            rclpy.shutdown() 

    def run(self):
        # initialise pd controller , 0.3 as the stopping distance 
        controller = Controller(P=0.3, D=0.01, set_point=0.3)

        msg = Twist() 
        obstacle = False 

        while not obstacle: 
            if self.position <= 0.3: 
                obstacle = True 
                break
            else: 
                speed = controller.update(self.position)
                msg.linear.x = speed * -1

            self.vel_pub.publish(msg)

        # last message to terminate robot
        msg.linear.x = 0.0 
        self.vel_pub.publish(msg)

    def laser_callback(self, msg):
        # Process laser scan data 
        if self.i > 4: 
            self.i = 0

        self.measurements[self.i] = msg.ranges[0]
        # take average of last 5 values 
        front_obstacle = sum(self.measurements) / len(self.measurements)
        self.position = front_obstacle
        self.i += 1

def main(args=None):
    turtlebot = Turtlebot3()

if __name__ == '__main__':
    main()