#! /usr/bin/env python
from typing import List
import rclpy
from rclpy.context import Context 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from math import pi 

class Mover(Node):
    def __init__(self, node_name: str = 'mover'):
        super().__init__(node_name=node_name)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        # denotes the total displacement done by the robot 
        self.linear = 0
        self.angular = 1
        self.target_reached = False
        self.get_logger().info("Mover has been initialized.")

    def timer_callback(self):
    
        msg = Twist()
        # reach a full square
        if self.linear == 80:
            self.stop_robot()
            return
        
        # i is 40 when 4m travelled, rotate 90 degrees = pi/2
        if self.linear != 0 and self.linear % 20 == 0 and not self.angular % 11 == 0:
            msg.angular.z = pi/20
            self.angular += 1
        # keep moving forward if havent reach 4m
        else: 
            msg.linear.x = 0.2
            self.linear += 1
            self.angular = 1

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        print("linear",self.linear)
        print("angular", self.angular)
        
    # terminate gracefully 
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.target_reached = True

def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    try:
        while not mover.target_reached:
            rclpy.spin_once(mover)
    except:
        mover.stop_robot()
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()