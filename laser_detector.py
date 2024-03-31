#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Laser(Node):
    def __init__(self, node_name: str = 'laser'):
        super().__init__(node_name=node_name)
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.laser_callback, 10)
        self.get_logger().info("Laser has been initialized.")
        self.scan_sub # Prevent unsued variable warning 

    def laser_callback(self):
        # Process laser scan data 
        ranges = msg.ranges 
        print('Front: ', ranges[0])
        print('Left: ', ranges[89])
        print('Back: ', ranges[179])
        print('Right: ', ranges[269])
    


def main(args=None):
    rclpy.init(args=args)
    laser_scan = Laser()
    rclpy.spin(laser_scan)
    laser_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()