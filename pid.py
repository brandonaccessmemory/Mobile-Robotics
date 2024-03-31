#!/usr/bin/env python
from math import pi, sqrt, atan2, cos, sin
import numpy as np
import threading

import rclpy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
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
        self.node.get_logger().info("Press Ctrl + C to terminate")
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel_2", 10)
        self.rate = self.node.create_rate(10)

        t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        t.start()

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = self.node.create_subscription(Odometry, "odom", self.odom_callback, 10)
    
        try:
            self.run()
        except KeyboardInterrupt:
            print('Interrupted')
        finally:
            # save trajectory to csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), delimiter=',')
            self.node.destroy_node()
            rclpy.shutdown()

    def run(self):
        # initialise pd controller 
        controller = Controller(P=1.0, D=0.2, set_point=0)

        msg = Twist() 
        # constant x velocity 
        msg.linear.x = 0.2

        # update angular velocity 
        target = [[4,0], [4,4], [0,4], [0,0]]


        for x,y in target:
            while(True):
                # current orientation 
                xdiff = x - self.pose.x 
                ydiff = y - self.pose.y
                
                if abs(xdiff) < 0.1 and abs(ydiff) < 0.1: 
                    break 

                reference = atan2(ydiff,xdiff) 
                theta = self.pose.theta
                # moving downwards, compensate for negative angle 
                if x == 0 and y == 4: 
                    if theta < 0: 
                        theta += 2 * pi

                    if reference < 0: 
                        reference += 2 * pi

                controller.setPoint(reference)
                angle = controller.update(theta)
                    
                msg.angular.z = angle 
                self.vel_pub.publish(msg)

        # last message to terminate robot
        msg.linear.x = 0.0 
        msg.angular.z = 0.0 
        self.vel_pub.publish(msg)

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            self.node.get_logger().info("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

def main(args=None):
    turtlebot = Turtlebot3()

if __name__ == '__main__':
    main()
