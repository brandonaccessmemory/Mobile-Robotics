#!/usr/bin/env python
from math import pi
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
        self.front_position = 0.5
        self.front_right = 0.5 
        self.front_left = 0.5
        self.right_position = 0.5 
        self.back_position = 0.5

        # 225 degree and 315 degree of the robot 
        self.top_right = 0.5 
        self.bottom_right = 0.5 

        # past 5 distance measurements 
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
        # initialise controllers , 0.3 as the stopping distance 
        movement_controller = Controller(P=0.3, D=0.02, set_point=0.3)
        angle_controller = Controller(P=1.5, D=0.5, set_point=0.3)
        parallel_controller = Controller(P=1.5, D=0.5, set_point=0.0)
        
        msg = Twist()
        first_obstacle = False  
        obstacle = False 
        parallel = False

        # move straight first 
        while not first_obstacle: 
            if self.front_position < 0.3 or self.front_left < 0.3 or self.front_right < 0.3 :
                first_obstacle = True 
                
                while self.right_position > 0.3: 
                    print(self.right_position)

                    angle = angle_controller.update(self.right_position)
                    msg.angular.z = abs(angle) 
                    msg.linear.x = 0.0
                    self.vel_pub.publish(msg) 

                while not parallel: 
                    diff = self.top_right - self.bottom_right

                    if abs(diff) <= 0.01: 
                        parallel = True 
                    else: 
                        value = parallel_controller.update(diff)
                        msg.angular.z = value
                        msg.linear.x = 0.0 
                        self.vel_pub.publish(msg)

                parallel = False
            else: 
                print("moving forward")
                speed = movement_controller.update(self.front_position)
                msg.linear.x = abs(speed)
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)

        print("exit first loop")
                
        # keep following the wall 
        while True: 
            # reached an obstacle 
            if self.front_position < 0.3 or self.front_left < 0.3 or self.front_right < 0.3 : 
                obstacle = True 
            
            # adjust angle 
            if obstacle: 
                # start turning to desired orientation 
                while self.right_position > 0.3 or self.back_position > 0.3:
                    print("second" ,self.right_position)
                    # angle = angle_controller.update(self.back_position) 
                    angle = angle_controller.update(self.right_position) 
                    msg.angular.z = abs(angle) 
                    msg.linear.x = 0.0
                    self.vel_pub.publish(msg)

        
                while not parallel: 
                    diff = self.top_right - self.bottom_right
                    print("diff", diff)
                    if abs(diff) <= 0.01: 
                        parallel = True 
                    else: 
                        value = parallel_controller.update(diff)
                        msg.angular.z = abs(value)
                        msg.linear.x = 0.0 
                        self.vel_pub.publish(msg)
                
                parallel = False
                obstacle = False
            # move straight 
            else: 
                print("moving forward second")

                speed = movement_controller.update(self.front_position)
                msg.linear.x = abs(speed)

                diff = self.top_right - self.bottom_right

                value = parallel_controller.update(diff)
                msg.angular.z = value
                self.vel_pub.publish(msg)

                # msg.angular.z = 0.0

        # last message to terminate robot
        msg.linear.x = 0.0 
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def laser_callback(self, msg):
        # Process laser scan data 
        if self.i > 4: 
            self.i = 0

        front = msg.ranges[0]
        right = ( msg.ranges[269] + msg.ranges[260] + msg.ranges [278] ) / 3
        back = msg.ranges[179]

        if front == float('inf'):
            front = 4 
            
        if back == float('inf'):
            back = 4

        self.measurements[self.i] = front 

        # take average of last 5 values 
        front_obstacle = sum(self.measurements) / len(self.measurements)

        self.front_position = front_obstacle
        self.right_position = right
        self.back_position = back 

        self.front_left = msg.ranges[14] 
        self.front_right = msg.ranges[345]

        self.top_right = msg.ranges[314]
        self.bottom_right = msg.ranges[224]
        self.i += 1

def main(args=None):
    turtlebot = Turtlebot3()

if __name__ == '__main__':
    main()