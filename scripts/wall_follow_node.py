#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers 
        # LaserScan subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,               
            self.scan_callback,
            10
            )
        self.scan_subscription  

        # create publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # # TODO: set PID gains
        # self.kp = self.declare_parameter('P', 2.0).get_parameter_value().double_value
        # self.kd = self.declare_parameter('D', 2.0).get_parameter_value().double_value
        # self.ki = self.declare_parameter('I', 2.0).get_parameter_value().double_value
        self.kp = 1.0
        self.kd = .01
        self.ki = .01

        # TODO: store history
        self.integral = 0.
        self.prev_error = 0.
        self.error = 0.

        # previous time set - need to_msg? 
        self.prev_time = self.get_clock().now()


        # TODO: store any necessary values you think you'll need
        self.speed = self.declare_parameter('speed', 2.0).get_parameter_value().double_value

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        # check within range? IndexError: array index out of range
        # to avoid array index out of range error subtract ang_min from angle 
        # TO-DO document explination
        
        ranges = range_data.ranges      # get range array 
        ang_min = range_data.angle_min  # get angle min to correct index value
        ang_inc = np.degrees(range_data.angle_increment)    # get angle increment & cast to degrees

        # find index that corresponds to array angle

        index = int(angle - ang_min/ang_inc)   
        result = ranges[index] # the distance is the result 

        # if nan or inf don't return (error message?)
        if not np.isnan(result) and not np.isinf (result):
            return result
        else:
            return None


    # implements follow left algorithm 
    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        # positive error - too far from left wall, turn left (neg angle)
        # negative error - too close to left wall, turn right (pos angle)

        # points a and b
        # a : 0 < theta < 70 
        # b : 270 (90 degrees but to the left)

        # if statement might be needed if a / b don't return anything 
        theta = 30
        lookahead = 0.7 # Lookahead distance (car length)
        b = self.get_range(range_data, -90) # 90 directly to the wall 
        a = self.get_range(range_data, -90 + theta)
        
        alpha = np.arctan((a* np.cos(theta) - b)/ (a * np.sin(theta)))
        Dt = b * np.cos(alpha) # current position        
        DtPlus = Dt + lookahead * np.sin(alpha) # projection 

        error = dist - DtPlus
        return error 

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """

        # get time and get change in time 
        curr_time = self.get_clock().now()
        delta_t = (curr_time - self.prev_time).nanoseconds * 1e-9 

        # Update the PID parameters
        self.integral += error * delta_t
        derivative = (self.prev_error - error) / delta_t
        
        # Calculate the angle
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # clamp the angle (avoid sharp turns)
        
        # update 
        self.prev_error = error
        self.prev_time = curr_time

        # Log the values for debugging
        self.get_logger().info(f"Error: {error:.2f}, Angle: {angle:.2f}, Velocity: {velocity:.2f}")

        # publish drive message 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = -angle # negative? 
        drive_msg.drive.speed = velocity 
        self.publisher_.publish(drive_msg) 




    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message
        Returns:
            None
        """
        
        # calculate error 
        # publish drive error 

        # error = 0.0 # TODO: replace with error calculated by get_error()
        distance = .85
        error = self.get_error(msg, distance) # 1 meter from left wall 

        # calculate velocity 
        # 0-10 1.5 m/s
        # 10-20 1.0 m/s
        # 0.5 m/s 

        velocity = 0.3 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()