#!/usr/bin/env python3
import sys
import math
import numpy as np
from rclpy import init, spin, shutdown
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# PID CONTROL PARAMS
kp = -15
kd = 5
ki = 0

prev_error = 0.0 
error = 0.0
integral = 0.0


# WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9  # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00  # meters per second
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow(Node):
    """ Implement Wall Following on the car """
    def __init__(self):
        super().__init__('wall_follow_node')
        # Topics & Subs, Pubs
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.prev_time = self.get_clock().now()
    def get_range(self, data, angle):
        """ Get range data for a specified angle """
        ranges = data.ranges
        angle_incr = np.degrees(data.angle_increment)
        ang_min = np.degrees(data.angle_min)
        idx = int((angle - ang_min) /angle_incr)
        result = ranges[idx]

        if not np.isnan(result) and not np.isinf (result):
            return result
        else:
            return None    
        

        # """ Get range data for a specified angle """
        # ranges = np.array(range_data.ranges)
        # angle_incr = range_data.angle_increment
        # desired_idx = int((np.radians(angle) - range_data.angle_min) / angle_incr)
        # if np.isfinite(ranges[desired_idx]):
        #     return ranges[desired_idx]
        # else:
        #     return None

    def pid_control(self, error):

        global integral, prev_error, kp, ki, kd, prev_time

        velocity = 0.8

        # get time and get change in time 
        curr_time = self.get_clock().now()
        delta_t = (curr_time - self.prev_time).nanoseconds * 1e-9 

        # Update the PID parameters
        integral += error * delta_t
        derivative = (prev_error - error) / delta_t
        
        # Calculate the angle
        angle = np.radians(kp * error + ki * integral + kd * derivative)
        
        # # clamp the angle (avoid sharp turns)
        # # Clamp the steering angle using if statements
        # if angle > 0.349:  # 20 degrees in radians
        #     angle = 0.349
        # elif angle < -0.349:  # -20 degrees in radians
        #     angle = -0.349
        # update 
        prev_error = error
        prev_time = curr_time

        # yoinked 
        # Log the values for debugging
        self.get_logger().info(f"Error: {error:.2f}, Angle: {angle:.2f}, Velocity: {velocity:.2f}")

        # publish drive message 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = angle 
        drive_msg.drive.speed = velocity 
        self.drive_pub.publish(drive_msg) 



        # global integral, prev_error, kp, ki, kd
        # deriv_error = error - prev_error
        # prev_error = error
         
        # integral += error
        # angle = np.radians(kp * error + kd * deriv_error + ki * integral)
        # velocity = self.calc_speed(angle)
        # self.get_logger().info(f"Error: {error}, Angle: {np.degrees(angle)}")

        # drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = self.get_clock().now().to_msg()
        # drive_msg.header.frame_id = "laser"
        # drive_msg.drive.steering_angle = angle
        # drive_msg.drive.speed = velocity
        # self.drive_pub.publish(drive_msg)

    # def calc_speed(self, angle):
    #     angle = np.abs(np.degrees(angle))
    #     if angle < 10:
    #         speed = 1.5
    #     elif angle < 20:
    #         speed = 1.0
    #     else:
    #         speed = 0.5
    #     return speed

    def follow_left(self, data):
        
        theta = 40
        lookahead = 0.7 # Lookahead distance (car length)
        b = self.get_range(data, 90) # 90 directly to the wall 
        a = self.get_range(data, 90 - theta)
        
        theta = np.radians(theta) # to radians chagned 
        alpha = np.arctan2((a* np.cos(theta) - b), (a * np.sin(theta)))
        Dt = b * np.cos(alpha) # current position        
        DtPlus = Dt + lookahead * np.sin(alpha) # projection 

        error = DESIRED_DISTANCE_LEFT - DtPlus   #OUR dist is being passed in 
        return error 
    
        # zero_angle = 90
        # b = self.get_range(data, zero_angle)
        
        # theta = 40
        # a = self.get_range(data, zero_angle - theta)
        # theta = np.radians(theta)
        # if b is not None and a is not None:
        #     alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
        #     Dleft = b * np.cos(alpha)

        #     D_left_lookahead = Dleft + CAR_LENGTH * np.sin(alpha)

        #     error = DESIRED_DISTANCE_LEFT - D_left_lookahead
        #     return error
        # else:
        #     return None


    def lidar_callback(self, data):
        """ Lidar callback for processing data """
        error = self.follow_left(data)
        if error is not None:
            self.pid_control(error)

def main(args=None):
    init(args=args)
    wall_follow = WallFollow()
    spin(wall_follow)
    wall_follow.destroy_node()
    shutdown()

if __name__ == '__main__':
    main()
