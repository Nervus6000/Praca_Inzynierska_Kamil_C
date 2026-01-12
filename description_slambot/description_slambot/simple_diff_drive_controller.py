#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState        
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import euler2quat
from tf2_ros import TransformBroadcaster


class DiffDriveController(Node):
    def __init__(self):
        super().__init__("diff_drive_controller")

        self.wheel_radius_ = 0.05
        self.wheel_separation_ = 0.225

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0


        # Publikator poleceń prędkości koła  
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "/simple_velocity_controller/commands", 10) 

        # Subskrypcja sygnału prędkości liniowej i kątowej robota (cmd_vel) 
        self.vel_sub_ = self.create_subscription(TwistStamped ,"slambot_controller/cmd_vel", self.velCallback, 10)

        # Subskrypcja stanów przegubów (kąty kół z joint_state_broadcastera)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        # Publikator odometrii robota
        self.odom_pub_ = self.create_publisher(Odometry, "slambot_controller/odom", 10) 


        # Macierz przekształcenia prędkości kół robota
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0



    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])                 
        
        # Wyliczenie prędkości kół na podstawie macierzy konwersji
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        # WYWOŁANIE PUBLISHERA self.wheel_cmd_pub_  ZADEKLAROWANEGO W 34 LINJICE      
        self.wheel_cmd_pub_.publish(wheel_speed_msg)



    def jointCallback(self, msg):        

        dp_left = msg.position[1] - self.left_wheel_prev_pos_      #    position z joint state broadcaster
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # odometria
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        
        # szumy
        d_theta_noise = np.random.normal(0.0015, 0.02)                          
        d_s_noise = np.random.normal(0.0, 0.02)
        
        if abs(d_s) > 1e-6 or abs(d_theta)> 1e-6: 
            d_s = d_s + d_s_noise
            d_theta = d_theta + d_theta_noise
        else:
            d_s = d_s 
            d_theta = d_theta

        # całkowanie
        theta_mid = self.theta_ + d_theta / 2.0
        self.x_ += d_s * math.cos(theta_mid)
        self.y_ += d_s * math.sin(theta_mid)
        self.theta_ += d_theta

        q = euler2quat(0, 0, self.theta_) 

        self.odom_msg_.pose.pose.orientation.x = q[1]
        self.odom_msg_.pose.pose.orientation.y = q[2]
        self.odom_msg_.pose.pose.orientation.z = q[3]
        self.odom_msg_.pose.pose.orientation.w = q[0]

        current_time = msg.header.stamp                                        
        self.odom_msg_.header.stamp = current_time
        self.transform_stamped_.header.stamp = current_time

        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear               
        self.odom_msg_.twist.twist.angular.z = angular

        #   transformata odometrii:
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0

        #  UWAGA NA ZMIENIONĄ KOLEJNOŚĆ
        self.transform_stamped_.transform.rotation.x = q[1]
        self.transform_stamped_.transform.rotation.y = q[2]
        self.transform_stamped_.transform.rotation.z = q[3]
        self.transform_stamped_.transform.rotation.w = q[0] 

        self.odom_pub_.publish(self.odom_msg_)
        self.br_.sendTransform(self.transform_stamped_)

def main():
    rclpy.init()
    diff_drive_controller = DiffDriveController()
    rclpy.spin(diff_drive_controller)
    diff_drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
