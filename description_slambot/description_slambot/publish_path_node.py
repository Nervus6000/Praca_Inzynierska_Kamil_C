#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PublishPath(Node):
    def __init__(self):
        super().__init__("publish_path_node")       

        self.path_msg = Path()      
        self.path_msg.header.frame_id = "odom"  

        self.path_pub_ = self.create_publisher(Path, "/slambot_controller/trajectory", 10)
                                        
        self.odom_sub_ = self.create_subscription(Odometry, "/slambot_controller/odom", self.odom_callback, 10)

                                                                
    def odom_callback(self, msg: Odometry):

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Dodajemy do Path poprzez append() -> wrzuca na koniec listy
        self.path_msg.poses.append(pose_stamped)      
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        self.path_pub_.publish(self.path_msg)   

def main(args=None):
    rclpy.init(args=args)
    node = PublishPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
