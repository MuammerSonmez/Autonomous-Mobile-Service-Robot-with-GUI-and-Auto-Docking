#!/usr/bin/env python3
"""
Test için dummy robot node - Simülasyon
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import random

class TestRobotNode(Node):
    def __init__(self):
        super().__init__('test_robot_node')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            String, '/robot_command', self.command_callback, 10)
        self.vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Durum
        self.status = 'IDLE'
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Timer
        self.timer = self.create_timer(1.0, self.publish_status)
        self.odom_timer = self.create_timer(0.1, self.publish_odom)
        
        self.get_logger().info('Test robot node başlatıldı')
        
    def command_callback(self, msg):
        """Komut mesajlarını işle"""
        self.get_logger().info(f'Komut alındı: {msg.data}')
        
        if 'NEW_ORDER' in msg.data:
            self.status = 'MOVING'
            self.get_logger().info('Yeni sipariş - Mutfağa gidiliyor')
        elif msg.data == 'GO_HOME':
            self.status = 'MOVING'
            self.get_logger().info('Eve dönülüyor')
        elif msg.data == 'GO_KITCHEN':
            self.status = 'MOVING'
            self.get_logger().info('Mutfağa gidiliyor')
        elif msg.data == 'PAUSE':
            self.status = 'IDLE'
        elif msg.data == 'RESUME':
            self.status = 'MOVING'
        elif msg.data == 'EMERGENCY_STOP':
            self.status = 'ERROR'
            self.get_logger().warn('ACİL DURDURMA!')
            
    def velocity_callback(self, msg):
        """Hız komutlarını işle"""
        if msg.linear.x != 0 or msg.angular.z != 0:
            self.get_logger().info(f'Hız komutu: linear={msg.linear.x}, angular={msg.angular.z}')
            self.x += msg.linear.x * 0.1
            self.y += msg.linear.y * 0.1
            self.theta += msg.angular.z * 0.1
            
    def goal_callback(self, msg):
        """Hedef pozisyon al"""
        self.get_logger().info(f'Hedef: x={msg.pose.position.x}, y={msg.pose.position.y}')
        self.status = 'MOVING'
        
    def publish_status(self):
        """Durum yayınla"""
        msg = String()
        
        # Rastgele durum değişimi (simülasyon)
        if random.random() < 0.1:
            states = ['IDLE', 'MOVING', 'SERVING']
            self.status = random.choice(states)
            
        msg.data = self.status
        self.status_pub.publish(msg)
        
    def publish_odom(self):
        """Odometri yayınla"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()