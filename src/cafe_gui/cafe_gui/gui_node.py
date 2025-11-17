#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from cafe_gui.main_window import MainWindow

class GUINode(Node):
    def __init__(self):
        super().__init__('cafe_gui_node')
        self.get_logger().info('Cafe GUI Node başlatıldı')

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    
    node = GUINode()
    
    main_window = MainWindow(node)
    main_window.show()
    
    # ROS2 spin'i Qt event loop ile entegre et
    from PyQt5.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(100)  # Her 100ms'de bir
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()