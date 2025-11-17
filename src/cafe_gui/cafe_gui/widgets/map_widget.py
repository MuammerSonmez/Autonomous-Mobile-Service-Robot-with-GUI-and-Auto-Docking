#!/usr/bin/env python3
"""
Harita görselleştirme widget'ı
"""
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPixmap, QPolygonF
import math

class MapWidget(QWidget):
    """Harita ve robot konumu görselleştirme"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Waypoint'ler (örnek lokasyonlar)
        self.waypoints = {
            'Mutfak': (50, 50),
            'Masa 1': (200, 100),
            'Masa 2': (200, 200),
            'Masa 3': (200, 300),
            'Masa 4': (350, 100),
            'Masa 5': (350, 200),
            'Şarj İstasyonu': (50, 300)
        }
        
        self.current_goal = None
        self.path = []
        
        self.setMinimumSize(400, 400)
        self.setStyleSheet("background-color: white; border: 2px solid #ccc;")
        
    def set_robot_pose(self, x, y, theta):
        """Robot pozisyonunu güncelle"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        self.update()
        
    def set_goal(self, goal_name):
        """Hedef nokta belirle"""
        if goal_name in self.waypoints:
            self.current_goal = goal_name
            self.update()
            
    def paintEvent(self, event):
        """Haritayı çiz"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Arka plan
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        
        # Grid çiz
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        for i in range(0, self.width(), 50):
            painter.drawLine(i, 0, i, self.height())
        for i in range(0, self.height(), 50):
            painter.drawLine(0, i, self.width(), i)
            
        # Waypoint'leri çiz
        for name, (x, y) in self.waypoints.items():
            # Nokta
            if name == 'Mutfak':
                color = QColor(255, 152, 0)  # Turuncu
            elif name == 'Şarj İstasyonu':
                color = QColor(76, 175, 80)  # Yeşil
            else:
                color = QColor(33, 150, 243)  # Mavi
                
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(Qt.black, 2))
            painter.drawEllipse(QPointF(x, y), 15, 15)
            
            # İsim
            painter.setPen(QPen(Qt.black))
            painter.drawText(x - 30, y + 30, name)
            
        # Hedef varsa çiz
        if self.current_goal and self.current_goal in self.waypoints:
            gx, gy = self.waypoints[self.current_goal]
            painter.setPen(QPen(QColor(244, 67, 54), 3, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(gx, gy), 25, 25)
            
        # Robotu çiz
        robot_screen_x = 100 + self.robot_x * 50  # Ölçeklendirme
        robot_screen_y = 200 - self.robot_y * 50
        
        painter.setBrush(QBrush(QColor(156, 39, 176)))  # Mor
        painter.setPen(QPen(Qt.black, 2))
        
        # Robot gövdesi (daire)
        painter.drawEllipse(QPointF(robot_screen_x, robot_screen_y), 20, 20)
        
        # Robot yönü (ok)
        arrow_length = 25
        arrow_x = robot_screen_x + arrow_length * math.cos(self.robot_theta)
        arrow_y = robot_screen_y - arrow_length * math.sin(self.robot_theta)
        
        painter.setPen(QPen(QColor(255, 235, 59), 3))  # Sarı ok
        painter.drawLine(QPointF(robot_screen_x, robot_screen_y),
                        QPointF(arrow_x, arrow_y))
        
        # Konum yazısı
        painter.setPen(QPen(Qt.black))
        painter.drawText(10, 20, f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f})')