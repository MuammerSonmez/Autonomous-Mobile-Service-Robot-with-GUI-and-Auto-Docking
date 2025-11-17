#!/usr/bin/env python3
"""
Ana GUI Penceresi - Kafe Robot Kontrol Paneli
"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QGroupBox, QListWidget, 
                             QLineEdit, QComboBox, QMessageBox, QStatusBar,
                             QTextEdit, QSplitter, QFrame)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

class MainWindow(QMainWindow):
    """Ana GUI Penceresi"""
    
    # Sinyaller
    status_update = pyqtSignal(str, str)  # (durum, renk)
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # ROS2 Publishers ve Subscribers
        self.setup_ros_interfaces()
        
        # GUI'yi başlat
        self.init_ui()
        
        # Robot durumu
        self.robot_status = "IDLE"
        self.current_order = None
        self.battery_level = 100
        
        # Zamanlayıcılar
        self.setup_timers()
        
    def setup_ros_interfaces(self):
        """ROS2 publisher ve subscriber'ları ayarla"""
        # Publishers
        self.goal_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.cmd_pub = self.node.create_publisher(
            String, '/robot_command', 10)
        self.vel_pub = self.node.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.status_sub = self.node.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.node.get_logger().info('ROS2 arayüzleri kuruldu')
        
    def init_ui(self):
        """Ana GUI arayüzünü oluştur"""
        self.setWindowTitle('Baristar Cafe Robot Kontrol Paneli')
        self.setGeometry(100, 100, 1400, 900)
        self.setMinimumSize(1000, 700)
        
        # Merkezi widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Sol panel - Sipariş ve Kontrol
        left_panel = self.create_left_panel()
        
        # Orta panel - Harita ve Görselleştirme
        center_panel = self.create_center_panel()
        
        # Sağ panel - Durum ve Loglar
        right_panel = self.create_right_panel()
        
        # Splitter ile panelleri ayarla
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        splitter.setStretchFactor(2, 1)
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('Sistem Hazır ✓')
        
        # Stil uygula
        self.apply_styles()
        
    def create_left_panel(self):
        """Sol panel - Sipariş yönetimi"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        layout = QVBoxLayout(panel)
        
        # Başlık
        title = QLabel('📋 Sipariş Yönetimi')
        title_font = QFont('Arial', 14, QFont.Bold)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Yeni sipariş grubu
        order_group = QGroupBox('Yeni Sipariş Oluştur')
        order_layout = QVBoxLayout()
        
        # Masa seçimi
        layout_masa = QHBoxLayout()
        layout_masa.addWidget(QLabel('Masa:'))
        self.table_combo = QComboBox()
        self.table_combo.addItems([f'Masa {i}' for i in range(1, 7)])
        layout_masa.addWidget(self.table_combo)
        order_layout.addLayout(layout_masa)
        
        # Ürün seçimi
        layout_urun = QHBoxLayout()
        layout_urun.addWidget(QLabel('Ürün:'))
        self.product_combo = QComboBox()
        self.product_combo.addItems([
            'Türk Kahvesi',
            'Espresso',
            'Cappuccino',
            'Latte',
            'Çay ',
            'Su ',
            'Tatlı'
        ])
        layout_urun.addWidget(self.product_combo)
        order_layout.addLayout(layout_urun)
        
        # Notlar
        layout_not = QVBoxLayout()
        layout_not.addWidget(QLabel('Notlar:'))
        self.notes_input = QLineEdit()
        self.notes_input.setPlaceholderText('Ek notlar (opsiyonel)...')
        layout_not.addWidget(self.notes_input)
        order_layout.addLayout(layout_not)
        
        # Sipariş oluştur butonu
        self.create_order_btn = QPushButton('✓ Sipariş Oluştur')
        self.create_order_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 12pt;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.create_order_btn.clicked.connect(self.create_order)
        order_layout.addWidget(self.create_order_btn)
        
        order_group.setLayout(order_layout)
        layout.addWidget(order_group)
        
        # Aktif siparişler
        active_group = QGroupBox('Aktif Siparişler')
        active_layout = QVBoxLayout()
        
        self.orders_list = QListWidget()
        active_layout.addWidget(self.orders_list)
        
        # İptal butonu
        self.cancel_order_btn = QPushButton('✗ Seçili Siparişi İptal Et')
        self.cancel_order_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                padding: 8px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)
        self.cancel_order_btn.clicked.connect(self.cancel_order)
        active_layout.addWidget(self.cancel_order_btn)
        
        active_group.setLayout(active_layout)
        layout.addWidget(active_group)
        
        layout.addStretch()
        return panel
        
    def create_center_panel(self):
        """Orta panel - Harita ve robot görselleştirme"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        layout = QVBoxLayout(panel)
        
        # Başlık
        title = QLabel('🗺️ Robot Haritası ve Konum')
        title_font = QFont('Arial', 14, QFont.Bold)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Harita gösterim alanı
        self.map_label = QLabel('Harita yükleniyor...')
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                border: 2px solid #ccc;
                border-radius: 10px;
                min-height: 400px;
            }
        """)
        layout.addWidget(self.map_label, 1)
        
        # Konum bilgisi
        position_layout = QHBoxLayout()
        self.pos_label = QLabel('Pozisyon: X: 0.00, Y: 0.00, Θ: 0.00°')
        self.pos_label.setStyleSheet('padding: 5px; background-color: #e8f5e9;')
        position_layout.addWidget(self.pos_label)
        layout.addLayout(position_layout)
        
        # Manuel kontrol
        control_group = QGroupBox('🎮 Manuel Kontrol')
        control_layout = QVBoxLayout()
        
        # Yön tuşları
        btn_layout = QVBoxLayout()
        
        # İleri
        up_layout = QHBoxLayout()
        up_layout.addStretch()
        self.forward_btn = QPushButton('↑')
        self.forward_btn.setFixedSize(60, 60)
        self.forward_btn.pressed.connect(self.move_forward)
        self.forward_btn.released.connect(self.stop_robot)
        up_layout.addWidget(self.forward_btn)
        up_layout.addStretch()
        btn_layout.addLayout(up_layout)
        
        # Sol-Geri-Sağ
        mid_layout = QHBoxLayout()
        mid_layout.addStretch()
        self.left_btn = QPushButton('←')
        self.left_btn.setFixedSize(60, 60)
        self.left_btn.pressed.connect(self.turn_left)
        self.left_btn.released.connect(self.stop_robot)
        mid_layout.addWidget(self.left_btn)
        
        self.backward_btn = QPushButton('↓')
        self.backward_btn.setFixedSize(60, 60)
        self.backward_btn.pressed.connect(self.move_backward)
        self.backward_btn.released.connect(self.stop_robot)
        mid_layout.addWidget(self.backward_btn)
        
        self.right_btn = QPushButton('→')
        self.right_btn.setFixedSize(60, 60)
        self.right_btn.pressed.connect(self.turn_right)
        self.right_btn.released.connect(self.stop_robot)
        mid_layout.addWidget(self.right_btn)
        mid_layout.addStretch()
        btn_layout.addLayout(mid_layout)
        
        control_layout.addLayout(btn_layout)
        
        # Acil dur butonu
        self.emergency_stop_btn = QPushButton('🛑 ACİL DURDUR')
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff0000;
                color: white;
                font-size: 14pt;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #cc0000;
            }
        """)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        control_layout.addWidget(self.emergency_stop_btn)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        return panel
        
    def create_right_panel(self):
        """Sağ panel - Durum ve loglar"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        layout = QVBoxLayout(panel)
        
        # Başlık
        title = QLabel('📊 Robot Durumu')
        title_font = QFont('Arial', 14, QFont.Bold)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Durum göstergesi
        status_group = QGroupBox('Anlık Durum')
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel('HAZIR')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #4CAF50;
                color: white;
                font-size: 16pt;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
            }
        """)
        status_layout.addWidget(self.status_label)
        
        # Batarya
        self.battery_label = QLabel('🔋 Batarya: 100%')
        self.battery_label.setStyleSheet('padding: 10px; font-size: 12pt;')
        status_layout.addWidget(self.battery_label)
        
        # Hız
        self.speed_label = QLabel('⚡ Hız: 0.00 m/s')
        self.speed_label.setStyleSheet('padding: 10px; font-size: 12pt;')
        status_layout.addWidget(self.speed_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Hızlı eylemler
        quick_group = QGroupBox('Hızlı Eylemler')
        quick_layout = QVBoxLayout()
        
        self.home_btn = QPushButton('🏠 Eve Dön')
        self.home_btn.clicked.connect(self.go_home)
        quick_layout.addWidget(self.home_btn)
        
        self.kitchen_btn = QPushButton('🍳 Mutfağa Git')
        self.kitchen_btn.clicked.connect(self.go_to_kitchen)
        quick_layout.addWidget(self.kitchen_btn)
        
        self.pause_btn = QPushButton('⏸️ Duraklat')
        self.pause_btn.clicked.connect(self.pause_robot)
        quick_layout.addWidget(self.pause_btn)
        
        self.resume_btn = QPushButton('▶️ Devam Et')
        self.resume_btn.clicked.connect(self.resume_robot)
        quick_layout.addWidget(self.resume_btn)
        
        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)
        
        # Log alanı
        log_group = QGroupBox('📝 Sistem Logları')
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)
        
        # Log temizle
        clear_log_btn = QPushButton('Logları Temizle')
        clear_log_btn.clicked.connect(self.log_text.clear)
        log_layout.addWidget(clear_log_btn)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        layout.addStretch()
        return panel
        
    def setup_timers(self):
        """Zamanlayıcıları ayarla"""
        # UI güncelleme zamanlayıcısı
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(500)  # Her 500ms
        
    def apply_styles(self):
        """Global stiller"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #fafafa;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                padding: 8px;
                border-radius: 4px;
                background-color: #2196F3;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
            QPushButton:pressed {
                background-color: #0960a8;
            }
        """)
        
    # ===== ROS Callback Fonksiyonları =====
    
    def status_callback(self, msg):
        """Robot durum mesajlarını işle"""
        self.robot_status = msg.data
        self.add_log(f'Durum güncellendi: {self.robot_status}')
        
    def odom_callback(self, msg):
        """Odometri mesajlarını işle"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Yaw hesapla (quaternion'dan)
        # Basitleştirilmiş, gerçekte quaternion_to_euler kullan
        self.pos_label.setText(f'Pozisyon: X: {x:.2f}, Y: {y:.2f}')
        
    # ===== Sipariş Yönetimi =====
    
    def create_order(self):
        """Yeni sipariş oluştur"""
        table = self.table_combo.currentText()
        product = self.product_combo.currentText()
        notes = self.notes_input.text()
        
        order_text = f'{table} - {product}'
        if notes:
            order_text += f' ({notes})'
            
        self.orders_list.addItem(order_text)
        self.add_log(f'✓ Yeni sipariş: {order_text}')
        self.notes_input.clear()
        
        # ROS mesajı gönder
        msg = String()
        msg.data = f'NEW_ORDER:{table}:{product}:{notes}'
        self.cmd_pub.publish(msg)
        
        QMessageBox.information(self, 'Sipariş Oluşturuldu', 
                               f'{order_text} başarıyla oluşturuldu!')
        
    def cancel_order(self):
        """Seçili siparişi iptal et"""
        current_item = self.orders_list.currentItem()
        if current_item:
            order = current_item.text()
            self.orders_list.takeItem(self.orders_list.row(current_item))
            self.add_log(f'✗ Sipariş iptal edildi: {order}')
            
            msg = String()
            msg.data = f'CANCEL_ORDER:{order}'
            self.cmd_pub.publish(msg)
        else:
            QMessageBox.warning(self, 'Uyarı', 'Lütfen iptal edilecek siparişi seçin!')
            
    # ===== Robot Kontrol =====
    
    def move_forward(self):
        """İleri git"""
        twist = Twist()
        twist.linear.x = 0.3
        self.vel_pub.publish(twist)
        self.add_log('İleri hareket')
        
    def move_backward(self):
        """Geri git"""
        twist = Twist()
        twist.linear.x = -0.3
        self.vel_pub.publish(twist)
        self.add_log('Geri hareket')
        
    def turn_left(self):
        """Sola dön"""
        twist = Twist()
        twist.angular.z = 0.5
        self.vel_pub.publish(twist)
        self.add_log('Sola dönüş')
        
    def turn_right(self):
        """Sağa dön"""
        twist = Twist()
        twist.angular.z = -0.5
        self.vel_pub.publish(twist)
        self.add_log('Sağa dönüş')
        
    def stop_robot(self):
        """Robotu durdur"""
        twist = Twist()
        self.vel_pub.publish(twist)
        
    def emergency_stop(self):
        """Acil durma"""
        self.stop_robot()
        msg = String()
        msg.data = 'EMERGENCY_STOP'
        self.cmd_pub.publish(msg)
        self.add_log('🛑 ACİL DURDURMA!')
        self.status_label.setText('ACİL DURDURMA')
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #f44336;
                color: white;
                font-size: 16pt;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
            }
        """)
        
    def go_home(self):
        """Eve dön"""
        msg = String()
        msg.data = 'GO_HOME'
        self.cmd_pub.publish(msg)
        self.add_log('🏠 Eve dönüş komutu gönderildi')
        
    def go_to_kitchen(self):
        """Mutfağa git"""
        msg = String()
        msg.data = 'GO_KITCHEN'
        self.cmd_pub.publish(msg)
        self.add_log('🍳 Mutfağa git komutu gönderildi')
        
    def pause_robot(self):
        """Robotu duraklat"""
        msg = String()
        msg.data = 'PAUSE'
        self.cmd_pub.publish(msg)
        self.add_log('⏸️ Robot duraklatıldı')
        
    def resume_robot(self):
        """Devam et"""
        msg = String()
        msg.data = 'RESUME'
        self.cmd_pub.publish(msg)
        self.add_log('▶️ Robot devam ediyor')
        
    # ===== UI Güncellemeleri =====
    
    def update_ui(self):
        """UI'yi güncelle"""
        # Durum rengini güncelle
        if self.robot_status == 'IDLE':
            color = '#4CAF50'  # Yeşil
            text = 'HAZIR'
        elif self.robot_status == 'MOVING':
            color = '#2196F3'  # Mavi
            text = 'HAREKET EDİYOR'
        elif self.robot_status == 'SERVING':
            color = '#FF9800'  # Turuncu
            text = 'SERVİS YAPIYOR'
        elif self.robot_status == 'CHARGING':
            color = '#9C27B0'  # Mor
            text = 'ŞARJ OLUYOR'
        elif self.robot_status == 'ERROR':
            color = '#f44336'  # Kırmızı
            text = 'HATA!'
        else:
            color = '#607D8B'  # Gri
            text = self.robot_status
            
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                color: white;
                font-size: 16pt;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
            }}
        """)
        
        # Batarya seviyesini güncelle
        self.battery_level = max(0, self.battery_level - 0.01)  # Simülasyon
        battery_color = '#4CAF50' if self.battery_level > 50 else '#FF9800' if self.battery_level > 20 else '#f44336'
        self.battery_label.setText(f'🔋 Batarya: {self.battery_level:.1f}%')
        self.battery_label.setStyleSheet(f'padding: 10px; font-size: 12pt; color: {battery_color}; font-weight: bold;')
        
    def add_log(self, message):
        """Log ekle"""
        from datetime import datetime
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f'[{timestamp}] {message}')
        self.node.get_logger().info(message)