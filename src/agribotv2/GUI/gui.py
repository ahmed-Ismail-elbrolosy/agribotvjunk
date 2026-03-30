import sys
import math
import csv
import random
from datetime import datetime

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QScrollArea, QGroupBox, QProgressBar, QFormLayout, 
    QSpinBox, QPushButton, QGridLayout, QTextEdit, QFileDialog, QMessageBox,
    QSizePolicy, QComboBox
)
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QPixmap, QImage

# Local Imports
import config
from database import AgriBotDB
from controller import RosController
from map_widget import MapWidget

class LegendWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.add_item(layout, "Empty", "#FFFFE0")
        self.add_item(layout, "Scanned", "#0000FF")
        self.add_item(layout, "Detected", "#00FFFF")  # CV-detected plants (cyan)
        self.add_item(layout, "Planted", "#90EE90")   # Robot-planted (light green, varies by batch)
        self.add_item(layout, "Checked", "#FFD700")
        
        layout.addStretch()
        self.setLayout(layout)

    def add_item(self, layout, text, color_code):
        lbl_color = QLabel()
        lbl_color.setFixedSize(15, 15)
        lbl_color.setStyleSheet(f"background-color: {color_code}; border: 1px solid gray;")
        layout.addWidget(lbl_color)
        layout.addWidget(QLabel(text))
        layout.addSpacing(15)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AgriBot Command Center")
        self.showMaximized()
        
        # 1. Initialize Backend
        self.db = AgriBotDB()
        self.ctrl = RosController()
        self.ctrl.db_ref = self.db  # Inject Database reference into Controller
        
        # 2. Main Layout Setup
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central) 

        # ==========================================
        # LEFT PANEL: MAP & INFO
        # ==========================================
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # Header Row
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel("<b>FIELD MAP</b>"))
        
        # Robot Position Label
        self.lbl_robot_pos = QLabel("Robot: X: 0.00m, Y: 0.00m, Yaw: 0°")
        self.lbl_robot_pos.setStyleSheet("font-weight: bold; color: blue; margin-left: 20px;")
        header_layout.addWidget(self.lbl_robot_pos)
        
        header_layout.addStretch()
        
        # Cursor Position Label
        self.lbl_coords = QLabel("Cursor: X: 0.00m, Y: 0.00m")
        header_layout.addWidget(self.lbl_coords)
        
        left_layout.addLayout(header_layout)
        left_layout.addWidget(LegendWidget())
        
        # Map Widget
        self.map_widget = MapWidget(self.db, self.ctrl)
        left_layout.addWidget(self.map_widget)
        
        main_layout.addWidget(left_widget, stretch=70)

        # ==========================================
        # RIGHT PANEL: CONTROLS
        # ==========================================
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setAlignment(Qt.AlignTop)
        
        # A. Camera Feed
        self.lbl_cam = QLabel("Camera Feed")
        self.lbl_cam.setFixedSize(320, 240)
        self.lbl_cam.setStyleSheet("background: black; border: 2px solid grey; color: white;")
        self.lbl_cam.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.lbl_cam, alignment=Qt.AlignCenter)

        # B. Seeder Status
        seed_grp = QGroupBox("Seeder Status")
        seed_layout = QVBoxLayout()
        self.lbl_seed_state = QLabel("IDLE")
        self.lbl_seed_state.setStyleSheet("font-weight: bold; color: darkcyan")
        self.bar_seed = QProgressBar()
        self.bar_seed.setRange(-15, 20)
        self.bar_seed.setFormat("%v cm")
        seed_layout.addWidget(QLabel("State:"))
        seed_layout.addWidget(self.lbl_seed_state)
        seed_layout.addWidget(self.bar_seed)
        seed_grp.setLayout(seed_layout)
        right_layout.addWidget(seed_grp)

        # C. Automated Tasks
        task_grp = QGroupBox("Automated Tasks")
        task_form = QFormLayout()
        
        self.n_points_spin = QSpinBox()
        self.n_points_spin.setValue(3)
        self.n_points_spin.setRange(1, 20)
        
        # Seed Selection Dropdown
        self.seed_combo = QComboBox()
        self.seed_combo.setStyleSheet("padding: 5px;")
        self.refresh_seed_list()
        
        btn_goto = QPushButton("Go To Point")
        btn_goto.clicked.connect(lambda: self.set_map_mode("GOTO"))
        
        btn_plant = QPushButton("Plant Region (Manual)")
        btn_plant.clicked.connect(lambda: self.set_map_mode("PLANT"))
        
        btn_scan = QPushButton("Soil Scan (Random)")
        btn_scan.clicked.connect(lambda: self.set_map_mode("SOIL_SCAN"))
        
        task_form.addRow("Samples:", self.n_points_spin)
        task_form.addRow("Seed Type:", self.seed_combo)
        task_form.addRow(btn_goto)
        task_form.addRow(btn_plant)
        task_form.addRow(btn_scan)
        task_grp.setLayout(task_form)
        right_layout.addWidget(task_grp)

        # D. Database Manager
        db_grp = QGroupBox("Database Manager")
        db_layout = QHBoxLayout()
        
        btn_export = QPushButton("Export CSV")
        btn_export.clicked.connect(self.export_db)
        
        btn_reset = QPushButton("Reset DB")
        btn_reset.setStyleSheet("color: white; background-color: #d9534f; font-weight: bold;")
        btn_reset.clicked.connect(self.reset_db)
        
        db_layout.addWidget(btn_export)
        db_layout.addWidget(btn_reset)
        db_grp.setLayout(db_layout)
        right_layout.addWidget(db_grp)

        # E. Manual Control
        man_grp = QGroupBox("Manual Control")
        man_layout = QGridLayout()
        
        # WASD Layout
        # Tuple format: (Text, Row, Col, LinVel, AngVel)
        btns = [
            ('▲', 0, 1, 2.0, 0.0), 
            ('▼', 2, 1, -2.0, 0.0), 
            ('◄', 1, 0, 0.0, 1.0), 
            ('►', 1, 2, 0.0, -1.0)
        ]
        
        for text, r, c, vx, vw in btns:
            b = QPushButton(text)
            b.setFixedSize(40, 40)
            b.pressed.connect(lambda vx=vx, vw=vw: self.ctrl.set_manual_vel(vx, vw))
            b.released.connect(lambda: self.ctrl.set_manual_vel(0, 0))
            man_layout.addWidget(b, r, c)
            
        btn_stop = QPushButton("■")
        btn_stop.setFixedSize(40, 40)
        btn_stop.setStyleSheet("color: red; font-weight: bold;")
        btn_stop.clicked.connect(lambda: self.ctrl.set_manual_vel(0, 0))
        man_layout.addWidget(btn_stop, 1, 1)
        
        # Seeder Manual
        man_layout.addWidget(QLabel("Drill"), 0, 3, 1, 1, Qt.AlignCenter)
        btn_drill_up = QPushButton("▲")
        btn_drill_dn = QPushButton("▼")
        btn_drill_up.setFixedSize(40, 40)
        btn_drill_dn.setFixedSize(40, 40)
        
        btn_drill_up.pressed.connect(lambda: self.ctrl.set_manual_seeder(1.0))
        btn_drill_dn.pressed.connect(lambda: self.ctrl.set_manual_seeder(-1.0))
        btn_drill_up.released.connect(lambda: self.ctrl.set_manual_seeder(0.0))
        btn_drill_dn.released.connect(lambda: self.ctrl.set_manual_seeder(0.0))
        
        man_layout.addWidget(btn_drill_up, 1, 3)
        man_layout.addWidget(btn_drill_dn, 2, 3)
        
        man_grp.setLayout(man_layout)
        right_layout.addWidget(man_grp)

        # F. Logs & E-Stop
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(100)
        right_layout.addWidget(self.log_view)
        
        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; height: 40px; font-size: 14px;")
        btn_estop.clicked.connect(self.e_stop)
        right_layout.addWidget(btn_estop)
        
        right_scroll.setWidget(right_widget)
        main_layout.addWidget(right_scroll, stretch=30)

        # ==========================================
        # SIGNAL CONNECTIONS
        # ==========================================
        
        # Controller -> UI
        self.ctrl.update_odom.connect(self.map_widget.update_robot)
        self.ctrl.update_odom.connect(self.update_robot_label)
        self.ctrl.update_cam.connect(self.update_camera_feed)
        self.ctrl.update_path_viz.connect(self.map_widget.update_path)
        self.ctrl.update_sensors.connect(self.map_widget.update_sensors)
        self.ctrl.update_seeder.connect(self.update_seeder_ui)
        self.ctrl.log_msg.connect(self.log)
        
        # Plant detection -> refresh map
        self.ctrl.plant_detected.connect(self.map_widget.update)
        
        # Controller -> Map Logic
        self.ctrl.point_reached.connect(self.map_widget.start_blink)
        self.ctrl.point_reached.connect(self.on_point_reached)
        
        # NEW: Popup Connection
        self.ctrl.show_popup.connect(self.show_popup_message)
        
        # Map -> UI
        self.map_widget.mouse_moved_signal.connect(self.update_cursor_label)
        
        # Start Controller Thread
        self.ctrl.start()

    # ==========================================
    # LOGIC & SLOTS
    # ==========================================

    def set_map_mode(self, mode):
        self.map_widget.mode = mode
        self.map_widget.setCursor(Qt.CrossCursor)
        self.log(f"Mode set to: {mode} - Please select area on map")

    def refresh_seed_list(self):
        """Populate the seed dropdown from database"""
        self.seed_combo.clear()
        seeds = self.db.get_all_seeds()
        for seed in seeds:
            # seed format: (id, seed_name, planting_depth_cm, spacing_cm, germination_days, description)
            seed_name = seed[1]
            depth = seed[2]
            self.seed_combo.addItem(f"{seed_name} ({depth}cm)", seed_name)

    def get_selected_seed(self):
        """Returns the currently selected seed name"""
        return self.seed_combo.currentData()

    def get_selected_seed_depth(self):
        """Returns the planting depth for the selected seed"""
        seed_name = self.get_selected_seed()
        if seed_name:
            return self.db.get_seed_depth(seed_name)
        return None

    def on_point_reached(self, x, y, task_type):
        """
        Called when the robot reaches a specific goal point.
        """
        # Database Update Logic
        if task_type == 'soil_scan':
            # Identify grid cell using config
            c = int((x - config.GRID_CONFIG['min_x']) / config.CELL_W)
            r = int((y - config.GRID_CONFIG['min_y']) / config.CELL_H)
            
            if 0 <= c < config.GRID_CONFIG['cols'] and 0 <= r < config.GRID_CONFIG['rows']:
                self.db.update_cell(c, r, 'soil_analyzed')
                self.log(f"Soil Sampled at Grid[{c},{r}]")

    def show_popup_message(self, title, message):
        QMessageBox.information(self, title, message)

    def update_robot_label(self, x, y, yaw):
        self.lbl_robot_pos.setText(f"Robot: X: {x:.2f}m, Y: {y:.2f}m, Yaw: {math.degrees(yaw):.1f}°")

    def update_cursor_label(self, x, y):
        self.lbl_coords.setText(f"Cursor: X: {x:.2f}m, Y: {y:.2f}m")

    def update_camera_feed(self, img):
        self.lbl_cam.setPixmap(QPixmap.fromImage(img).scaled(320, 240, Qt.KeepAspectRatio))

    def update_seeder_ui(self, status, height):
        self.lbl_seed_state.setText(status)
        self.bar_seed.setValue(int(height * 100))

    def log(self, msg):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_view.append(f"[{timestamp}] {msg}")
        # Scroll to bottom
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def e_stop(self):
        self.ctrl.set_path([]) # Clear path
        self.ctrl.state = "IDLE"
        # Manually publish 0 velocity using the controller's publisher
        # Note: We need to import Twist here or rely on controller to have a stop method
        self.ctrl.stop_robot() 
        self.log("!!! EMERGENCY STOP TRIGGERED !!!")

    def export_db(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Database", "", "CSV Files (*.csv)")
        if path:
            try:
                rows = self.db.get_all_cells()
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Grid X", "Grid Y", "State", "Batch ID", "Timestamp", "Data", "Seed Type"])
                    writer.writerows(rows)
                self.log(f"Database successfully exported to {path}")
            except Exception as e:
                self.log(f"Export Failed: {str(e)}")

    def reset_db(self):
        reply = QMessageBox.question(
            self, 'Reset Database', 
            "Are you sure you want to delete ALL farm data?\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                self.db.clear_cells()
                self.map_widget.update()
                self.log("Database and Map visuals cleared.")
            except Exception as e:
                self.log(f"Reset Failed: {str(e)}")