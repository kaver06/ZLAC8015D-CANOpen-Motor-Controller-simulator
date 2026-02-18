import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGroupBox, QGridLayout
from PyQt5.QtCore import QTimer, Qt

class SimulatorDashboard(QMainWindow):
    def __init__(self, backend_simulator):
        super().__init__()
        self.sim = backend_simulator 
        
        self.setWindowTitle("ZLAC8015D Motor Controller Simulator")
        self.setGeometry(100, 100, 800, 350)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)
        
        # --- Top Header ---
        header = QLabel(f"ZLAC8015D Dashboard - Node ID: {self.sim.node_id}")
        header.setStyleSheet("font-size: 22px; font-weight: bold; background-color: #2c3e50; color: white; padding: 10px;")
        header.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(header)

        # --- Status Panel ---
        self.status_label = QLabel("NMT State: BOOTING | Drive State: BOOTING")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #d35400; padding: 10px;")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.status_label)

        # --- Telemetry Layout ---
        telemetry_layout = QHBoxLayout()
        
        # Left Motor Box
        left_group = QGroupBox("Left Motor (Sub-Index 1)")
        left_group.setStyleSheet("font-size: 16px; font-weight: bold; margin-top: 10px;")
        left_layout = QGridLayout()
        self.l_vel_label = QLabel("Velocity: 0.0 RPM")
        self.l_pos_label = QLabel("Position: 0 counts")
        self.l_trq_label = QLabel("Torque: 0.0 mA")
        left_layout.addWidget(self.l_vel_label, 0, 0)
        left_layout.addWidget(self.l_pos_label, 1, 0)
        left_layout.addWidget(self.l_trq_label, 2, 0)
        left_group.setLayout(left_layout)
        
        # Right Motor Box
        right_group = QGroupBox("Right Motor (Sub-Index 2)")
        right_group.setStyleSheet("font-size: 16px; font-weight: bold; margin-top: 10px;")
        right_layout = QGridLayout()
        self.r_vel_label = QLabel("Velocity: 0.0 RPM")
        self.r_pos_label = QLabel("Position: 0 counts")
        self.r_trq_label = QLabel("Torque: 0.0 mA")
        right_layout.addWidget(self.r_vel_label, 0, 0)
        right_layout.addWidget(self.r_pos_label, 1, 0)
        right_layout.addWidget(self.r_trq_label, 2, 0)
        right_group.setLayout(right_layout)

        telemetry_layout.addWidget(left_group)
        telemetry_layout.addWidget(right_group)
        
        self.main_layout.addLayout(telemetry_layout)
        self.main_layout.addStretch()
        
        # --- Setup UI Update Timer (Refresh screen at 20Hz) ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50) # 50 milliseconds

    def update_ui(self):
        """This function runs 20 times a second to pull live data from the backend"""
        
        # 1. Read States
        nmt_state = self.sim.node.nmt.state
        drive_state = self.sim.state_machine.state
        mode_num = self.sim.physics.operating_mode
        mode_str = {1: "Position", 3: "Velocity", 4: "Torque"}.get(mode_num, "Unknown")
        
        self.status_label.setText(f"NMT: {nmt_state}   |   Drive: {drive_state}   |   Mode: {mode_str} ({mode_num})")

        # 2. Read from physics engine safely
        with self.sim.physics.lock:
            vl = self.sim.physics.current_velocity_left / 10.0
            vr = self.sim.physics.current_velocity_right / 10.0
            pl = int(self.sim.physics.current_position_left)
            pr = int(self.sim.physics.current_position_right)
            tl = self.sim.physics.current_torque_left
            tr = self.sim.physics.current_torque_right
            
        # 3. Update Text Labels
        self.l_vel_label.setText(f"Velocity:  {vl:.1f} RPM")
        self.r_vel_label.setText(f"Velocity:  {vr:.1f} RPM")
        
        self.l_pos_label.setText(f"Position:  {pl} counts")
        self.r_pos_label.setText(f"Position:  {pr} counts")
        
        self.l_trq_label.setText(f"Torque:   {tl:.1f} mA")
        self.r_trq_label.setText(f"Torque:   {tr:.1f} mA")
