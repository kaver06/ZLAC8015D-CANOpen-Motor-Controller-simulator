import sys
import threading
import os

os.environ["QT_QPA_PLATFORM"] = "wayland"

print("[Step 1] Loading PyQt5 library...")
try:
    from PyQt5.QtWidgets import QApplication
    print("         -> PyQt5 loaded successfully!")
except Exception as e:
    print(f"         -> PyQt5 ERROR: {e}")

print("[Step 2] Loading Simulator modules...")
from backend.can_node import MotorControllerSimulator
from frontend.app_window import SimulatorDashboard

def run_backend(simulator):
    print("[Backend Thread] Simulator loop starting now...")
    simulator.start()

def main():
    print("\n[Step 3] Initializing PyQt Application Engine...")
    app = QApplication(sys.argv)
    
    print("[Step 4] Connecting to CAN bus (vcan0)...")
    backend_sim = MotorControllerSimulator(interface='vcan0', node_id=1)
    
    print("[Step 5] Launching CAN backend in background thread...")
    sim_thread = threading.Thread(target=run_backend, args=(backend_sim,), daemon=True)
    sim_thread.start()
    
    print("[Step 6] Building Graphical Dashboard...")
    dashboard = SimulatorDashboard(backend_simulator=backend_sim)
    
    print("[Step 7] Ordering Window to SHOW on screen...")
    dashboard.show()
    
    print("[Step 8] Handing control to GUI Event Loop. Waiting for window to close...")
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
