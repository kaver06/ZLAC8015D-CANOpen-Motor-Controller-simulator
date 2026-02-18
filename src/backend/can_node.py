import canopen
import time
import os
import sys
import threading

# --- FIX: Dynamically add the backend folder to Python's path ---
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from state_machine import CiA402StateMachine
from physics_sim import MotorPhysics

class MotorControllerSimulator:
    def __init__(self, interface='vcan0', node_id=1):
        self.network = canopen.Network()
        self.node_id = node_id
        self.state_machine = CiA402StateMachine()
        self.physics = MotorPhysics()
        
        print(f"Connecting to {interface}...")
        self.network.connect(channel=interface, bustype='socketcan')
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        eds_path = os.path.join(current_dir, 'zlac8015d.eds')
        
        print(f"Initializing ZLAC8015D Simulator at Node-ID {self.node_id}...")
        self.node = canopen.LocalNode(self.node_id, eds_path)
        self.network.add_node(self.node)
        
        # Initialize memory registers
        self.node.sdo[0x6041].phys = 0      # Status
        self.node.sdo[0x6060].phys = 3      # Mode (Default 3 = Velocity)
        
        self.node.sdo[0x606C][1].phys = 0   # Actual Vel L
        self.node.sdo[0x606C][2].phys = 0   # Actual Vel R
        self.node.sdo[0x60FF][1].phys = 0   # Target Vel L
        self.node.sdo[0x60FF][2].phys = 0   # Target Vel R
        
        self.node.sdo[0x6064][1].phys = 0   # Actual Pos L
        self.node.sdo[0x6064][2].phys = 0   # Actual Pos R
        self.node.sdo[0x607A][1].phys = 0   # Target Pos L
        self.node.sdo[0x607A][2].phys = 0   # Target Pos R
        
        self.node.sdo[0x6077][1].phys = 0   # Actual Trq L
        self.node.sdo[0x6077][2].phys = 0   # Actual Trq R
        self.node.sdo[0x6071][1].phys = 0   # Target Trq L
        self.node.sdo[0x6071][2].phys = 0   # Target Trq R
        
        self.node.sdo[0x6081][1].phys = 1000 # Default 100.0 RPM
        self.node.sdo[0x6081][2].phys = 1000
        self.node.sdo[0x6083][1].phys = 200  # Default 200 RPM/s accel
        self.node.sdo[0x6083][2].phys = 200
        self.node.sdo[0x6084][1].phys = 200  # Default 200 RPM/s decel
        self.node.sdo[0x6084][2].phys = 200
        
        self.setup_pdos()
        self.setup_callbacks()
        self.physics.start()

    def setup_pdos(self):
        self.node.tpdo.read()
        
        self.node.tpdo[1].clear()
        self.status_var = self.node.tpdo[1].add_variable(0x6041)
        self.status_var.phys = 0
        self.node.tpdo[1].trans_type = 254
        self.node.tpdo[1].event_timer = 100
        self.node.tpdo[1].enabled = True
        
        self.node.tpdo[2].clear()
        self.vel_l_var = self.node.tpdo[2].add_variable(0x606C, 1)
        self.vel_r_var = self.node.tpdo[2].add_variable(0x606C, 2)
        self.vel_l_var.phys = 0
        self.vel_r_var.phys = 0
        self.node.tpdo[2].trans_type = 254
        self.node.tpdo[2].event_timer = 100
        self.node.tpdo[2].enabled = True
        
        self.node.tpdo.save()

    def setup_callbacks(self):
        self.node.add_write_callback(self.on_sdo_write)
        self.network.subscribe(0x000, self.on_nmt_command)

    def on_nmt_command(self, can_id, data, timestamp):
        if len(data) >= 2:
            cmd = data[0]
            node = data[1]
            if node == self.node_id or node == 0x00:
                if cmd == 0x01: # Start Node (Operational)
                    self.node.nmt.state = 'OPERATIONAL'
                    self.node.tpdo[1].start(0.1)
                    self.node.tpdo[2].start(0.1)
                    
                    # Ensure physics engine wakes up if Drive was already Enabled
                    drive_is_ready = (self.state_machine.state == "OPERATION_ENABLE")
                    self.physics.set_enabled(drive_is_ready)
                    
                    print(f"[NMT] Node {self.node_id} is now OPERATIONAL.")
                    
                elif cmd in [0x02, 0x80, 0x81, 0x82]: # Stop, Pre-Op, Reset
                    self.node.nmt.state = 'STOPPED' if cmd == 0x02 else 'PRE-OPERATIONAL'
                    
                    # --- FIX: UNCONDITIONAL HARDWARE SAFETY SHUTDOWN ---
                    print(f"[NMT] Safety Triggered: Node is no longer OPERATIONAL. Cutting motor power!")
                    self.state_machine.force_shutdown() 
                    
                    # Force the CANopen memory to instantly reflect the disabled state
                    self.status_var.phys = self.state_machine.status_word 
                    
                    # Kill the physics engine
                    self.physics.set_enabled(False)
                    
                    # Stop broadcasting PDOs
                    self.node.tpdo[1].stop()
                    self.node.tpdo[2].stop()

    def on_sdo_write(self, **kwargs):
        index = kwargs.get('index')
        subindex = kwargs.get('subindex')
        data = kwargs.get('data') 
        
        # 1. State Machine (0x6040)
        if index == 0x6040: 
            control_value = int.from_bytes(data, byteorder='little')
            new_status_word = self.state_machine.process_control_word(control_value)
            self.status_var.phys = new_status_word
            
            drive_is_ready = (self.state_machine.state == "OPERATION_ENABLE")
            nmt_is_operational = (self.node.nmt.state == 'OPERATIONAL')
            
            self.physics.set_enabled(drive_is_ready and nmt_is_operational)

            if drive_is_ready and not nmt_is_operational:
                print("[WARNING] Drive Enabled but NMT is PRE-OP. Motor power is BLOCKED.")

        # 2. Mode Switch (0x6060)
        elif index == 0x6060: 
            mode = int.from_bytes(data, byteorder='little', signed=True)
            self.physics.set_mode(mode)
            mode_str = {1: "Position", 3: "Velocity", 4: "Torque"}.get(mode, "Unknown")
            print(f"\n[Mode Config] Switched to Profile {mode_str} Mode ({mode})")

        # 3. Target Velocity (0x60FF)
        elif index == 0x60FF: 
            raw_target = int.from_bytes(data, byteorder='little', signed=True)
            t_left = raw_target * 10 if subindex == 1 else self.physics.target_velocity_left
            t_right = raw_target * 10 if subindex == 2 else self.physics.target_velocity_right
            self.physics.update_velocity_target(t_left, t_right)

        # 4. Target Position (0x607A)
        elif index == 0x607A: 
            raw_target = int.from_bytes(data, byteorder='little', signed=True)
            t_left = raw_target if subindex == 1 else self.physics.target_position_left
            t_right = raw_target if subindex == 2 else self.physics.target_position_right
            self.physics.update_position_target(t_left, t_right)

        # 5. Target Torque (0x6071)
        elif index == 0x6071: 
            raw_target = int.from_bytes(data, byteorder='little', signed=True)
            t_left = raw_target if subindex == 1 else self.physics.target_torque_left
            t_right = raw_target if subindex == 2 else self.physics.target_torque_right
            self.physics.update_torque_target(t_left, t_right)
            
        # 6. Kinematic Profile Limits (0x6081, 0x6083, 0x6084)
        elif index in [0x6081, 0x6083, 0x6084]:
            p_vel_l = self.node.sdo[0x6081][1].phys
            p_vel_r = self.node.sdo[0x6081][2].phys
            p_acc_l = self.node.sdo[0x6083][1].phys
            p_acc_r = self.node.sdo[0x6083][2].phys
            p_dec_l = self.node.sdo[0x6084][1].phys
            p_dec_r = self.node.sdo[0x6084][2].phys
            self.physics.update_kinematic_profiles(p_vel_l, p_vel_r, p_acc_l, p_acc_r, p_dec_l, p_dec_r)
                   


    def start(self):
        def update_od_from_physics():
            while True:
                with self.physics.lock:
                    vl = int(self.physics.current_velocity_left)
                    vr = int(self.physics.current_velocity_right)
                    pl = int(self.physics.current_position_left)
                    pr = int(self.physics.current_position_right)
                    tl = int(self.physics.current_torque_left)
                    tr = int(self.physics.current_torque_right)
                
                # Direct updates to TPDO buffers
                self.vel_l_var.phys = vl
                self.vel_r_var.phys = vr
                
                # Update memory for Position and Torque (Master can read these via SDO)
                self.node.sdo[0x6064][1].raw = pl.to_bytes(4, byteorder='little', signed=True)
                self.node.sdo[0x6064][2].raw = pr.to_bytes(4, byteorder='little', signed=True)
                self.node.sdo[0x6077][1].raw = tl.to_bytes(2, byteorder='little', signed=True)
                self.node.sdo[0x6077][2].raw = tr.to_bytes(2, byteorder='little', signed=True)
                
                time.sleep(0.05)
                
        threading.Thread(target=update_od_from_physics, daemon=True).start()
        
        nmt_state_map = {'INITIALISING': 0x00, 'STOPPED': 0x04, 'OPERATIONAL': 0x05, 'PRE-OPERATIONAL': 0x7F}
        self.node.nmt.state = 'PRE-OPERATIONAL'
        
        try:
            while True:
                state_byte = nmt_state_map.get(self.node.nmt.state, 0x7F)
                self.network.send_message(0x700 + self.node_id, [state_byte])
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.physics.stop()
            self.network.disconnect()

if __name__ == "__main__":
    sim = MotorControllerSimulator()
    sim.start()
