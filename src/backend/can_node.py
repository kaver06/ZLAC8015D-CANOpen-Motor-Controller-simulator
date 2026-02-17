import canopen
import time
import os
import threading
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
        
        self.setup_pdos()     # Setup PDOs FIRST so we can get the direct buffer variables
        self.setup_callbacks()
        self.physics.start()

    def setup_pdos(self):
        self.node.tpdo.read()
        
        # --- FIX: Capture the direct TPDO buffer references ---
        # TPDO1 (Status Word)
        self.node.tpdo[1].clear()
        self.status_var = self.node.tpdo[1].add_variable(0x6041)
        self.status_var.phys = 0 # Initialize to 0
        self.node.tpdo[1].trans_type = 254
        self.node.tpdo[1].event_timer = 100
        self.node.tpdo[1].enabled = True
        
        # TPDO2 (Actual Velocities)
        self.node.tpdo[2].clear()
        self.vel_l_var = self.node.tpdo[2].add_variable(0x606C, 1)
        self.vel_r_var = self.node.tpdo[2].add_variable(0x606C, 2)
        self.vel_l_var.phys = 0 # Initialize left to 0
        self.vel_r_var.phys = 0 # Initialize right to 0
        self.node.tpdo[2].trans_type = 254
        self.node.tpdo[2].event_timer = 100
        self.node.tpdo[2].enabled = True
        
        self.node.tpdo.save()

    def setup_callbacks(self):
        self.node.add_write_callback(self.on_sdo_write)
        self.network.subscribe(0x000, self.on_nmt_command)

    def on_nmt_command(self, can_id, data, timestamp):
        if len(data) >= 2:
            command_specifier = data[0]
            target_node = data[1]
            if target_node == self.node_id or target_node == 0x00:
                if command_specifier == 0x01:
                    self.node.nmt.state = 'OPERATIONAL'
                    print("\n[CiA301 NMT] Switching to OPERATIONAL -> TPDOs will now broadcast!")
                    self.node.tpdo[1].start(0.1)
                    self.node.tpdo[2].start(0.1)
                elif command_specifier in [0x02, 0x80]:
                    self.node.nmt.state = 'STOPPED' if command_specifier == 0x02 else 'PRE-OPERATIONAL'
                    print(f"\n[CiA301 NMT] Switching to {self.node.nmt.state} -> TPDOs paused.")
                    self.node.tpdo[1].stop()
                    self.node.tpdo[2].stop()

    def on_sdo_write(self, **kwargs):
        index = kwargs.get('index')
        subindex = kwargs.get('subindex')
        data = kwargs.get('data') 
        
        if index == 0x6040:
            control_value = int.from_bytes(data, byteorder='little')
            new_status_word = self.state_machine.process_control_word(control_value)
            
            # Write directly to the TPDO buffer!
            self.status_var.phys = new_status_word
            
            print(f"\n[CiA402] Master sent Control Word: {hex(control_value)}")
            print(f"[CiA402] State transitioned to: {self.state_machine.state}")
            
            is_enabled = (self.state_machine.state == "OPERATION_ENABLE")
            self.physics.update_parameters(
                self.physics.target_velocity_left, 
                self.physics.target_velocity_right, 
                500, 500, is_enabled
            )

        elif index == 0x60FF:
            raw_target = int.from_bytes(data, byteorder='little', signed=True)
            side = "Left" if subindex == 1 else "Right"
            print(f"\n[Physics] Target Velocity Updated -> {side} Motor: {raw_target} RPM")

            t_left = raw_target * 10 if subindex == 1 else self.physics.target_velocity_left
            t_right = raw_target * 10 if subindex == 2 else self.physics.target_velocity_right
            
            self.physics.update_parameters(t_left, t_right, 500, 500, self.physics.is_enabled)

    def start(self):
        print(f"Simulator Node {self.node_id} is running. Heartbeat active.")
        print("Press Ctrl+C to stop.")
        
        def update_od_from_physics():
            while True:
                with self.physics.lock:
                    vl = int(self.physics.current_velocity_left)
                    vr = int(self.physics.current_velocity_right)
                
                # Write directly to the TPDO buffer!
                self.vel_l_var.phys = vl
                self.vel_r_var.phys = vr
                
                if self.state_machine.state == "OPERATION_ENABLE" and (vl != 0 or vr != 0):
                    print(f"   -> [Physics Engine] Speed: L={vl/10.0} RPM | R={vr/10.0} RPM   ", end='\r')
                
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
            print("\nShutting down simulator...")
        finally:
            self.physics.stop()
            self.network.disconnect()

if __name__ == "__main__":
    simulator = MotorControllerSimulator()
    simulator.start()
