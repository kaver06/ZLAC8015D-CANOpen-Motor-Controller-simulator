import canopen
import os
import threading
import time
from .state_machine import CiA402StateMachine
from .driver_logic import MotorDriverLogic

class MotorControllerSimulator:
    def __init__(self, interface='vcan0', node_id=1):
        self.network = canopen.Network()
        self.network.connect(bustype='socketcan', channel=interface)
        self.node_id = node_id

        # Load EDS as a LOCAL SERVER (The Virtual Motor)
        eds_path = os.path.join(os.path.dirname(__file__), 'zlac8015d.eds')
        self.node = self.network.create_node(self.node_id, eds_path)

        # Attach State Machine and the Cascaded Driver Logic
        self.state_machine = CiA402StateMachine()
        self.physics = MotorDriverLogic()

        # Link SDOs
        self.status_var = self.node.sdo[0x6041]
        self.status_var.phys = self.state_machine.status_word

        # Setup Kinematic Defaults to ZLTECH Factory Specs
        self.node.sdo[0x6081][1].phys = 120  # Cruising Speed: 120 RPM
        self.node.sdo[0x6081][2].phys = 120
        
        self.node.sdo[0x6083][1].phys = 500  # Acceleration Time: 500 ms
        self.node.sdo[0x6083][2].phys = 500
        
        self.node.sdo[0x6084][1].phys = 500  # Deceleration Time: 500 ms
        self.node.sdo[0x6084][2].phys = 500

        # --- THE FIX: The official python-canopen local callback! ---
        # This one line catches EVERY SDO write sent by the master
        self.node.add_write_callback(self.on_sdo_write)
        # ------------------------------------------------------------

        # TPDO Mapping
        self.node.tpdo.read()
        self.node.tpdo[1].clear()
        self.node.tpdo[1].add_variable(0x6041) # Status Word
        self.node.tpdo[1].trans_type = 254
        self.node.tpdo[1].event_timer = 100
        self.node.tpdo[1].enabled = True

        self.node.tpdo[2].clear()
        self.node.tpdo[2].add_variable(0x606C, 1) # Left Vel
        self.node.tpdo[2].add_variable(0x606C, 2) # Right Vel
        self.node.tpdo[2].trans_type = 254
        self.node.tpdo[2].event_timer = 100
        self.node.tpdo[2].enabled = True

        self.node.tpdo.save()

    def on_sdo_write(self, **kwargs):
        """Heavily instrumented callback to catch silent background crashes"""
        try:
            index = kwargs.get('index')
            subindex = kwargs.get('subindex')
            data = kwargs.get('data') 
            
            print(f"\n[SDO RECV] Index: 0x{index:04X}, Sub: {subindex}, Raw Data: {data.hex() if data else 'None'}")
            
            if index == 0x6040: 
                control_value = int.from_bytes(data, byteorder='little')
                print(f"  -> State Machine Processing Control Word: 0x{control_value:04X}")
                
                cw_fault_reset = (control_value >> 7) & 1
                if self.state_machine.state == 'FAULT' and cw_fault_reset:
                    print("  -> [FAULT RESET] Master cleared the fault. Restarting Drive.")
                    # --- ZLTECH FIX: Clear the proprietary error code memory ---
                    self.physics.zlac_error_code = 0x00000000
                    if 0x603F in self.node.object_dictionary:
                        self.node.object_dictionary[0x603F].raw = (0).to_bytes(4, 'little')

                old_state = self.state_machine.state
                new_status_word = self.state_machine.process_control_word(control_value)
                print(f"  -> Transition: {old_state} -> {self.state_machine.state} (New Status: 0x{new_status_word:04X})")
                
                # Safely write to internal memory without looping back to the CAN bus
                self.node.object_dictionary[0x6041].raw = new_status_word.to_bytes(2, 'little')
                
                drive_is_ready = (self.state_machine.state == "OPERATION_ENABLE")
                nmt_is_operational = (self.node.nmt.state == 'OPERATIONAL')
                self.physics.set_enabled(drive_is_ready and nmt_is_operational)

            elif index == 0x6060: 
                mode = int.from_bytes(data, byteorder='little', signed=True)
                self.physics.set_mode(mode)
                print(f"  -> Switched to Mode: {mode}")

            elif index == 0x60FF: 
                raw_target = int.from_bytes(data, byteorder='little', signed=True)
                # --- ZLTECH FIX: Target Velocity is exactly 1 r/min units! ---
                t_left = float(raw_target) if subindex == 1 else self.physics.target_velocity_left
                t_right = float(raw_target) if subindex == 2 else self.physics.target_velocity_right
                self.physics.update_velocity_target(t_left, t_right)
                print(f"  -> Updated Target Velocity: L={t_left}, R={t_right}")

            elif index == 0x607A: 
                raw_target = int.from_bytes(data, byteorder='little', signed=True)
                t_left = raw_target if subindex == 1 else self.physics.target_position_left
                t_right = raw_target if subindex == 2 else self.physics.target_position_right
                self.physics.update_position_target(t_left, t_right)

            elif index == 0x6071: 
                raw_target = int.from_bytes(data, byteorder='little', signed=True)
                t_left = raw_target if subindex == 1 else self.physics.target_torque_left
                t_right = raw_target if subindex == 2 else self.physics.target_torque_right
                self.physics.update_torque_target(t_left, t_right)
                
            elif index in [0x6081, 0x6083, 0x6084]:
                p_vel_l = self.node.sdo[0x6081][1].phys
                p_vel_r = self.node.sdo[0x6081][2].phys
                p_acc_l = self.node.sdo[0x6083][1].phys
                p_acc_r = self.node.sdo[0x6083][2].phys
                p_dec_l = self.node.sdo[0x6084][1].phys
                p_dec_r = self.node.sdo[0x6084][2].phys
                self.physics.update_kinematic_profiles(p_vel_l, p_vel_r, p_acc_l, p_acc_r, p_dec_l, p_dec_r)

        except Exception as e:
            import traceback
            print(f"\n!!! [CRASH IN SDO WRITE THREAD] !!!\n{traceback.format_exc()}")

    def _hardware_watchdog_loop(self):
        """Watches for NMT state changes and hardware smoke."""
        last_nmt_state = self.node.nmt.state
        
        while True:
            # 1. Check for NMT State Changes (Network Safety)
            current_nmt_state = self.node.nmt.state
            if current_nmt_state != last_nmt_state:
                if current_nmt_state == 'OPERATIONAL':
                    self.node.tpdo[1].start(0.1)
                    self.node.tpdo[2].start(0.1)
                    drive_is_ready = (self.state_machine.state == "OPERATION_ENABLE")
                    self.physics.set_enabled(drive_is_ready)
                    print(f"[NMT] Node {self.node_id} is now OPERATIONAL.")
                else:
                    print(f"[NMT] Safety Triggered: Node is {current_nmt_state}. Cutting motor power!")
                    self.state_machine.force_shutdown() 
                    self.status_var.phys = self.state_machine.status_word 
                    self.physics.set_enabled(False)
                    self.node.tpdo[1].stop()
                    self.node.tpdo[2].stop()
                last_nmt_state = current_nmt_state

            # 2. Check for Hardware Faults (Physical Safety)
            # --- ZLTECH FIX: Use zlac_error_code and update 0x603F ---
            if self.physics.zlac_error_code != 0x00000000 and self.state_machine.state != 'FAULT':
                self.state_machine.trigger_fault()
                self.status_var.phys = self.state_machine.status_word
                self.physics.set_enabled(False) # Physically kill power
                
                # Write to the official 0x603F ZLTECH error registry
                if 0x603F in self.node.object_dictionary:
                    self.node.object_dictionary[0x603F].raw = self.physics.zlac_error_code.to_bytes(4, 'little')
                
                self.node.emcy.send(0xFF00, 0x01) 
                print(f"\n[🚨 ZLAC FAULT 🚨] Drive locked! Read 0x603F for details. Code: 0x{self.physics.zlac_error_code:08X}")
                
            time.sleep(0.05) # Check 20 times a second

    def update_pdos(self):
        while True:
            if self.node.nmt.state == 'OPERATIONAL':
                self.node.sdo[0x606C][1].phys = int(self.physics.current_velocity_left)
                self.node.sdo[0x606C][2].phys = int(self.physics.current_velocity_right)
            time.sleep(0.05)

    def start(self):
        # 1. Start the CANopen loop
        self.pdo_thread = threading.Thread(target=self.update_pdos, daemon=True)
        self.pdo_thread.start()
        
        # 2. Start the Virtual Microprocessor & Motor Physics
        self.physics.start()
        
        # 3. Start the Hardware Safety Monitor
        self.watchdog_thread = threading.Thread(target=self._hardware_watchdog_loop, daemon=True)
        self.watchdog_thread.start()
