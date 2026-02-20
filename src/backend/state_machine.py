class CiA402StateMachine:
    def __init__(self):
        self.state = 'SWITCH_ON_DISABLED'
        self.status_word = 0x0250
        
    def force_shutdown(self):
        """Called when NMT fails or Emergency Stop occurs"""
        self.state = 'SWITCH_ON_DISABLED'
        self.status_word = 0x0250

    def trigger_fault(self):
        """Called by the Hardware Monitor when the motor physically fails"""
        self.state = 'FAULT'
        self.status_word = 0x0208  # xxxx xxxx x000 1000 (Bit 3 HIGH = FAULT)

    def process_control_word(self, control_word):
        cw_switch_on = (control_word >> 0) & 1
        cw_enable_voltage = (control_word >> 1) & 1
        cw_quick_stop = (control_word >> 2) & 1
        cw_enable_op = (control_word >> 3) & 1
        cw_fault_reset = (control_word >> 7) & 1

        # --- FAULT RESET LOGIC ---
        if self.state == 'FAULT':
            # Master must send a command with Bit 7 HIGH to clear the fault
            if cw_fault_reset:
                self.state = 'SWITCH_ON_DISABLED'
                self.status_word = 0x0250
            return self.status_word

        # 1. State: SWITCH_ON_DISABLED
        if self.state == 'SWITCH_ON_DISABLED':
            if cw_enable_voltage and cw_quick_stop:
                if cw_switch_on == 0: 
                    self.state = 'READY_TO_SWITCH_ON'
                    self.status_word = 0x0231

        # 2. State: READY_TO_SWITCH_ON
        elif self.state == 'READY_TO_SWITCH_ON':
            if cw_switch_on and cw_enable_voltage and cw_quick_stop:
                if cw_enable_op == 0:
                    self.state = 'SWITCHED_ON'
                    self.status_word = 0x0233
            elif not (cw_enable_voltage and cw_quick_stop):
                self.state = 'SWITCH_ON_DISABLED'
                self.status_word = 0x0250

        # 3. State: SWITCHED_ON
        elif self.state == 'SWITCHED_ON':
            if cw_enable_op and cw_switch_on and cw_enable_voltage and cw_quick_stop:
                self.state = 'OPERATION_ENABLE'
                self.status_word = 0x0237
            elif not (cw_enable_voltage and cw_quick_stop):
                self.state = 'SWITCH_ON_DISABLED'
                self.status_word = 0x0250
            elif not cw_switch_on:
                self.state = 'READY_TO_SWITCH_ON'
                self.status_word = 0x0231

        # 4. State: OPERATION_ENABLE
        elif self.state == 'OPERATION_ENABLE':
            if not cw_enable_op:
                self.state = 'SWITCHED_ON'
                self.status_word = 0x0233
            if not cw_switch_on:
                self.state = 'READY_TO_SWITCH_ON'
                self.status_word = 0x0231
            if not (cw_enable_voltage and cw_quick_stop):
                self.state = 'SWITCH_ON_DISABLED'
                self.status_word = 0x0250
                
        return self.status_word
