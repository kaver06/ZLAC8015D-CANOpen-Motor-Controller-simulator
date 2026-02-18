class CiA402StateMachine:
    def __init__(self):
        # Initial State: Switch On Disabled
        self.state = 'SWITCH_ON_DISABLED'
        self.status_word = 0x0250  # xxxx xxxx x01x 0000
        
    def force_shutdown(self):
        """Called when NMT fails or Emergency Stop occurs"""
        self.state = 'SWITCH_ON_DISABLED'
        self.status_word = 0x0250

    def process_control_word(self, control_word):
        """
        STRICT CiA 402 Transition Table.
        Rejects '0x0F' if sent prematurely.
        """
        
        # Extract Command Bits
        # Bit 0: Switch On
        # Bit 1: Enable Voltage
        # Bit 2: Quick Stop (Active Low)
        # Bit 3: Enable Operation
        # Bit 7: Fault Reset
        
        cw_switch_on = (control_word >> 0) & 1
        cw_enable_voltage = (control_word >> 1) & 1
        cw_quick_stop = (control_word >> 2) & 1
        cw_enable_op = (control_word >> 3) & 1
        cw_fault_reset = (control_word >> 7) & 1

        # --- FAULT RESET ---
        if self.state == 'FAULT':
            if cw_fault_reset:
                self.state = 'SWITCH_ON_DISABLED'
                self.status_word = 0x0250
            return self.status_word

        # --- STRICT TRANSITION LOGIC ---
        
        # 1. State: SWITCH_ON_DISABLED
        # Target: READY_TO_SWITCH_ON
        # Command Requirement: Shutdown (0x06) -> Voltage=1, QuickStop=1, SwitchOn=0
        if self.state == 'SWITCH_ON_DISABLED':
            if cw_enable_voltage and cw_quick_stop:
                # STRICT CHECK: If user sends 0x07 or 0x0F, 'Switch On' (Bit 0) is 1.
                # We must REJECT this to prevent jumping steps.
                if cw_switch_on == 0: 
                    self.state = 'READY_TO_SWITCH_ON'
                    self.status_word = 0x0231
            # If command was 0x00 or 0x02 (Disable Voltage), stay here.

        # 2. State: READY_TO_SWITCH_ON
        # Target: SWITCHED_ON
        # Command Requirement: Switch On (0x07) -> SwitchOn=1, Voltage=1, QuickStop=1, EnableOp=0
        elif self.state == 'READY_TO_SWITCH_ON':
            if cw_switch_on and cw_enable_voltage and cw_quick_stop:
                # STRICT CHECK: If user sends 0x0F, 'Enable Op' (Bit 3) is 1.
                # We must REJECT this. You cannot enable operation directly from Ready.
                if cw_enable_op == 0:
                    self.state = 'SWITCHED_ON'
                    self.status_word = 0x0233
            
            # Fallback: If voltage or quickstop lost, go back
            elif not (cw_enable_voltage and cw_quick_stop):
                self.state = 'SWITCH_ON_DISABLED'
                self.status_word = 0x0250

        # 3. State: SWITCHED_ON
        # Target: OPERATION_ENABLE
        # Command Requirement: Enable Operation (0x0F) -> EnableOp=1, plus others
        elif self.state == 'SWITCHED_ON':
            if cw_enable_op and cw_switch_on and cw_enable_voltage and cw_quick_stop:
                self.state = 'OPERATION_ENABLE'
                self.status_word = 0x0237
                
            # Fallback logic
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
