class CiA402StateMachine:
    def __init__(self):

        self.state = "SWITCH_ON_DISABLED"
        self.status_word = 0x0000  # Default low 4 bits: 0000

    def process_control_word(self, control_word):
        """
        Takes the incoming 0x6040 Control Word and transitions the CiA402 state.
        Returns the updated 0x6041 Status Word.
        """
        # Initialization Step 0: Disable Voltage / Shutdown
        if control_word == 0x00:
            self.state = "SWITCH_ON_DISABLED"
            # Low 4 bits: 0000
            self.status_word = 0x0040 # Bit 6 is 1 for Switch On Disabled

        # Initialization Step 1: Shutdown command
        elif control_word == 0x06:
            self.state = "READY_TO_SWITCH_ON"
            # Low 4 bits: 0001
            self.status_word = 0x0021 # Bit 5 is 1 for Ready

        # Initialization Step 2: Switch On command
        elif control_word == 0x07:
            self.state = "SWITCHED_ON"
            # Low 4 bits: 0011
            self.status_word = 0x0023 # Bit 5 is 1 for Ready

        # Initialization Step 3: Enable Operation command
        elif control_word == 0x0F:
            self.state = "OPERATION_ENABLE"
            # Low 4 bits: 0111
            self.status_word = 0x0027 # Bit 5 is 1 for Ready
            
        return self.status_word
