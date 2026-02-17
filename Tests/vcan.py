import can
import time

def test_virtual_can():
    print("Connecting to vcan0...")
    
 
    try:
        bus = can.interface.Bus(channel='vcan0', interface='socketcan', receive_own_messages=True)
        print("Successfully connected to vcan0!")
    except OSError as e:
        print(f"Error connecting to vcan0: {e}")
        print("Did you run the modprobe and ip link commands from the previous step?")
        return

    test_msg = can.Message(
        arbitration_id=0x123, 
        data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88], 
        is_extended_id=False
    )

    print(f"Sending message: {test_msg}")
    bus.send(test_msg)
    time.sleep(0.1) 

    received_msg = bus.recv(timeout=1.0)
    
    if received_msg:
        print(f"Success! Received message back: {received_msg}")
    else:
        print("Failed to receive the message. Something is wrong with the vcan0 routing.")

    bus.shutdown()

if __name__ == "__main__":
    test_virtual_can()
