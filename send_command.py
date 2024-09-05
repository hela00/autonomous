import can
import struct
import time

# Initialize CAN interface
can_interface = 'can0'
bus = can.interface.Bus(can_interface, interface='socketcan')



# Define CAN IDs
SLAVE1_CAN_ID = 2484273056  
SLAVE3_CAN_ID = 0x1413FFC8 

def send_command(action, value):
    # Check the action type and calculate the raw values accordingly
    if action == 0:
        action_spec_raw = 0
        load_spec_physical = 0  # No load for release
    elif action == 1:
        action_spec_raw = 1
        load_spec_physical = value  # Brake value
    elif action == 2:
        action_spec_raw = 2
        load_spec_physical = value  # Current value
    elif action == 3:
        action_spec_raw = 3
        load_spec_physical = value  # Speed value
    elif action == 4:
        action_spec_raw = 4
        load_spec_physical = value  # Duty cycle value
    elif action == 5:
        action_spec_raw = 5
        load_spec_physical = 0  # Error state
    else:
        print("Invalid action")
        return

    # Convert the physical value of LOAD_SPEC to the raw value
    load_spec_raw = int((load_spec_physical + 100000) / 0.001)

    # Pack the data into a byte array
    data = struct.pack('<I', load_spec_raw) + struct.pack('<B', action_spec_raw & 0x0F)

    # Create CAN message
    message = can.Message(arbitration_id=SLAVE1_CAN_ID, data=data, is_extended_id=True)

    # Send CAN message
    try:
        bus.send(message)
        print(f"Sent message: {message}")
    except can.CanError as e:
        print(f"Error sending CAN message: {e}")

def receive_feedback():
    while True:
        message = bus.recv()
        #print(f"Received message: {message}")
        if message.arbitration_id == SLAVE3_CAN_ID:
            load_spec_raw = struct.unpack('<I', message.data[0:4])[0]
            action_spec_raw = message.data[4]
            
            # Convert raw values back to physical values
            load_spec_physical = load_spec_raw * 0.001 - 100000
            action_spec_physical = action_spec_raw & 0x0F

            
            print(f"LOAD_SPEC: {load_spec_physical}")
            print(f"ACTION_SPEC: {action_spec_physical}")
       


if __name__ == "__main__":
    

    while(True):
        send_command(4, 0.1)
        time.sleep(0.1)
        

    receive_feedback()