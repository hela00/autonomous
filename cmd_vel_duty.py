import socket
import struct
import can
import time

# Initialize CAN interface
can_interface = 'can0'
bus = can.interface.Bus(can_interface, interface='socketcan')

# Parameters for motor control
MAX_SPEED = 2.0  # Max speed of the robot in m/s (forward speed)
WHEEL_BASE_WIDTH = 0.21  # Distance between the left and right wheels in meters
MAX_DUTY_CYCLE = 0.07  # Max duty cycle for the motor

# Define CAN IDs
SLAVE1_CAN_ID = 2484273056
SLAVE3_CAN_ID = 0x1413FFC8

# Function to send CAN commands to the motors
def send_motor_commands(left_duty_cycle, right_duty_cycle):
    # Send command to the left motor (SLAVE1)
    send_command(SLAVE1_CAN_ID, left_duty_cycle)
    
    # Send command to the right motor (SLAVE3)
    send_command(SLAVE3_CAN_ID, right_duty_cycle)

# Function to send a command via CAN
def send_command(can_id, duty_cycle):
    try:
        # Convert duty cycle to appropriate format (e.g., scaling it)
        duty_cycle_value = int(duty_cycle * 1000)  # Scale to a suitable range

        # Pack the data into a byte array
        data = struct.pack('<I', duty_cycle_value)  # Assuming 4-byte integer

        # Create CAN message
        message = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)

        # Send CAN message
        bus.send(message)
        print("Sent CAN message to {}: Duty Cycle = {}".format(can_id, duty_cycle))

    except can.CanError as e:
        print("Error sending CAN message: {}".format(e))

# Function to calculate duty cycles for both motors based on cmd_vel
def calculate_duty_cycles(linear_x, angular_z):
    # Calculate left and right motor speeds using a differential drive model
    left_speed = linear_x - angular_z * WHEEL_BASE_WIDTH / 2
    right_speed = linear_x + angular_z * WHEEL_BASE_WIDTH / 2

    # Normalize speeds to duty cycle range
    left_duty_cycle = max(min(left_speed / MAX_SPEED, 1), -1) * MAX_DUTY_CYCLE
    right_duty_cycle = max(min(right_speed / MAX_SPEED, 1), -1) * MAX_DUTY_CYCLE

    return left_duty_cycle, right_duty_cycle

# Function to process and transform received cmd_vel data
def process_cmd_vel_data(data):
    try:
        # Parse the received data (linear_x, angular_z)
        linear_x, angular_z = map(float, data.split(','))

        # Print the received velocities (for debugging)
        print("Received cmd_vel: linear_x = {}, angular_z = {}".format(linear_x, angular_z))

        # Calculate duty cycles for both motors
        left_duty_cycle, right_duty_cycle = calculate_duty_cycles(linear_x, angular_z)

        # Send the calculated duty cycles via CAN to both motors
        send_motor_commands(left_duty_cycle, right_duty_cycle)

    except Exception as e:
        print("Error processing cmd_vel data: {}".format(e))

# Socket server function to receive cmd_vel data from Jetson Nano
def socket_server():
    server_ip = '0.0.0.0'
    port = 5000

    # Create a TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, port))
    server_socket.listen(1)
    
    print("Listening for connections on {}:{}".format(server_ip, port))
    
    while True:
        # Wait for a connection
        client_socket, client_address = server_socket.accept()
        print("Connection from {}".format(client_address))

        try:
            # Receive data from the client (Jetson Nano)
            while True:
                data = client_socket.recv(1024).decode()
                if not data:
                    break  # Exit if no data received

                print("Received data: {}".format(data))

                # Process the received cmd_vel data
                process_cmd_vel_data(data)

        except Exception as e:
            print("Error receiving data: {}".format(e))

        finally:
            client_socket.close()

if __name__ == "__main__":
    socket_server()
