import socket

# Function to process and print received cmd_vel data
def process_cmd_vel_data(data):
    try:
        # Parse the received data (linear_x, angular_z)
        linear_x, angular_z = data.split(',')
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        
        # Example: Print the received velocities
        print("Received cmd_vel: linear_x = {}, angular_z = {}".format(linear_x, angular_z))
        
        # TODO: Convert to motor commands and send via CAN
        # send_can_commands(linear_x, angular_z)
        
    except Exception as e:
        print("Error processing data: {}".format(e))

# Function to handle socket communication
def socket_server():
    # IP and port to listen on (0.0.0.0 listens on all available interfaces)
    server_ip = '0.0.0.0'
    port = 5000
    
    # Create a TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Bind the socket to the IP and port
    server_socket.bind((server_ip, port))
    
    # Listen for incoming connections
    server_socket.listen(1)
    print("Listening for connections on {}:{}".format(server_ip, port))
    
    while True:
        # Wait for a connection
        client_socket, client_address = server_socket.accept()
        print("Connection from {}".format(client_address))
        
        try:
            # Receive data from the client (Jetson Nano)
            data = client_socket.recv(1024).decode()
            if data:
                print("Received data: {}".format(data))
                
                # Process the received cmd_vel data
                process_cmd_vel_data(data)
            
        except Exception as e:
            print("Error receiving data: {}".format(e))
        
        finally:
            # Close the connection
            client_socket.close()

if __name__ == "__main__":
    socket_server()


