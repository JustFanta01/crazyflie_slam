import logging
import struct
from datetime import datetime
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp import init_drivers
from cflib.utils.callbacks import Caller
import csv

# URI of the Crazyflie (update this based on your setup)
uri = 'radio://0/83/2M/E7E7E7E7EA'

# filename
log_file = f"crazyflie_log_{datetime.today().strftime('%Y-%m-%d')}.csv"

# header logfile
header = ['id', 'front', 'right', 'back', 'left', 'x', 'y', 'yaw']
log_id = 0


def log_data(data):
    global log_id
    data = [log_id, *data]
    log_id += 1

    # print(data)
    with open(log_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)

# Custom CRTP port (matches the firmware port)
CUSTOM_CTRP_PORT = 0x09

def custom_sensor_data_handler(packet):
    """
    Callback to handle incoming CRTP packets from the Crazyflie.
    """
    # The packet is a byte array, so we need to unpack the data
    # The format string is defined based on the packet structure in the firmware: 
    # a message (10 bytes), deltaX (2 bytes), deltaY (2 bytes), and squal (1 byte) and padding (1 byte)
    # Here is the format string: https://docs.python.org/3/library/struct.html
    
    # format_string = "10s hh B h"
    format_string = "f f f H H H H H bb"

    print(len(packet))
    print(struct.calcsize(format_string))

    if len(packet) == (struct.calcsize(format_string)):
        # Unpack the data from the packet
        # msg, deltaX, deltaY, squal, shutter = struct.unpack(format_string, packet)
        yaw, x, y, front, back, left, right, up, padding1, padding2 = struct.unpack(format_string, packet)

        data = [front, right, back, left, x, y, yaw]
        log_data(data)

        # If you want to keep the raw packet, use the following line
        # print(f"Raw packet: {packet}")
        
        # Print the unpacked values
        # print(f"Message: {msg_decoded}")
        # print(f"Received - deltaX: {deltaX}, deltaY: {deltaY}, squal: {squal}, shutter: {shutter}")

        # print(f"Received - yaw: {yaw}, front: {front}, x: {x}")
        print(f"Received - front: {front}")
    else:
        print(f"Received unknown packet: {packet.decode('utf-8', errors='ignore')}")
        print(f"Raw packet: {packet}")


if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    # Create a Crazyflie instance
    cf = Crazyflie(rw_cache='./cache')

    # Initialize the drivers for Crazyflie
    init_drivers(enable_debug_driver=False)    

    # Register the callback for handling incoming CRTP packets
    # NOTE: 
    # - The callbacks are defined in the Crazyflie class (https://github.com/bitcraze/crazyflie-lib-python/tree/master/cflib/crazyflie
    #   and https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/crazyflie/__init__.py). 
    # - The crazyflie class contains a attribute cf.appchannel which is an instance of the AppChannel class. 
    # - List of callbacks available can be found here: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/#callbacks
    # - The crazyflie class already a appchannel instance, so we can directly use it to register the callback.

    cf.appchannel.packet_received.add_callback(custom_sensor_data_handler)

    # Creating the log file and writing the header
    with open(log_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)

    # Connect to the Crazyflie
    cf.open_link(uri)
    
    print('Connecting to %s' % uri)    
        
    # Start receiving sensor data
    print("Listening for sensor data...")
    try:
        while True:
            pass  # Keep the script running to listen for packets
    except KeyboardInterrupt:
        print("Exiting...")
