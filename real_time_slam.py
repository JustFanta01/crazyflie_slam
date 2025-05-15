import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from crazyslam.slam import SLAM
from crazyslam.mapping import init_params_dict, discretize
import pandas as pd
import queue
import threading
import logging
import struct
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp import init_drivers
from cflib.utils.callbacks import Caller

CUSTOM_CTRP_PORT = 0x09

parser = argparse.ArgumentParser()
parser.add_argument(
    "--n_particles",
    default=400,
    help="Number of particles in the particle filter",
)
parser.add_argument(
    "--cf_uri",
    default="radio://0/83/2M/E7E7E7E7EA",
    help="Crazyflie radio URI",
)

# common data structure to exchange and buffer data
data_queue = queue.Queue()

def custom_sensor_data_handler(packet):
    # [Thread-1]
    format_string = "f f f H H H H H bb"
    if len(packet) == (struct.calcsize(format_string)):
        yaw, x, y, front, back, left, right, up, padding1, padding2 = struct.unpack(format_string, packet)
        data = [front, right, back, left, x, y, yaw]
        data_queue.put(data)
    else:
        print(f"Received unknown packet: {packet.decode('utf-8', errors='ignore')}")
        print(f"Raw packet: {packet}")

if __name__ == '__main__':
    args = parser.parse_args()
    
    # -------------------------------
    # |   CRAZYFLIE CONFIGURATION   |
    # -------------------------------
    logging.basicConfig(level=logging.ERROR)
    uri = args.cf_uri
    cf = Crazyflie(rw_cache='./cache')          # Create a Crazyflie instance
    init_drivers(enable_debug_driver=False)     # Initialize the drivers for Crazyflie

    cf.appchannel.packet_received.add_callback(custom_sensor_data_handler)
    

    # ----------------------------
    # |    SLAM CONFIGURATION    |
    # ----------------------------
    # Useful values
    init_state = np.zeros(shape=(3,1))
    # system_noise_variance = np.diag([0.01, 0.01, 0.01])
    # system_noise_variance = np.diag([0.1, 0.1, 0.1])
    system_noise_variance = np.diag([0.0, 0.0, 0.0])
    
    freq = 10 # sample frequency [Hz]
    skip = freq
    size = 1
    resolution = 100
    
    correlation_matrix = np.array([
        [0, -1],
        [-1, 10],
    ])

    # Init the SLAM agent
    slam_agent = SLAM (
        params=init_params_dict(size, resolution),
        n_particles=int(args.n_particles),
        current_state=init_state,
        system_noise_variance=system_noise_variance,
        correlation_matrix=correlation_matrix,
    )
    

    # ---------------------------------
    # |   CRAZYFLIE START RECEIVING   |
    # ---------------------------------
    cf.open_link(uri)       # Connect to the Crazyflie
    print(f"Connecting to {uri}")    
        
    # Start receiving sensor data
    print("Listening for sensor data...")

    plt.ion()

    try:
        scan_angles = np.array([0, 1/2*np.pi, np.pi, 3/2*np.pi]).T
        (old_front, old_right, old_back, old_left, old_x, old_y, old_yaw) = data_queue.get()
        
        arrow_length = 10

        slam_states = []
        k = 0
        fig = plt.figure(figsize=(5,5))

        # ---------------------------------
        # |   REAL-TIME PROCESSING DATA   |
        # ---------------------------------
        while True: # [Main thread]
            (front, right, back, left, x, y, yaw) = data_queue.get()

            # print(f"yaw: {yaw}")
            
            # filter weird values
            if True: # used only for collapsing the code
                if front == 32766 or front == 0:
                    front = old_front
                else:
                    old_front = front

                if left == 32766 or left == 0:
                    left = old_left
                else:
                    old_left = old_left

                if right == 32766 or right == 0:
                    right = old_right
                else:
                    old_right = old_right
                
                if back == 32766 or back == 0:
                    back = old_back
                else:
                    old_back = back

            # prepare data for the SLAM algorithm
            yaw = yaw * np.pi / 180.0 # yaw in rad
            ranges = np.array([front / 1000, right / 1000, back / 1000, left / 1000]) # ranges in meters
            motion_update = np.array([x - old_x, y - old_y, yaw - old_yaw]) # motion delta
            
            slam_states.append (
                slam_agent.update_state (
                    ranges,
                    scan_angles,
                    motion_update
                )
            )

            # Update old values
            old_x = x
            old_y = y
            old_yaw = yaw

            # Visualization
            if k % skip == 0:
                plt.clf()
                slam_map = slam_agent.map

                plt.imshow(slam_map, cmap="gray")
                idx_slam = discretize(slam_states[k][:2], slam_agent.params)
                px, py = idx_slam
                
                plt.plot(py, px, "ro", label="slam")
                dx = arrow_length * np.cos(yaw)
                dy = arrow_length * np.sin(yaw)
                plt.arrow(py, px, dx, dy, head_width=3, head_length=3, fc='blue', ec='blue')

                plt.title(f"Iterazione #{k}")
                plt.legend()
                plt.draw()
                plt.pause(0.01)
                # plt.show(block=False)
            
            k += 1

    except KeyboardInterrupt:
        plt.ioff()
        print("Exiting...")