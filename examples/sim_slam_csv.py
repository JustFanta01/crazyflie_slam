import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from context import crazyslam
from crazyslam.slam import SLAM
from crazyslam.mapping import init_params_dict, discretize
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument(
    "--n_data_points",
    default=4,
    help="Number of data points to use for each scan",
)
parser.add_argument(
    "--n_particles",
    default=100,
    help="Number of particles in the particle filter",
)


if __name__ == '__main__':
    args = parser.parse_args()
    
    # Leggi il file CSV
    file_path = '../crazyflie_log.csv'

    # Use genfromtxt to read the CSV in a NumPy array
    data_frame = pd.read_csv(file_path)

   # Estrai le colonne specifiche
    id = data_frame['id']
    front = data_frame['front']
    right = data_frame['right']
    back = data_frame['back']
    left = data_frame['left']
    x = data_frame['x']
    y = data_frame['y']
    yaw = data_frame['yaw'] 

    # Create "ranges" that contains front, right, back and left
    ranges = np.array([front, right, back, left])

    # Create "scanangles" that contains 0, 90, 180, 270
    scanangles = np.array([0, 90, 180, 270]).T

    # Create "scanangles" that contains x, y and yaw
    states = np.array([x, y, yaw])
    print(states[46:50])

    # Get Delta of the states 
    motion_updates = np.diff(states, axis=1)#, prepend=np.zeros((1, 4)))
    print(motion_updates[46:50])
    states = np.cumsum(motion_updates, axis=1)
    print(states.shape)

    # Useful values
    system_noise_variance = np.diag([0.2, 0.2, 0.2])
    correlation_matrix = np.array([
        [0, -1],
        [-1, 10],
    ])
    slam_states = np.zeros_like(states)

    # Init the SLAM agent
    slam_agent = SLAM(
        params=init_params_dict(size=70, resolution=10),
        n_particles=int(args.n_particles),
        current_state=states[:, 0],
        system_noise_variance=system_noise_variance,
        correlation_matrix=correlation_matrix,
    )

    # Main loop
    for t in range(states.shape[1]):
        slam_states[:, t]  = slam_agent.update_state(
            ranges[:, t],
            scanangles,
            motion_updates[:, t], # un lista di delta nelle posizioni
        )    

    slam_map = slam_agent.map
    idx_slam = discretize(slam_states[:2, :], slam_agent.params)
    idx_noise = discretize(states[:2, :], slam_agent.params)

    plt.figure(figsize=(11, 11))
    plt.imshow(slam_map, cmap="gray")
    plt.plot(idx_slam[1, :], idx_slam[0, :], "-r", label="slam")
    plt.legend()
    plt.show()
