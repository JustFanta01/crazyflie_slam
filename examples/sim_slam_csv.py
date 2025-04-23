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
    file_path = '../../CPS/dataset CF 38x80,5cm/crazyflie_log_perimetro.csv'
    # Use genfromtxt to read the CSV in a NumPy array
    data_frame = pd.read_csv(file_path)

   # Estrai le colonne specifiche
    id = data_frame['id']
    front = data_frame['front'] / 1000
    right = data_frame['right'] / 1000
    back = data_frame['back'] / 1000
    left = data_frame['left'] / 1000
    x = data_frame['x']
    y = data_frame['y']
    yaw = data_frame['yaw'] * np.pi / 180.0

    # Create "ranges" that contains front, right, back and left
    ranges = np.array([front, right, back, left])

    # Create "scanangles" that contains 0, 90, 180, 270
    scan_angles = np.array([0, 1/2*np.pi, np.pi, 3/2*np.pi]).T

    # Create "scanangles" that contains x, y and yaw
    states = np.array([x, y, yaw])
    # print("states[:, 0:5] (normale)", states[:, 0:5])

    # Get Delta of the states 
    motion_updates = np.diff(states, axis=1, prepend=np.zeros((3, 1)))
    # print("motion_updates[:, 0:5]", motion_updates[:, 0:5])

    states = np.cumsum(motion_updates, axis=1)

    # print("states[:, 0:5] (cumsum)", states[:, 0:5])

    # Useful values
    system_noise_variance = np.diag([0.0, 0.0, 0.0]) # assume zero systems noise variance
    correlation_matrix = np.array([
        [0, -1],
        [-1, 10],
    ])
    slam_states = np.zeros_like(states)

    # Init the SLAM agent
    slam_agent = SLAM(
        params=init_params_dict(size=2, resolution=100),
        n_particles=int(args.n_particles),
        current_state=states[:, 0],
        system_noise_variance=system_noise_variance,
        correlation_matrix=correlation_matrix,
    )

    # Main loop
    plt.ion()
    
    # TODO: find a way to intecept CTRL+C on the plot

    fig = plt.figure(figsize=(7,7))
    for t in range(states.shape[1]):
        slam_states[:, t]  = slam_agent.update_state(
            ranges[:, t],
            scan_angles,
            motion_updates[:, t], # un lista di delta nelle posizioni
        )

        plt.clf()
        slam_map = slam_agent.map
        idx_slam = discretize(slam_states[:2, :], slam_agent.params)
        idx_noise = discretize(states[:2, :], slam_agent.params)


        plt.imshow(slam_map, cmap="gray") # vmin=-50, vmax=100)
        plt.plot(idx_slam[1, :t+1], idx_slam[0, :t+1], "-r", label="slam")
        plt.title(f"Iterazione #{t}")
        plt.legend()
        plt.draw()
        plt.pause(0.01)


plt.ioff()
plt.imshow(slam_map, cmap="gray")
plt.plot(idx_slam[1, :], idx_slam[0, :], "-r", label="slam")
plt.title(f"Final map")
plt.legend()
plt.show()