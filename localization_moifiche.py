"""Localization module

This module implements an algorithm that updates the state estimate of the
Crazyflie using a particle filter
"""


import numpy as np
np.random.seed(42)  # Fixed seed for reproducible results
from crazyslam.mapping import target_cell, discretize


def init_random_particles(n):
    """Initializes a set of n random particles"""
    random_states = np.zeros((3, n))
    random_states[2, :] = np.random.uniform(
        low=-5,
        high=2,
        size=n
    ).reshape(1, n)
    random_particles = np.concatenate(
        (
            random_states,
            (1/random_states.shape[1])*np.ones((1, random_states.shape[1]))
        ),
        axis=0
    )
    return random_particles





def normalize_weights(weights):
    """Normalizes weights (Softmax) so that the sum of the weights equals 1"""
    # Max shift to avoid exploding exp values
    weights -= weights.max()
    normalized_weights = np.exp(weights)/sum(np.exp(weights))
    return normalized_weights


def get_correlation_score(grid_map, target_cells, correlation_matrix):
    """Computes the correlation score of a single particle

    Args:
        grid_map: Occupancy grid map
        target_cells: 2D or 3D vector (2 x n_cells x n_particles)
            of index coordinates
        correlation_matrix: Matrix with the scores hits/misses

    Returns:
        Colleration score for a single particle
    """
    if target_cells.ndim == 3:
        n_particles = target_cells.shape[2]
        n_cells = target_cells.shape[1]
        target_maps = np.vstack(
            [grid_map[target_cells[0, :, i], target_cells[1, :, i]] > 0
             for i in range(n_particles)]  # TODO: delete for loop
        )
        hits = np.sum(target_maps, axis=1)
        misses = n_cells * np.ones_like(hits) - hits
    elif target_cells.ndim == 2:
        target_map = grid_map[target_cells[0, :], target_cells[1, :]] > 0
        hits = np.sum(target_map)
        misses = target_map.size - hits
    return hits*correlation_matrix[1, 1] + misses*correlation_matrix[0, 1]


def update_particle_weights(
    particles, correlation_matrix,
    grid_map, map_params,
    ranges, angles
):
    """Updates the particle weights

    Args:
        particles: Set of state estimates and their corresponding weight
        correlation_matrix: LIDAR/MAP correlation matrix for weight updates
        grid_map: Occupancy grid map
        map_params: Grid map parameters dictionary
        ranges: Set on range inputs from sensor
        angles: Scan angles

    Returns:
        Set of particles with updated weights
    """
    # Find target cells
    target_cells = target_cell(particles[:3, :], ranges, angles)
    target_cells = discretize(target_cells, map_params)
    # Compute correlation scores
    particles[-1, :] = get_correlation_score(
        grid_map,
        target_cells,
        correlation_matrix,
    )
    # Normalize all weights
    particles[-1, :] = normalize_weights(particles[-1, :])
    return particles


def get_best_particle(particles):
    """Returns the best particle (the one with the max weigth)

    Args:
        particles: Set of state estimates and their corresponding weight

    Returns:
        The particle with the biggest weight
    """
    return particles[:, np.argmax(particles[-1, :])]


def compute_effective_n_particles(weights):
    """Compute effective number of particles given their weights"""
    return (weights.sum()**2) / (weights**2).sum()


def resample(particles):
    """Resamples particles given their weights/probability"""
    idx = np.random.choice(
        a=np.arange(0, particles.shape[1], 1),
        size=particles.shape[1],
        replace=True,
        p=particles[3, :]
    )
    return particles[:, idx]


def get_state_estimate(
    particles,
    system_noise_variance, correlation_matrix,
    grid_map, map_params,
    ranges, angles,
    resample_threshold
):
    """Computes a state estimate using a particle filter

    Args:
        particles: Set of state estimates and their corresponding weight
        system_noise_variance:
        correlation_matrix: LIDAR/MAP correlation matrix for weight updates
        grid_map: Occupancy grid map
        map_params: Grid map parameters dictionary
        ranges: Set on range inputs from sensor
        angles: Scan angles
        resample_threshold: Threshold for resampling

    Returns:
        State vector representing the new state estimate.
        Set of updated particles (maybe resampled)

    """
    # Propagate the particles (noise removed as per project requirements)
    # Weight update
    particles = update_particle_weights(
        particles,
        correlation_matrix,
        grid_map,
        map_params,
        ranges,
        angles,
    )
    # Choose the best particle to update the pose
    best_state_estimate = get_best_particle(particles)
    # Resample if the effective number of particles is smaller than a threshold
    if compute_effective_n_particles(particles[-1, :]) < resample_threshold:
        particles = resample(particles)
    # Return new pose and particles
    return best_state_estimate[:-1], particles
