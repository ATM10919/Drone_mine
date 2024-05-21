import numpy as np

def transform_point(x_rel, y_rel, z_rel, tx, ty, tz, theta_x, theta_y, theta_z, sx, sy, sz):
    # Create transformation matrices
    T_translation = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

    T_rotation_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta_x), -np.sin(theta_x), 0],
        [0, np.sin(theta_x), np.cos(theta_x), 0],
        [0, 0, 0, 1]
    ])

    T_rotation_y = np.array([
        [np.cos(theta_y), 0, np.sin(theta_y), 0],
        [0, 1, 0, 0],
        [-np.sin(theta_y), 0, np.cos(theta_y), 0],
        [0, 0, 0, 1]
    ])

    T_rotation_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0, 0],
        [np.sin(theta_z), np.cos(theta_z), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T_scaling = np.array([
        [sx, 0, 0, 0],
        [0, sy, 0, 0],
        [0, 0, sz, 0],
        [0, 0, 0, 1]
    ])

    # Combine transformation matrices
    T_total = np.linalg.multi_dot([T_translation, T_rotation_z, T_rotation_y, T_rotation_x, T_scaling])

    # Represent the point in homogeneous coordinates
    point_rel = np.array([x_rel, y_rel, z_rel, 1])

    # Transform the point to absolute coordinates
    point_abs = np.dot(T_total, point_rel)

    # Extract absolute coordinates
    x_abs, y_abs, z_abs, _ = point_abs

    return x_abs, y_abs, z_abs
