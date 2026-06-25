import numpy as np

from scipy.spatial.transform import Rotation


def pose_to_matrix(position, quaternion):

    T = np.eye(4)

    T[:3, :3] = Rotation.from_quat(
        quaternion
    ).as_matrix()

    T[:3, 3] = position

    return T


def matrix_to_pose(T):

    position = T[:3, 3]

    quaternion = Rotation.from_matrix(
        T[:3, :3]
    ).as_quat()

    return position, quaternion


def invert_transform(T):

    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4)

    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t

    return T_inv


def transform_point(T, point):

    point_h = np.append(point, 1)

    transformed = T @ point_h

    return transformed[:3]

def create_transform(
    position=(0, 0, 0),
    euler_deg=(0, 0, 0)
):

    T = np.eye(4)

    T[:3, :3] = Rotation.from_euler(
        "xyz",
        euler_deg,
        degrees=True
    ).as_matrix()

    T[:3, 3] = position

    return T