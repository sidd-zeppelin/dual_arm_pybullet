import pybullet as p


def load_robot(robot_cfg):

    robot_id = p.loadURDF(
        robot_cfg["urdf"],
        basePosition=robot_cfg["position"],
        baseOrientation=robot_cfg["quaternion"],
        useFixedBase=True
    )

    return robot_id