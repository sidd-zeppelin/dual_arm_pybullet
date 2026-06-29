# pyrefly: ignore [missing-import]
import pybullet as p


def load_robot(robot_cfg):

    robot_id = p.loadURDF(
        robot_cfg["urdf"],
        basePosition=robot_cfg["position"],
        baseOrientation=robot_cfg["quaternion"],
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
    )

    return robot_id