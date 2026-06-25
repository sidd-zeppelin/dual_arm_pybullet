import pybullet as p


def load_object(cfg):

    collision = p.createCollisionShape(
        p.GEOM_MESH,
        fileName=cfg["mesh"],
        meshScale=[cfg["scale"]] * 3
    )

    visual = p.createVisualShape(
        p.GEOM_MESH,
        fileName=cfg["mesh"],
        meshScale=[cfg["scale"]] * 3
    )

    body = p.createMultiBody(
        baseMass=cfg["mass"],
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=cfg["position"],
        baseOrientation=cfg["quaternion"]
    )

    return body