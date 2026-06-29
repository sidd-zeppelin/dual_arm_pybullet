# pyrefly: ignore [missing-import]
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

def drop_object_on_table(object_id, table_id, clearance=0.05):
    table_aabb = p.getAABB(table_id)
    table_z_max = table_aabb[1][2]
    
    obj_aabb = p.getAABB(object_id)
    obj_z_min = obj_aabb[0][2]
    
    current_pos, current_quat = p.getBasePositionAndOrientation(object_id)
    z_offset = (table_z_max + clearance) - obj_z_min
    new_pos = (current_pos[0], current_pos[1], current_pos[2] + z_offset)
    
    p.resetBasePositionAndOrientation(object_id, new_pos, current_quat)