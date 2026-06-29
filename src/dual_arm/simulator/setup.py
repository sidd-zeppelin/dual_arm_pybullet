# pyrefly: ignore [missing-import]
import pybullet as p

from dual_arm.utils.config import load_config
from dual_arm.simulator.world import World
from dual_arm.simulator.robots import load_robot
from dual_arm.simulator.objects import load_object, drop_object_on_table
from dual_arm.grasping.ik import PandaIK
from dual_arm.simulator.filters import setup_self_collision_filters

def setup_simulation(config_path, limits_path):
    config = load_config(config_path)
    joint_limits = load_config(limits_path)
    
    world = World(config)

    left_robot = load_robot(config["robots"]["left"])
    setup_self_collision_filters(left_robot)

    right_robot = load_robot(config["robots"]["right"])
    setup_self_collision_filters(right_robot)

    rightIK = PandaIK(right_robot, joint_limits)
    leftIK = PandaIK(left_robot, joint_limits)
    leftIK.open_gripper()
    rightIK.open_gripper()
              
    object_id = load_object(config["object"])
    
    drop_object_on_table(object_id, world.table_id, clearance=0.05)
    
    print("Waiting for object to stabilize on the table...")
    for _ in range(480):
        world.step()

    return config, world, left_robot, right_robot, leftIK, rightIK, object_id
