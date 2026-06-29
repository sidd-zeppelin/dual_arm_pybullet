# pyrefly: ignore [missing-import]
import pybullet as p
import pybullet_data
import time

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

num_joints = p.getNumJoints(robot_id)
print("Num joints:", num_joints)
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    parent_index = info[16]
    print(f"Joint {i}: {info[1]} (Parent: {parent_index})")

p.disconnect()
