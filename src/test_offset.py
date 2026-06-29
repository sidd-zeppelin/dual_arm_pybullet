import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Let's say T_target is the grasp target
# We can visualize the axes
def draw_frame(T, length=0.1):
    pos = T[:3, 3]
    R = T[:3, :3]
    p.addUserDebugLine(pos, pos + R[:,0]*length, [1,0,0], 3)
    p.addUserDebugLine(pos, pos + R[:,1]*length, [0,1,0], 3)
    p.addUserDebugLine(pos, pos + R[:,2]*length, [0,0,1], 3)

T_target = np.eye(4)
T_target[:3, 3] = [0.5, 0, 0.5]
T_target[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]] # point down

# original target
draw_frame(T_target, 0.2)

# with +
T_plus = T_target.copy()
T_plus[:3, 3] += 0.107 * T_plus[:3, 2]
p.addUserDebugText("+0.107 Z", T_plus[:3, 3], [1,0,0])

# with -
T_minus = T_target.copy()
T_minus[:3, 3] -= 0.107 * T_minus[:3, 2]
p.addUserDebugText("-0.107 Z", T_minus[:3, 3], [0,1,0])

time.sleep(5)
p.disconnect()
