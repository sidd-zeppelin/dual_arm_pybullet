import pybullet as p
import pybullet_data
import time

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

lower_limits = []
upper_limits = []
joint_ranges = []
rest_poses = []

num_joints = p.getNumJoints(robot_id)
for j in range(num_joints):
    info = p.getJointInfo(robot_id, j)
    if info[2] != p.JOINT_FIXED:
        lower_limits.append(info[8])
        upper_limits.append(info[9])
        joint_ranges.append(info[9] - info[8])
        rest_poses.append(0.0)

print(f"Num DOFs: {len(lower_limits)}")

# Try IK with length 7 (what user did essentially, or rather mismatch)
try:
    ik_7 = p.calculateInverseKinematics(
        robot_id, 11, [0,0,0], [0,0,0,1],
        lowerLimits=lower_limits[:7],
        upperLimits=upper_limits[:7],
        jointRanges=joint_ranges[:7],
        restPoses=rest_poses[:7]
    )
    print("IK 7 succeeded")
except Exception as e:
    print("IK 7 failed:", e)

# Try IK with length 9 (correct)
try:
    ik_9 = p.calculateInverseKinematics(
        robot_id, 11, [0,0,0], [0,0,0,1],
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses
    )
    print("IK 9 succeeded")
except Exception as e:
    print("IK 9 failed:", e)

p.disconnect()
