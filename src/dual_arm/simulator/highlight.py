import pybullet as p

def highlight_robot(
    robot_id,
    color
):
    
    for j in range (-1, p.getNumJoints(robot_id)):
        p.changeVisualShape(
        robot_id,
        j,
        rgbaColor=color
        )