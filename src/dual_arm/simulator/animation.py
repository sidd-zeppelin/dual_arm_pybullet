import time
# pyrefly: ignore [missing-import]
import pybullet as p

def animate_dual_arm_transition(world, config, left_robot, right_robot, leftIK, rightIK, left_q, right_q, steps=240):
    print("Animating motors to final grasp position...")
    
    left_start = [p.getJointState(left_robot, i)[0] for i in range(7)]
    right_start = [p.getJointState(right_robot, i)[0] for i in range(7)]
    
    for step in range(steps):
        alpha = step / float(steps)
        alpha = alpha * alpha * (3 - 2 * alpha) 
        
        left_current = [left_start[i] * (1 - alpha) + left_q[i] * alpha for i in range(7)]
        leftIK.set_configuration(left_current)
        
        right_current = [right_start[i] * (1 - alpha) + right_q[i] * alpha for i in range(7)]
        rightIK.set_configuration(right_current)
        
        world.step()
        time.sleep(config["simulation"]["timestep"])

    leftIK.set_configuration(left_q)
    rightIK.set_configuration(right_q)
