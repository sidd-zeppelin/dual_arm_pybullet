# Dual Arm Pybullet Simulation Environment for Dual Arm Grasping

## File Structure:
```
dual_arm_pybullet/

    - assets/
        - grasps/ (contains decoupled grasp files)
        - meshes/ (contains .obj files for the meshes)
        - urdf/ 

    - configs/
        -scene.yaml (settings for the scene in simulator)

    - src/
        - dual_arm/

            - dataset/
                - dg16m.py  (contains function to load grasps)

            - simulator/
                - objects.py (function to load the object in the simulator)
                - robots.py  (function to load the robot in the simulator)
                - world.py   (class for the world to load in the simulator)

            - utils/
                - config.py     (connects the settings from config.yaml to the code)

        - main.py
    
```

