# pyrefly: ignore [missing-import]
import pybullet as p
import pybullet_data


class World:

    def __init__(self, config):

        self.config = config

        gui = config["simulation"]["gui"]

        if gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )

        p.setGravity(0, 0, -9.81)

        p.loadURDF("plane.urdf")
        
        # table
        self.table_id = p.loadURDF(
            "table/table.urdf", 
            basePosition=[0, 0, 0], 
            useFixedBase=True
        )
        
        # Central torso stand (visual only)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.75], rgbaColor=[0.5, 0.5, 0.5, 1.0])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[0, -0.6, 0.75])
        
        visual_shoulder = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.05, 0.05], rgbaColor=[0.5, 0.5, 0.5, 1.0])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shoulder, basePosition=[0, -0.6, 1.5])

    def step(self):

        p.stepSimulation()