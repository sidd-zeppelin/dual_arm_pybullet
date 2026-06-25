import pybullet as p
import pybullet_data


class World:

    def __init__(self, config):

        self.config = config

        gui = config["simulation"]["gui"]

        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )

        p.setGravity(0, 0, 0)

        p.loadURDF("plane.urdf")

    def step(self):

        p.stepSimulation()