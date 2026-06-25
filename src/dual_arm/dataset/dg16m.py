import h5py


class DG16MLoader:

    def __init__(self, h5_path):
        self.h5_path = h5_path

    def get_grasp(self, grasp_idx):

        with h5py.File(self.h5_path, "r") as f:

            grasp = f["grasps"]["grasps"][grasp_idx]
            
        return grasp