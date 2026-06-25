class AssignmentManager:
    
    def __init__(self):
        self.mode = "left"
        
        self.left_grasp_idx = None
        self.right_grasp_idx = None
        
        self.left_grasp_T = None
        self.right_grasp_T = None
        
        self.left_hand_T = None
        self.right_hand_T = None
        
    def current_arm(self):
        return self.mode

    def accept(self,
               grasp_idx,
               T_world_grasp,
               T_world_hand):
        
        if self.mode == "left":
            self.left_grasp_idx = grasp_idx
            self.left_grasp_T = T_world_grasp
            self.left_hand_T = T_world_hand
            
            self.mode = "right"
            grasp_idx +=1
            
            print(f"Left arm assigned with grasp idx {grasp_idx}\n")
        
        elif self.mode == "right":
            self.right_grasp_idx = grasp_idx
            self.right_grasp_T = T_world_grasp
            self.right_hand_T = T_world_hand
            
            self.mode = "ik"
            grasp_idx +=1
            
            print(f"Right arm assigned with grasp idx {grasp_idx}\n")
            
    def done(self):
        return self.mode == "ik"