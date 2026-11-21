"""
Ground Truth of the simulation world, defines:
- Nodes & their attributes
- Anchors & their position
- Interactions in the simulation
- Interface to show realistic derived measurements to clamour 
"""
from dataclasses import dataclass

@dataclass
class Tag: 
    """
    The real data of a simulated tag 
    """
    id: int 
    real_pos: tuple[int,int] 
    velocity: tuple[int,int] 

class World: 
    def __init__(self, dt, map, tags, anchors): 
        self.dt = dt 

    def update(self):
        """
        Updates the world by one timestep. 
        Nodes move according to their velocity & the world rules. 
        """
        pass

    def available_anchors_for(self, tag_id:int)->list[int]:
        """
        For a given tag ID, returns LOS anchors. 
        """

    def available_neighbors_for(self, tag_id:int)->list[int]:
        """
        For a given tag ID, returns LOS neighbors. 
        """

    def sim_range(self, sender_id:int, target_id:int)->float: 
        """
        Generates a realistic range measurement between 2 ranging nodes taking into account the map. 
        """

    def sim_imu(self, tag_id:int): 
        """
        Generates a realistic IMU sample from the last world update until now. 
        """

    
