import numpy as np
class Stone:
    def __init__(self, center: np.ndarray, width: float, height: float):
        self.center = center
        self.width = width
        self.height = height

        # distance from center to corners
        c2tr = np.array([width, height]) / 2
        c2br = np.array([width, -height]) / 2
        
        #pos of corners
        self.top_right = center + c2tr
        self.bottom_right = center + c2br
        self.top_left = center - c2br
        self.bottom_left = center - c2tr
        
        #halfspace representation of stepping stone
        #be inside all 4 sides
        # Ax <= b
        self.A = np.array([[1,0],[0,1],[-1,0],[0,-1]])
        self.b = np.concatenate([c2tr]*2) + self.A.dot(center)
    
class SteppingStones:
    def __init__(self):
        self.stones = []
    def add_stones(self, stone: Stone):
        self.stones.append(stone)
    def num_stones(self):
        return len(self.stones)