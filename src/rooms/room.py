class Room:
    def __init__(self,  label: str, x: float, y: float, x_lim: tuple, y_lim: tuple):
        """x, y are the center's coordinates;
        x_lim, y_lim are the limits of the room relative to its own center."""

        self.label = label
        self.x = x
        self.y = y
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.neighbors = []

    def add_neighbor(self, neighbor: 'Room') -> None:
        self.neighbors.append(neighbor)
