from matplotlib import pyplot as plt
import numpy as np
from scipy.signal import convolve2d
from scipy.ndimage.measurements import label

class OccSpace:
    def __init__(self, grid) -> None:
        '''
        Initialize OccSpace object based on occupancy grid

        Parameters
        ----------
        grid: occupancy grid numpy array, where -1 is lane, 0 is unknown, 1 is driveable
        '''
        self.grid = grid
    
    def generate_mesh_grid(self, resolution) -> None:
        # Sample mesh grid with number of points horizontally equal to resolution
        # Array of edge statuses for each edge, indexed by vertex coordinates and vertical/horizontal
        self.edge_array = np.zeros((resolution, resolution, 2), dtype=bool)
        sample_x = np.linspace(0, self.grid.shape[0]-1e-5, resolution)
        sample_y = np.linspace(0, self.grid.shape[1]-1e-5, resolution)
        
        # Iterate through sampled vertices, and record the occupancy of each edge
        for i in range(resolution):
            for j in range(resolution):
                if i < resolution-1:
                    # Record occupancy of horizontal edge
                    xx, y = sample_x[i:i+2].astype(int), sample_y[j].astype(int)
                    self.edge_array[i, j, 1] = np.all(self.grid[xx, y] == 1) and not \
                                               np.any(self.grid[xx[0]:xx[1], j] == -1)

                if j < resolution-1:
                    # Record occupancy of vertical edge
                    x, yy = sample_y[i].astype(int), sample_y[j:j+2].astype(int)
                    self.edge_array[i, j, 0] = np.all(self.grid[x, yy] == 1) and not \
                                               np.any(self.grid[i, yy[0]:yy[1]] == -1)
        return self.edge_array

    def erode_mesh(self):
        '''
        removes edges that do not have any adjacent parallel neighbours
        '''
        kernel = np.ones((1,3), dtype=int)
        for i, k in enumerate([kernel, kernel.T]):
            self.edge_array[:, :, i] *= convolve2d(self.edge_array[:, :, i], k, 
                                                   mode='same', boundary='fill', 
                                                   fillvalue=1) > 1
        return self.edge_array
    
    def segment(self):
        # Returns a connected components labeling of drivable areas
        return label(np.any(self.edge_array, axis=2))
    


if __name__ == '__main__':
    grid = np.ones((100, 100))
    grid[:, 20:30] = -1
    grid[:, 70:80] = -1
    grid[:, 30:70] = 1
    grid[60:70, 20:30] = 1
    grid[15:25, 70:80] = 1
    grid[30:60, 40:60] = 0
    
    space = OccSpace(grid)
    edges = space.generate_mesh_grid(20)

    fig, ax = plt.subplots(3, 2)
    ax[0,0].imshow(space.segment()[0])
    ax[1,0].imshow(edges[:, :, 0])
    ax[2,0].imshow(edges[:, :, 1])

    eroded = space.erode_mesh()
    ax[1,1].imshow(eroded[:, :, 0])
    ax[2,1].imshow(eroded[:, :, 1])
    ax[0,1].imshow(space.segment()[0])
    plt.show()
