import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv

home = os.path.expanduser("~")

class Map:
    size = [1715, 881]
    data = plt.imread(home + '/catkin_ws/src/roboat_social_simulator/config/maps/herengracht_3_way.png')
    origin = [-78, -40, 0.0]
    resolution = 0.081


class OccupancyGrid():
  """
  Occupancy grid class for capturing static object information.
  This occupancy grid is aligned with the Cartesian coordinate frame: 
    index 0: x-axis
    index 1: y-axis
  """

  def __init__(self):
    self.gridmap = None
    self.resolution = None
    self.map_size = None
    self.center = np.array([0.0, 0.0])
    
  def getIdx(self, pos_x, pos_y):
    """
    Get indices of position. 
    pos_x and pos_y are the positions w.r.t. the center of the map.
    """
    idx_x = int((pos_x + float(self.map_size[0]) / 2.0) / self.resolution)
    idx_y = int((pos_y + float(self.map_size[1]) / 2.0) / self.resolution)
    # modified
    #idx_x = int(pos_x  / self.resolution)
    #idx_y = int(pos_y / self.resolution)
    
    # Projecting index on map if out of bounds
    idx_x = max(0, min(idx_x, self.map_size[0] / self.resolution))
    idx_y = max(0, min(idx_y, self.map_size[1] / self.resolution))
    
    return idx_x, idx_y
    
  
  def getSubmapByIndices(self, center_idx_x, center_idx_y, span_x, span_y):
    """
    Extract a submap of span (span_x, span_y) around 
    center index (center_idx_x, center_idx_y)
    """
    debug_info = {}
    start_idx_x = max(0, int(center_idx_x - np.floor(span_x / 2)))
    start_idx_y = max(0, int(center_idx_y - np.floor(span_y / 2)))
    
    # Compute end indices (assure size of submap is correct, if out pf bounds)
    max_idx_x = self.gridmap.shape[0] - 1
    max_idx_y = self.gridmap.shape[1] - 1
    
    end_idx_x = start_idx_x + span_x
    if end_idx_x > max_idx_x:
      end_idx_x = max_idx_x
      start_idx_x = end_idx_x - span_x
    end_idx_y = start_idx_y + span_y
    if end_idx_y > max_idx_y:
      end_idx_y = max_idx_y
      start_idx_y = end_idx_y - span_y
    
    # Collect debug information
    debug_info["start_x"] = start_idx_x
    debug_info["start_y"] = start_idx_y
    debug_info["end_x"] = end_idx_x
    debug_info["end_y"] = end_idx_y
    
    return self.gridmap[start_idx_x:end_idx_x, start_idx_y:end_idx_y], debug_info
    
  def getSubmapByCoords(self, center_pos_x, center_pos_y, size_x, size_y):
    """
    Get submap around specified coordinates. 
    The sizes in x and y direction are within the same coordinate frame as the center coordinates.
    """
    center_idx_x, center_idx_y = self.getIdx(center_pos_x, center_pos_y)
    span_x = int(np.ceil(size_x / self.resolution))
    span_y = int(np.ceil(size_y / self.resolution))
    
    return self.getSubmapByIndices(center_idx_x, center_idx_y, span_x, span_y)[0]

  def getFrontSubmap(self, center, velocity, size_x, size_y):
    """
    Get submap around specified coordinates.
    The sizes in x and y direction are within the same coordinate frame as the center coordinates.
    """
    center_idx_x, center_idx_y = self.getIdx(center[0], center[1])
    span_x = int(np.ceil(size_x / self.resolution))
    span_y = int(np.ceil(size_y / self.resolution))

    if velocity[0] > 0.1 :
      center_idx_x += span_x
    elif velocity[0] < -0.1:
      center_idx_x -= span_x

    return self.getSubmapByIndices(center_idx_x, center_idx_y, span_x, span_y)[0]

  def getFrontSubmapByIndices(self, center_idx_x, center_idx_y, span_x, span_y):
    """
    Extract a submap of span (span_x, span_y) around
    center index (center_idx_x, center_idx_y)
    """
    debug_info = {}
    start_idx_x = max(0, int(center_idx_x ))
    start_idx_y = max(0, int(center_idx_y ))

    # Compute end indices (assure size of submap is correct, if out pf bounds)
    max_idx_x = self.gridmap.shape[0] - 1
    max_idx_y = self.gridmap.shape[1] - 1

    end_idx_x = start_idx_x + 2*span_x
    if end_idx_x > max_idx_x:
      end_idx_x = max_idx_x
      start_idx_x = end_idx_x - span_x
    end_idx_y = start_idx_y + span_y
    if end_idx_y > max_idx_y:
      end_idx_y = max_idx_y
      start_idx_y = end_idx_y - span_y

    return self.gridmap[start_idx_x:end_idx_x, start_idx_y:end_idx_y], debug_info

  def getFrontSubmapByCoords(self, center_pos_x, center_pos_y, size_x, size_y, grid_map):
    """
    Get submap around specified coordinates.
    The sizes in x and y direction are within the same coordinate frame as the center coordinates.
    """
    center_idx_x, center_idx_y = self.getIdx(center_pos_x, center_pos_y)
    span_x = int(np.ceil(size_x / self.resolution))
    span_y = int(np.ceil(size_y / self.resolution))
    grid = np.zeros((span_x,span_y))

    start_idx_x = max(0, int(center_idx_x))
    start_idx_y = max(0, int(center_idx_y-span_y/2))

    # Compute end indices (assure size of submap is correct, if out pf bounds)
    max_idx_x = self.gridmap.shape[0] - 1
    max_idx_y = self.gridmap.shape[1] - 1

    end_idx_x = start_idx_x + span_x
    end_idx_y = start_idx_y + span_y
    if end_idx_x > max_idx_x:
      end_idx_x = max_idx_x
    if end_idx_y > max_idx_y:
      end_idx_y = max_idx_y
    dx = end_idx_x-start_idx_x
    dy = min(0,span_y/2-start_idx_y)
    if start_idx_y+span_y<max_idx_y:
      dy_end = span_y
    else:
      dy_end = max_idx_y-start_idx_y

    print(start_idx_x, end_idx_x, start_idx_y, end_idx_y)
    print(grid_map.shape)

    grid[0:dx,dy:dy_end] = grid_map[start_idx_x:end_idx_x,start_idx_y:end_idx_y]
    return grid

  def plot_map(self, map, ax):
    ax.imshow(map.data,
             extent = (map.origin[0], map.origin[0] + map.size[0] * map.resolution,
                       map.origin[1], map.origin[1] + map.size[1] * map.resolution),
             cmap='gray_r')
    ax.set_facecolor('#262626')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.yaxis.set_label_coords(-0.08, .5)
    ax.xaxis.set_label_coords(0.5, -0.09)


def main():
    map = Map()
    grid = OccupancyGrid()
    grid.resolution = map.resolution
    grid.map_size = map.size
    grid.center = map.origin
    grid.gridmap = map.data[:,:,0]

    pos_x = 0
    pos_y = 0

    size_x = 50
    size_y = 50

    idx, idy = grid.getIdx(80,80)

    #print(idx, idy)
    #print(grid.gridmap.shape)

    #patch_ = grid.getFrontSubmapByCoords(pos_x, pos_y, size_x, size_y, grid.gridmap)
    patch_, __= grid.getSubmapByIndices(idx, idy, size_x, size_y)

    test = grid.getSubmapByCoords(pos_x, pos_y, size_x, size_y)

    print(test.shape)

    fig, ax = plt.subplots()
    #ax.imshow(patch_, cmap='gray_r')
    #grid.plot_map(map, ax)
    ax.imshow(test, cmap='gray_r')
    plt.show()

if __name__ == "__main__":
    main()