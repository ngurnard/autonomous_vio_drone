from heapq import heappush, heappop  # Recommended.
import numpy as np
import scipy.spatial # I imported this 

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.
# from occupancy_map import OccupancyMap 

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    # Get an idea of all of the variables
    # print("start index: ", start_index)
    # print("goal_index: ", goal_index)

    # Initialize Neccessary Variables
    if (astar == False):
        heuristic = 0 # Dijkstra's
    else:
        heuristic = np.linalg.norm(np.asarray(start_index) - np.asarray(goal_index)) # A* heuristic

    pq_unvisited = [] # Initialize the priority queue
    start_node = get_node(0, start_index, [None], 0) # a list in the order of [cost, index, [parent], parent_cost]
    heappush(pq_unvisited, start_node) # 
    # print("pq_unvisited: ", pq_unvisited)
    visited = {} # Initialize visited nodes

    while (goal_index not in visited and len(pq_unvisited) != 0):
        # print("I am in the loop!")

        # Get the current point and calculate its info
        current_list = heappop(pq_unvisited)
        current_point = current_list[1] # The index of the current point
        parent = current_list[2].copy() # The parent LIST, a list of tuples of indices of previous parents
        parent_cost = current_list[-1] # the cost up to the current parent 
        
        # print("current_point: ", current_point)
        # print("parent: ", parent, type(parent))
        # print("parent cost: ", parent_cost)


        if (current_point in visited):
            pass # if the current point is already closed, then it is already optimal
        else:
            # Add the current node to the visited dict with its cost (mark as closed)
            visited[current_point] = current_list 
            
            # Compute the neighboring voxels and their distances
            neighbors, neighbors_cost = Get_Neighbors(current_point, resolution, occ_map).get_neighbor_cost()
            # print('neighbors: ', len(neighbors), ": ", neighbors)
            # print('neighbors_cost: ', len(neighbors_cost), ": ", neighbors_cost)

            # Make all the neighbors a node with info, then put it in the priority queue
            for i in range(len(neighbors)):
                parent_copy = parent.copy()
                # print("parent copy: ", parent_copy)
                parent_copy.append(current_point)
                # print("testing append: ", parent_copy)
                if (astar == True):
                    heuristic = np.linalg.norm(np.asarray(neighbors[i]) - np.asarray(goal_index))
                    # print(heuristic)
                neigh_node = get_node(neighbors_cost[i] + parent_cost + heuristic, neighbors[i], parent_copy, parent_cost + neighbors_cost[i]) # [cost, index, parent, parent_cost]
                # print("neigh_node: ", neigh_node)
                heappush(pq_unvisited, neigh_node)

    visited[goal_index][2].append(goal_index)
    # print("goal index info: ", visited[goal_index][3])
    path1 = np.asarray(visited[goal_index][2][1:])
    # print("goal index path: ", path1) # get rid of the None value
    path = occ_map.index_to_metric_center(path1)
    path[0] = start # replace the start with the actual start
    path[-1] = goal # replace the goal with the actual goal
    # print("goal index path new: ", path) # make it make sense on the map
    
    # Return a tuple (path, nodes_expanded)
    # return None, 0
    return (path, len(visited))


class Get_Neighbors:
    """
    A class used to calculate the neighbors and then find their distances
    """

    def __init__(self, current_point, resolution, occ_map): # Organize some data
        self.current_point = current_point 
        self.resolution = resolution
        self.occ_map = occ_map

    def get_neighbors(self):
        """
        Computes the 26 voxels (neighbors) that are around the current point. 
        It then checks if these points are within the map and if they obtruct an object. If True, they are removed.
        Returns a list of only neighbors that are valid -- i.e. within the map and in the free space
        """
        neighbors = [] # initialize the empty neighbor list
        # neighbors_real = [] # intialize the empty neighbors list that takes resolution into account

        # Find the 26 neighboring voxels' coordinates
        # for i in [-self.resolution[0], 0, self.resolution[0]]: # i coordinate
        #     for j in [-self.resolution[1], 0, self.resolution[1]]: # j coordinate
        #         for k in [-self.resolution[2], 0, self.resolution[2]]: # k coordinate
        #             if (i ==0 and j ==0 and k ==0): # if at the same point
        #                 pass # skip the current point
        #             else:
        #                 neighbors.append([self.current_point[0]+i,self.current_point[1]+j,self.current_point[2]+k]) # add the neighbor to the neighbors list

        # Find the 26 neighboring voxels' coordinates
        for i in [-1, 0, 1]: # i coordinate
            for j in [-1, 0, 1]: # j coordinate
                for k in [-1, 0, 1]: # k coordinate
                    if (i ==0 and j ==0 and k ==0): # if at the same point
                        pass # skip the current point
                    else:
                        neighbors.append((self.current_point[0]+i,self.current_point[1]+j,self.current_point[2]+k)) # add the neighbor to the neighbors list

        # print("neighbors: ", len(neighbors), ": ", neighbors)
        # print("neighbors_real: ", len(neighbors_real), ": ", neighbors_real)

        # Find the 26 neighbors using meshgrid instead (faster!)
        rows, cols, depth = np.meshgrid(np.arange(-1, 2, 1), np.arange(-1, 2, 1), np.arange(-1, 2, 1))
        # print("rows: ", rows)
        # print("cols: ", cols)
        # print("depth: ", depth)


        new_neighbors = []

        # check if all of the neighbors are in the map and not in an object
        for voxel in range(len(neighbors)): # Can I speed this up??
            if (self.occ_map.is_occupied_index(neighbors[voxel]) == False):
                new_neighbors.append(neighbors[voxel]) # only save valid neighbors

        # print("New neighbors: ", len(new_neighbors), ": ", new_neighbors)
        # print("New neighbors real: ", len(new_neighbors_real), ": ", new_neighbors_real)

        return new_neighbors#, new_neighbors_real

    def get_neighbor_cost(self):
        """
        Computes the distance (cost) to the computed valid neighboring voxels (self.neighbors). 
        Recommended to pass in the output from .get_self.neighbors()
        Returns ...
        """
        
        # neighbors, neighbors_real = self.get_neighbors()
        neighbors = self.get_neighbors()
        # print("Checking neighbors: ", neighbors)
        # print("Checking neighbors real: ", neighbors_real)
        # neighbor_cost = scipy.spatial.distance.cdist(np.asarray(self.current_point).reshape(1,-1), np.asarray(neighbors_real)).flatten() # calculate distance to all neighbors including resolution
        neighbor_cost = scipy.spatial.distance.cdist(np.asarray(self.current_point).reshape(1,-1), np.asarray(neighbors),'euclidean').flatten() # calculate distance to all neighbors including resolution
        # print('neightbors', neighbors)
        # print('neighbor costs', neighbor_cost)
        # print("Computed cost: ", neighbor_cost)
        #exit()

        return neighbors, neighbor_cost


def get_node(cost, index, parent, parent_cost):
    """
    Simply takes the inputs and returns a list in the following order:
    [cost, index, heuristic, parent, parent_cost, astar]
    Makes code more readable
    """
    return [cost, index, parent, parent_cost]