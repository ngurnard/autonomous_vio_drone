from audioop import reverse
from math import dist
import numpy as np
from scipy.sparse import csc_matrix
from scipy.sparse.linalg import spsolve
# from scipy.spatial.distance import pdist, squareform # this is something I added, idk if the autograder takes it
from scipy.spatial import cKDTree # this is something I added

# Uncomment for the autograder!
from .graph_search import graph_search
from flightsim.world import World

class WorldTraj(object):
    """
    Implements a minimum snap trajectory.
    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """
        self.start = start # used in update when hovering

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!

        # Use these for the physical lab
        # self.resolution = 0.65*np.array([0.25, 0.25, 0.25])
        # self.margin = 0.15

        # Use these for the simulator
        # self.resolution = 0.7*np.array([0.25, 0.25, 0.25])
        # self.margin = 0.3
        # self.resolution = 0.84*np.array([0.25, 0.25, 0.25])
        # self.margin = 0.5
        self.resolution = np.array([0.25, 0.25, 0.25]) # default
        self.margin = 0.5 # default

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        ## Idea for future - prune a path that checks whether there are obstacles in between pts using occupancy_map.py
        self.points = self.path # I need to prune this 

        """
        # RAMER-DOUGLASS-PEUKER ALOGITHM FOR PRUNING THE PATH
        self.points = self.rdp(self.points, epsilon=0.25) # remove colinear points with ramer-douglass-peukar

        # Now see if there are points too close to the start or the end - if yes, remove them (simulation only)
        thresh_min = 0.64 # threshold to how far apart the points need to be
        while(np.linalg.norm(self.points[0]-self.points[1]) < thresh_min): # while the second points is too close to the first
            self.points = np.delete(self.points, 1, axis = 0)
        while(np.linalg.norm(self.points[-1]-self.points[-2]) < thresh_min): # while the second points is too close to the first
            self.points = np.delete(self.points, -2, axis = 0)

        # # If points are too far apart, add an intermediate point to prevent the speed from getting way too high (simulation only)
        distances = np.sqrt(np.sum(np.diff(self.points, axis=0)**2,axis=1)) # computes the distance between consecutive points
        thresh_max = 2.5 # threshold to how far apart the points need to be
        while (np.max(distances) > thresh_max):
            far_idx = np.where(distances >= thresh_max)
            roll_back = np.roll(self.points, -1, axis = 0)
            midpoints = (roll_back[far_idx[0]]+self.points[far_idx[0]])/2
            self.points = np.insert(self.points, far_idx[0]+1, midpoints, axis=0)
            distances = np.sqrt(np.sum(np.diff(self.points, axis=0)**2,axis=1)) # computes the distance between consecutive points

        # If points are too close together, combine them at their midpoint
        # # print(self.points)
        # tree = cKDTree(self.points)
        # rows_to_fuse = tree.query_pairs(r=thresh_min+.001, output_type='ndarray') # the +0.001 ensures the start and endpoints to not change
        # # print("rows to fuse: ", rows_to_fuse)    
        # for i in reversed(rows_to_fuse):
        #     # print("printing i in the loop: ", i)
        #     replace_idx = i[0] # get the index of the first point
        #     self.points[replace_idx] = (self.points[i[0]] + self.points[i[1]]) / 2 # replace with the midpoint
        #     self.points = np.delete(self.points, i[1], axis = 0)
        # # print("new self.points: ", self.points)

        # ONLY TAKING EVERY THIRD POINT - LAB ONLY
        # self.points = self.points[::3]
        # self.points[-1, :] = goal # don't accidentally remove the goal. IDEALLY WANT BTW 0.64 AND 1.1
        # print("points: \n", self.points, goal)
        """

        
        ## Trying a pruning algorithm that checks iteratively along the trajectory -- raytracing/line-of-sight
        # If a line drawn between 2 consecutive points has no obstacle, connect them
        current_point = 0
        new_points = []
        while current_point != len(self.path)-1: # while not at the last point in the trajectory
            # Find the farthest waypoint that has no collision from current waypoint
            for future_pt in range(current_point + 1, len(self.path)): # checking all of the points after the current point
                if future_pt == len(self.path)-1: # if at the last point
                    current_point = future_pt # this will break the while
                    new_points.append(self.path[-1]) # append the goal state
                    break # break the FOR to the break the WHILE
                else: # check for a collision
                    # If there is a collision then add the previous non collision waypoint[index] to self.points
                    vector_list = [self.path[current_point], self.path[future_pt]] # a vector between the current point and the future point 
                    if world.path_collisions(np.array(vector_list), self.margin).size != 0: # if there is a collision
                        if current_point + 1 == future_pt: # if the current point is right before the future point
                            continue # continue the FOR. This prevents duplicate points
                        else:
                            new_points.append(self.path[future_pt-1]) # append the previous NON-COLLISION point
                            current_point = future_pt - 1 # jump to the point where there was no collision instead of inspecting every single point
                            break # break the FOR
                    else:
                        continue # continue the FOR if no collision
        self.points = np.array(new_points)
        self.points = np.insert(self.points, 0, 0, axis=0) # make sure you start at the start!
        self.points[0] = start
        print(self.points[0], type(self.points[0]), self.points.shape, self.points) 
        

        ## Implementing the minimum snap trajectory
        ## Find needed parameters
        #  time allocation
        segments = len(self.points) - 1 # the number of segments
        # self.avg_velocity = 2.45 # m/s This number can be adjusted! Using 2.5 because that is what my controller is tuned to
        self.avg_velocity = 1.75 # m/s This number can be adjusted! Using 2.5 because that is what my controller is tuned to
        distances = np.sqrt(np.sum(np.diff(self.points, axis=0)**2,axis=1)) # computes the distance between consecutive points
        self.time = (distances**(1/3)/self.avg_velocity) # get the self.time to each segment
        # big_idx = np.where(distances >= 5) # get the indices where there are long straight aways
        # self.time[big_idx] = distances[big_idx]**(1/4)/self.avg_velocity # make the straight aways faster
        # self.time[0] = distances[0]/self.avg_velocity # give it time to accelerate
        self.time[0] *= 2 # give it time to accelerate 
        # self.time[-1] = distances[-1]**(1/2)/self.avg_velocity # give it time to decellerate
        self.time[-1] *= 1.5 # give it time to decellerate
        self.time = np.insert(self.time, 0, 0) # Make sure self.time0 = 0!
        # print(f"Distances: {distances}, \nTimes: {self.time}")
        self.cum_time = np.cumsum(self.time) # get the cumulative times! this is necessary for final block
        # print(len(self.time), self.time)
        # print("\n\n\n\n\ ................. \n\n\n\n\n")
        # print(self.cum_time)

        # First set up the massive trajectory matrix
        initial_block = np.array([[0,0,0,0,0,0,0,1], # position
                                   [0,0,0,0,0,0,1,0], # velocity
                                   [0,0,0,0,0,2,0,0], # accleration
                                   [0,0,0,0,6,0,0,0]]) # jerk (n-1)th derivative for snap
        traj_matrix = np.block([initial_block, np.zeros((4,(segments-1)*8))]) # this is the first 4 rows of the traj_matrix
        final_block = np.array([[self.time[-1]**7,self.time[-1]**6,self.time[-1]**5,self.time[-1]**4,self.time[-1]**3,self.time[-1]**2,self.time[-1],1], # position
                                    [7*self.time[-1]**6,6*self.time[-1]**5,5*self.time[-1]**4,4*self.time[-1]**3,3*self.time[-1]**2,2*self.time[-1],1,0], # velocity
                                    [42*self.time[-1]**5,30*self.time[-1]**4,20*self.time[-1]**3,12*self.time[-1]**2,6*self.time[-1],2,0,0], # accleration
                                    [210*self.time[-1]**4,120*self.time[-1]**3,60*self.time[-1]**2,24*self.time[-1],6,0,0,0]]) # jerk (n-1)th derivative for snap
        traj_matrix = np.vstack((traj_matrix, np.block([np.zeros((4,(segments-1)*8)), final_block]))) # tack on the rows that correspond to the final point
        for i in range(1, len(self.time)-1):
            intermediate_block = np.array([[self.time[i]**7,self.time[i]**6,self.time[i]**5,self.time[i]**4,self.time[i]**3,self.time[i]**2,self.time[i],1,0,0,0,0,0,0,0,0], # position
                                    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1], # position
                                    [7*self.time[i]**6,6*self.time[i]**5,5*self.time[i]**4,4*self.time[i]**3,3*self.time[i]**2,2*self.time[i],1,0,0,0,0,0,0,0,-1,0], # velocity
                                    [42*self.time[i]**5,30*self.time[i]**4,20*self.time[i]**3,12*self.time[i]**2,6*self.time[i],2,0,0,0,0,0,0,0,-2,0,0], # acceleration
                                    [210*self.time[i]**4,120*self.time[i]**3,60*self.time[i]**2,24*self.time[i],6,0,0,0,0,0,0,0,-6,0,0,0], # jerk
                                    [840*self.time[i]**3,360*self.time[i]**2,120*self.time[i],24,0,0,0,0,0,0,0,-24,0,0,0,0], # snap
                                    [2520*self.time[i]**2,720*self.time[i],120,0,0,0,0,0,0,0,-120,0,0,0,0,0], # crackle
                                    [5040*self.time[i],720,0,0,0,0,0,0,0,-720,0,0,0,0,0,0]]) # pop
            temp = np.block([np.zeros((8,8*(i-1))), intermediate_block, np.zeros((8,8*(segments - i - 1)))])
            traj_matrix = np.vstack((traj_matrix, temp))

        # Now make vector b in Ax = b
        boundary_b_start = np.array([start, # position
                      [0,0,0], # velocity
                      [0,0,0], # acceleration
                      [0,0,0]]) # jerk
        boundary_b_goal = np.array([goal, # position
                            [0,0,0], # velocity
                            [0,0,0], # acceleration
                            [0,0,0]]) # jerk
        boundary_b = np.vstack((boundary_b_start, boundary_b_goal))
        continuity_temp = np.concatenate((self.points, self.points), axis = 1) # first repeat the points once
        continuity_temp = np.concatenate((continuity_temp, np.zeros((len(self.points), 18))), axis = 1) # then concatenate 18 zeros (6 rows of 1x3 zeros)
        continuity_b = continuity_temp.reshape(-1, 3)[8:-8, :] # reshape to get the correct vector size, then take only the intermediate points
        b = np.vstack((boundary_b, continuity_b)) # finally construct the entire b vector

        # Now solve Ax = b using sparse matrices
        sparse_A = csc_matrix(traj_matrix)
        sparse_b = csc_matrix(b)
        self.coefficients = spsolve(sparse_A, sparse_b).toarray()

        # # Here I am finding unit vectors in order to do yaw control - which is something new that I am trying
        # compare = np.roll(self.points, -1, axis = 0) # p_{i+1}
        # numerator = compare - self.points # numerator of the unit vector intialization (p_{i+1} - p_{i})
        # self.unit_vectors = numerator[:-1,:]/distances.reshape(-1,1) # define unit vectors of point segments (n-1 unit vectors)
        # self.normal = np.array([0, 0, 1]) # this is a normal vector to the xy plane in order to do vector projection
        # print("TESTING: ", self.unit_vectors)

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """

        ## Implementation of minimum snap trajectory
        # print("self.cum_time: ", self.cum_time)
        check = np.where(self.cum_time > t) # check if at the end point. Get's indices where cum_time > t
        hover_thresh = 0.225
        yaw_control = False # CHANGE THIS TO TRUE IF YOU WISH TO SEE YAW CONTROL -- HOWEVER, YAW ANGLE WRAP AROUND IS NOT YET SOLVED IN THIS IMPLEMENTATION
        # print("CHECK: \n", check)
        if (t < self.cum_time[-1] and t >= hover_thresh): # if not at the end
            t = t - hover_thresh # account for the hover!!!! This is very important or else the time convention would be wrong (0 at the start of each segment)
            # print("where? \n", np.where(self.time > t))
            idx = check[0][0] # get the index where cum_time > t. Basically your segment number!
            # print("idx: ", idx)
            x        = self.coefficients[8*(idx-1)]*(t-self.cum_time[idx-1])**7 + \
                        self.coefficients[8*(idx-1)+1]*(t-self.cum_time[idx-1])**6 + \
                        self.coefficients[8*(idx-1)+2]*(t-self.cum_time[idx-1])**5 + \
                        self.coefficients[8*(idx-1)+3]*(t-self.cum_time[idx-1])**4 + \
                        self.coefficients[8*(idx-1)+4]*(t-self.cum_time[idx-1])**3 + \
                        self.coefficients[8*(idx-1)+5]*(t-self.cum_time[idx-1])**2 + \
                        self.coefficients[8*(idx-1)+6]*(t-self.cum_time[idx-1]) + \
                        self.coefficients[8*(idx-1)+7] # position which retrieves C_{segment, #}
            x_dot    = 7*self.coefficients[8*(idx-1)]*(t-self.cum_time[idx-1])**6 + \
                        6*self.coefficients[8*(idx-1)+1]*(t-self.cum_time[idx-1])**5 + \
                        5*self.coefficients[8*(idx-1)+2]*(t-self.cum_time[idx-1])**4 + \
                        4*self.coefficients[8*(idx-1)+3]*(t-self.cum_time[idx-1])**3 + \
                        3*self.coefficients[8*(idx-1)+4]*(t-self.cum_time[idx-1])**2 + \
                        2*self.coefficients[8*(idx-1)+5]*(t-self.cum_time[idx-1]) + \
                        self.coefficients[8*(idx-1)+6] # velocity which retrieves C_{segment, #}
            x_ddot   = 42*self.coefficients[8*(idx-1)]*(t-self.cum_time[idx-1])**5 + \
                        30*self.coefficients[8*(idx-1)+1]*(t-self.cum_time[idx-1])**4 + \
                        20*self.coefficients[8*(idx-1)+2]*(t-self.cum_time[idx-1])**3 + \
                        12*self.coefficients[8*(idx-1)+3]*(t-self.cum_time[idx-1])**2 + \
                        6*self.coefficients[8*(idx-1)+4]*(t-self.cum_time[idx-1]) + \
                        2*self.coefficients[8*(idx-1)+5] # acceleration which retrieves C_{segment, #}
            x_dddot  = 210*self.coefficients[8*(idx-1)]*(t-self.cum_time[idx-1])**4 + \
                        120*self.coefficients[8*(idx-1)+1]*(t-self.cum_time[idx-1])**3 + \
                        60*self.coefficients[8*(idx-1)+2]*(t-self.cum_time[idx-1])**2 + \
                        24*self.coefficients[8*(idx-1)+3]*(t-self.cum_time[idx-1]) + \
                        6*self.coefficients[8*(idx-1)+4] # jerk which retrieves C_{segment, #}
            x_ddddot = 840*self.coefficients[8*(idx-1)]*(t-self.cum_time[idx-1])**3 + \
                        360*self.coefficients[8*(idx-1)+1]*(t-self.cum_time[idx-1])**2 + \
                        120*self.coefficients[8*(idx-1)+2]*(t-self.cum_time[idx-1]) + \
                        24*self.coefficients[8*(idx-1)+3] # snap which retrieves C_{segment, #} # snap

            # implement yaw control
            # scalar = np.dot(self.unit_vectors[idx-1], self.normal)/np.linalg.norm(self.normal) # this is the scalar part of the projection of the unit vector of the segment onto the normal vector
            # print("scalar: ", scalar)
            # proj_normal = scalar * self.normal # get the projection onto the normal vector
            # proj_xy = self.unit_vectors[idx-1] - proj_normal
            # yaw = np.arccos(np.clip(np.dot(np.array([0, 1, 0]), proj_xy/np.linalg.norm(proj_xy)), -1.0, 1.0)) # yaw

            if (yaw_control == True):
                scalar = np.dot(x_dot, np.array([0, 0, 1]))/np.linalg.norm(np.array([0, 0, 1])) # this is the scalar part of the projection of the unit vector of the segment onto the normal vector
                proj_normal = scalar * np.array([0, 0, 1]) # get the projection onto the normal vector
                proj_xy = x_dot - proj_normal
                # NOTE: Arccos() is bounded between 0 and pi, which never gives a negative angle, we can get that by using the sign from the cross product
                yaw_sign = (np.cross(np.array([0, 1, 0]), proj_xy)/np.linalg.norm(np.cross(np.array([0, 1, 0]), proj_xy)))[2]
                # print("yaw sign: ", yaw_sign)
                yaw = yaw_sign * np.arccos(np.clip(np.dot(np.array([0, 1, 0]), proj_xy/np.linalg.norm(proj_xy)), -1.0, 1.0)) # yaw
                # print("yaw: ", idx, yaw, proj_xy)
                yaw_dot = 0 # angular rate
            else:
                yaw = 0
                yaw_dot = 0
        elif (t < hover_thresh): # Allow the drone to simply hover for a bit to allow the initial measurement covariances to settle
            idx = check[0][0] # get the index where cum_time > t. Basically your segment number!
            x = self.start # set the position to be the starting point
            x_dot    = np.zeros((3,)) # velocity should be zero when hovering
            x_ddot   = np.zeros((3,)) # acceleration
            x_dddot  =  np.zeros((3,)) # jerk
            x_ddddot =  np.zeros((3,)) # snap
            yaw = 0 # yaw
            yaw_dot = 0 # angular rate 
        else:
            x        = self.points[-1] # set to the end point
            x_dot    = np.zeros((3,))
            x_ddot   =  np.zeros((3,)) # acceleration
            x_dddot  =  np.zeros((3,)) # jerk
            x_ddddot =  np.zeros((3,)) # snap
            yaw = 0 # yaw
            yaw_dot = 0 # angular rate

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output

    def pldist(self, point, start, end):
        """
        Calculates the distance from ``point`` to the line given
        by the points ``start`` and ``end``.
        :param point: a point
        :type point: numpy array
        :param start: a point of the line
        :type start: numpy array
        :param end: another point of the line
        :type end: numpy array
        """
        if np.all(np.equal(start, end)):
            return np.linalg.norm(point - start)

        return np.divide(np.abs(np.linalg.norm(np.cross(end - start, start - point))),np.linalg.norm(end - start))


    def rdp_rec(self, M, epsilon):
        """
        Simplifies a given array of points.
        Recursive version.
        :param M: an array
        :type M: numpy array
        :param epsilon: epsilon in the rdp algorithm
        :type epsilon: float
        :param dist: distance function
        :type dist: function with signature ``f(point, start, end)`` -- see :func:`rdp.pldist`
        """
        dmax = 0.0
        index = -1

        for i in range(1, M.shape[0]):
            d = self.pldist(M[i], M[0], M[-1])

            if d > dmax:
                index = i
                dmax = d

        if dmax > epsilon:
            r1 = self.rdp_rec(M[:index + 1], epsilon)
            r2 = self.rdp_rec(M[index:], epsilon)

            return np.vstack((r1[:-1], r2))
        else:
            return np.vstack((M[0], M[-1]))

    def rdp(self, M, epsilon=0, return_mask=False):
        """
        Simplifies a given array of points using the Ramer-Douglas-Peucker
        algorithm.
        This is a convenience wrapper around `rdp.rdp_rec` that detects 
        if the input is a numpy array
        in order to adapt the output accordingly. This means that
        when it is called using a Python list as argument, a Python
        list is returned, and in case of an invocation using a numpy
        array, a NumPy array is returned.
        
        :param M: a series of points
        :type M: numpy array with shape ``(n,d)`` where ``n`` is the number of points and ``d`` their dimension
        :param epsilon: epsilon in the rdp algorithm
        :type epsilon: float
        :param dist: distance function
        :type dist: function with signature ``f(point, start, end)`` -- see :func:`rdp.pldist`
        :param algo: either ``iter`` for an iterative algorithm or ``rec`` for a recursive algorithm
        :type algo: string
        :param return_mask: return mask instead of simplified array
        :type return_mask: bool
        """

        if return_mask:
            raise NotImplementedError("return_mask=True not supported with algo=\"rec\"")
        algo = self.rdp_rec
            
        if "numpy" in str(type(M)):
            return algo(M, epsilon)

        return algo(np.array(M), epsilon).tolist()
