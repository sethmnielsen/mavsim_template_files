import numpy as np
from message_types.msg_waypoints import msg_waypoints
import parameters.planner_parameters as PLAN

class Node():
    def __init__(self, ned=[0,0,0], parent_id=0, cost=0, connectToGoalFlag=0, 
                 isGoal=0):
        self.ned = np.array(ned)
        self.parent_id = parent_id
        self.cost = cost
        self.connectToGoalFlag = connectToGoalFlag
        self.isGoal = isGoal

    def copy(self):
        new_node = Node()
        new_node.ned = np.array(self.ned)
        new_node.parent_id = self.parent_id
        new_node.cost = self.cost
        new_node.connectToGoalFlag = self.connectToGoalFlag
        new_node.isGoal = self.isGoal
        return new_node

    
class planRRT():
    def __init__(self):
        self.segmentLength = 400 # standard length of path segments
        self.margin = 20 # desired clearance between mav and obstacles
        self.dx = 5  # resolution of checking for collisions [meters]

    def planPath(self, wp_start, wp_end, map, wp_type="fillet"):
        # get buildings that are safe (short enough) for mav to fly over
        self.bldgs_tall = np.array(np.nonzero(map.bldg_heights>(100 - self.margin))).T
        
        # 2d array of points with [low, high] values for collision range
        window = np.ones(map.num_city_blocks) * map.building_width/2
        self.collision_range = np.array([map.building_north - (window + self.margin),
                                         map.building_north + (window + self.margin)]).T

        # desired down position is down position of end node
        self.pd = wp_end[2]
        
        # specify start and end nodes from wp_start and wp_end
        # format: N, E, D, parent_id, cost, connectsToGoalFlag
        start_node = Node([wp_start[0], wp_start[1], self.pd], parent_id=-1)
        end_node = Node([wp_end[0], wp_end[1], self.pd], isGoal=1)

        # establish tree starting with the start node
        tree = np.array([start_node])

        # check to see if start_node connects directly to end_node
        dist_end = np.linalg.norm(start_node.ned - end_node.ned)
        if (dist_end < self.segmentLength) \
            and not self.collision(start_node, end_node, map):
            end_node.cost = dist_end
            end_node.parent_id = 0
            start_node.connectToGoalFlag = 1
            tree = np.append(tree, end_node)
        else:
            numPaths = 0
            while numPaths < 3:
                tree, flag = self.extendTree(tree, end_node, map)
                numPaths = numPaths + flag


        # find path (array of Nodes) with minimum cost to end_node
        paths = self.create_paths(tree, end_node, numPaths)
        minpath = self.findMinimumPath(paths)  # ndarray of Nodes
        minpath = self.smoothPath(minpath, map) # msg_waypoints
        waypoints = self.createWaypoints(minpath, wp_type)
        return waypoints

    def generateRandomPoint(self, map):
        limit = map.city_width
        pt = np.array([0, 0, self.pd])
        pt[:2] = np.random.uniform(0, limit, 2)
        return pt

    def collision(self, start_node, end_node, map):
        vec = end_node.ned - start_node.ned
        vec_norm = np.linalg.norm(vec)
        n_vec = vec/vec_norm # normalized vector from start to end node

        size = vec_norm/self.dx
        nums = np.arange(size)*self.dx
        pts = np.array([nums,nums,nums]).T * n_vec + start_node.ned
        pts = np.vstack((pts, end_node.ned))
        inds_n = np.array([], dtype=int)
        inds_e = np.array([], dtype=int)
        for low, high in self.collision_range:
            inside_n = np.nonzero( ( pts[:,0] > low ) & ( pts[:,0] < high ) )[0]
            inside_e = np.nonzero( ( pts[:,1] > low ) & ( pts[:,1] < high ) )[0]
            inds_n = np.append(inds_n, inside_n.astype(int))
            inds_e = np.append(inds_e, inside_e.astype(int))

        colls = np.intersect1d(inds_n, inds_e)
        if colls.size == 0:
            return False

        coll_pts = pts[colls]
        nearest_bldg = np.zeros(2, dtype=int)
        for coll_pt in coll_pts:
            nearest_bldg[0] = np.argmin(np.abs(map.building_north - coll_pt[0]))
            nearest_bldg[1] = np.argmin(np.abs(map.building_east  - coll_pt[1]))
            for bldg in self.bldgs_tall:
                if np.array_equal(bldg, nearest_bldg):
                    return True

        return False        


    def createNewNode(self, tree, pt):
        # first find closest node to pt
        vstar = Node()
        min_dist = -1
        index = 0
        close_vec = np.zeros(3)  # vector from closest node to pt
        for i in range(tree.size):
            # vec = np.array([pt[0]-tree[i].ned[0], pt[1]-tree[i].ned[1]])
            vec = pt - tree[i].ned
            dist = np.linalg.norm(vec)
            if min_dist == -1 or dist < min_dist:
                min_dist = dist
                vstar = tree[i]
                index = i
                close_vec = vec

        # create new node 1 segmentLength in direction of pt from closest node
        L = min_dist
        L_ratio = self.segmentLength/L
        xplus = close_vec[0] * L_ratio + vstar.ned[0]
        yplus = close_vec[1] * L_ratio + vstar.ned[1]
        vplus = Node([xplus, yplus, self.pd], parent_id=index, cost=self.segmentLength)
        return vplus

    def extendTree(self, tree, end_node, map):
        while 1:
            pt = self.generateRandomPoint(map)
            vplus = self.createNewNode(tree, pt)
            if not self.collision(tree[vplus.parent_id], vplus, map):
                vplus.cost = self.segmentLength
                tree = np.append(tree, vplus)
                
                dist_end = np.linalg.norm(end_node.ned - vplus.ned)
                if dist_end < self.segmentLength and \
                           not self.collision(vplus, end_node, map):
                    vplus.connectToGoalFlag = 1
                    goal = end_node.copy()
                    goal.parent_id = tree.size-1
                    goal.cost = dist_end
                    tree = np.append(tree, goal)
                
                return tree, vplus.connectToGoalFlag

    def findMinimumPath(self, paths):
        cost_sums = np.array([0.]*paths.size)
        k = 0
        for k in range(paths.size):
            for node in paths[k]:
                cost_sums[k] += node.cost

        min_ind = np.argmin(cost_sums)
        minpath = paths[np.argmin(cost_sums)]
        return minpath
            
    def create_paths(self, tree, end_node, numPaths):
        paths = np.array([end_node]*numPaths)
        k = 0
        for node_ in tree:
            if node_.isGoal == 1:
                paths[k].parent_id = node_.parent_id
                node = tree[node_.parent_id]
                while node.parent_id != -1:
                    paths[k] = np.append(paths[k], node)
                    node = tree[node.parent_id]
                paths[k] = np.append(paths[k], tree[0]) # append start node
                k += 1

        for i in range(paths.size):
            paths[i] = np.flip(paths[i])
        
        return paths


    def smoothPath(self, path, map):
        path_smooth = np.array([path[0]])
        i = 0
        j = 1
        point_added = False
        while j < path.size-1:
            point_added = False
            if self.collision(path[i], path[j+1], map):
                if self.distance(path[i], path[j]) > PLAN.R_min:
                    path_smooth = np.append(path_smooth, path[j])
                    i = j
                    point_added = True
                else:
                    for k in range(1, j-i-2): # from 3rd-to-last node to 3rd node
                        if self.distance(path[i], path[j-k]) > PLAN.R_min:
                            path_smooth = np.append(path[i], path[j-k])
                            i = j
                            point_added = True
                            break
                    if not point_added:
                        path_smooth = np.append(path[i], path[i+1])

            j += 1
        
        path_smooth = np.append(path_smooth, path[-1])
        for k in range(path_smooth.size-1):
            path_smooth[k+1].cost = np.linalg.norm(path[k+1].ned - path[k].ned)                        
        
        return path_smooth
    
    def distance(self, node1, node2):
        return np.linalg.norm(node2.ned - node1.ned)

    def createWaypoints(self, path, wp_type):
        waypoints = msg_waypoints()
        neds = np.zeros((path.size,3))
        courses = np.zeros(path.size)
        for i in range(path.size):
            neds[i] = path[i].ned
            if wp_type == 'dubins':
                if i == path.size-1:
                    vec = path[i].ned - path[i-1].ned
                else:
                    vec = path[i+1].ned - path[i].ned
                courses[i] = np.arctan2(vec[0], vec[1])

        waypoints.add_waypoints(wp_type, neds, courses)
        return waypoints
