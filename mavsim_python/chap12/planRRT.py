import numpy as np
from message_types.msg_waypoints import msg_waypoints

class Node():
    def __init__(self, ned=[0,0,0], parentIndex=0, cost=0, goalFlag=0):
        self.ned = np.array(ned)
        self.parentIndex = parentIndex
        self.cost = cost
        self.goalFlag = goalFlag
    
class planRRT():
    def __init__(self):
        self.waypoints = msg_waypoints()
        self.segmentLength = 300 # standard length of path segments

    def planPath(self, wpp_start, wpp_end, map):
        # desired down position is down position of end node
        self.pd = wpp_end[2]

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, parentIndex, cost, connectsToGoalFlag
        start_node = Node([wpp_start[0], wpp_start[1], self.pd], parentIndex=-1)
        end_node = Node([wpp_end[0], wpp_end[1], self.pd])

        # establish tree starting with the start node
        tree = np.array([start_node])

        # check to see if start_node connects directly to end_node
        if ((np.linalg.norm(start_node.ned - end_node.ned) < self.segmentLength ) 
            and not self.collision(start_node, end_node, map)):
            self.waypoints.ned = np.copy(end_node.ned)
        else:
            numPaths = 0
            while numPaths < 3:
                tree, flag = self.extendTree(tree, end_node, map)
                numPaths = numPaths + flag


        # find path with minimum cost to end_node
        path = self.findMinimumPath(tree, end_node)
        return self.smoothPath(path, map)

    def generateRandomPoint(self, map):
        limit = map.city_width
        pt = np.random.uniform(0, limit, 2)
        return pt

    def collision(self, start_node, end_node, map):
        dx = 5  # resolution of checking for collisions [meters]
        vec = end_node.ned - start_node.ned
        nstar = vec/np.linalg.norm(vec)

        size = self.segmentLength/dx
        nums = np.arange(size)
        pts = np.array([nums,nums,nums]).T * nstar + start_node.ned
        for i in range(map.num_city_blocks):
            inside_x = (pts[:,0] > map.collision_range[i,0]) \
                & (pts[:, 0] > map.collision_range[i, 1])

            inside_y = (pts[:, 1] > map.collision_range[i, 0]) \
                & (pts[:, 1] > map.collision_range[i, 1])

            indsx = np.nonzero(inside_x)[0]
            indsy = np.nonzero(inside_y)[0]
            ind_collision = np.intersect1d(indsx, indsy)
            if ind_collision.size:
                return True

        return False


    def pointsAlongPath(self, start_node, end_node, Del):
        pass

    def createNewNode(self, tree, pt):
        # first find closest node to pt
        node = Node()
        min_dist = -1
        index = 0
        vec = np.zeros(2)  # vector from closest node to pt
        for i in range(tree.size):
            vec = np.array([pt[0]-tree[i].ned[0], pt[1]-tree[i].ned[1]])
            dist = np.linalg.norm(vec)
            if min_dist == -1 or dist < min_dist:
                min_dist = dist
                node = tree[i]
                index = i

        # create new node 1 segmentLength in direction of pt from closest node
        L = min_dist
        L_ratio = self.segmentLength/L
        xstar = vec[0] * L_ratio + node.ned[0]
        ystar = vec[1] * L_ratio + node.ned[1]
        vstar = Node([xstar, ystar, self.pd], index)
        return vstar

    def extendTree(self, tree, end_node, map):
        # node format: N, E, D, cost, parentIndex, connectsToGoalFlag
        successful = False
        while not successful:
            pt = self.generateRandomPoint(map)
            vstar = self.createNewNode(tree, pt)
            if not self.collision(tree[vstar.parentIndex], vstar, map):
                vplus = np.array([vstar[0], vstar[1], self.pd, 0, vstar_index)
        

    def findMinimumPath(self, tree, end_node):
        pass

    def smoothPath(self, path, map):
        pass

