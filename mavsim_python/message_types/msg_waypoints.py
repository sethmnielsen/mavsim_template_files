"""
msg_waypoints
    - messages type for input to path manager
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:z
        3/26/2019 - RWB
"""
import numpy as np
import parameters.planner_parameters as PLAN

class msg_waypoints:
    def __init__(self):
        # the first two flags are used for interacting with the path planner
        #
        # flag to indicate waypoints recently changed (set by planner)
        self.flag_waypoints_changed = True
        # flag to indicate that the waypoint manager needs new waypoints (set by manager)
        self.flag_manager_requests_waypoints = True

        # type of waypoint following:
        #   - straight line following
        #   - fillets between straight lines
        #   - follow dubins paths
        self.type = 'straight_line'
        # self.type = 'fillet'
        # self.type = 'dubins'
        # maximum number of waypoints. Used to pre-allocate memory/improve efficiency
        self.max_waypoints = 100
        # current number of valid waypoints in memory
        self.num_waypoints = 0
        # [n, e, d] - coordinates of waypoints
        self.ned = np.inf * np.ones((self.max_waypoints, 3))
        # the airspeed that is commanded along the waypoints
        self.airspeed = np.inf * np.ones(self.max_waypoints)
        # the desired course at each waypoint (used only for Dubins paths)
        self.course = np.inf * np.ones(self.max_waypoints)

        # these variables are used by the path planner running cost at each node
        # self.cost = np.inf * np.ones(self.max_waypoints)
        # index of the parent to the node
        # self.parent_idx = np.inf * np.ones(self.max_waypoints)
        # can this node connect to the goal?
        # self.flag_connect_to_goal = 0 * np.ones(self.max_waypoints)

    def add_waypoints(self, wp_type, wp_neds, wp_courses):
        self.ned = wp_neds
        self.type = wp_type
        self.course = wp_courses
        self.flag_waypoints_changed = True
        self.num_waypoints = self.ned.shape[0]
        self.airspeed = np.array([PLAN.Va0]*self.num_waypoints)
        
