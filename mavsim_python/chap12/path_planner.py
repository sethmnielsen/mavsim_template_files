# path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - BGM
import numpy as np
import sys
sys.path.append('..')
from message_types.msg_waypoints import msg_waypoints
from chap12.planRRT import planRRT

class path_planner:
    def __init__(self):
        # waypoints definition
        self.waypoints = msg_waypoints()
        self.rrt = planRRT()

    def update(self, map, state):
        # planner_flag = 1  # return simple waypoint path
        # planner_flag = 2  # return dubins waypoint path
        # planner_flag = 3  # plan path through city using straight-line RRT
        planner_flag = 4  # plan path through city using dubins RRT
        if planner_flag == 1:
            self.waypoints.type = 'fillet'
            self.waypoints.num_waypoints = 4
            Va = 25
            self.waypoints.ned[:self.waypoints.num_waypoints] \
                = np.array([[0, 0, -100],
                            [1000, 0, -100],
                            [0, 1000, -100],
                            [1000, 1000, -100]])
            self.waypoints.airspeed[:self.waypoints.num_waypoints] \
                = np.array([Va, Va, Va, Va])
        elif planner_flag == 2:
            self.waypoints.type = 'dubins'
            self.waypoints.num_waypoints = 4
            Va = 25
            self.waypoints.ned[:self.waypoints.num_waypoints] \
                = np.array([[0, 0, -100],
                            [1000, 0, -100],
                            [0, 1000, -100],
                            [1000, 1000, -100]])
            self.waypoints.airspeed[:self.waypoints.num_waypoints] \
                = np.array([Va, Va, Va, Va])
            self.waypoints.course[:self.waypoints.num_waypoints] \
                = np.array([np.radians(0),
                             np.radians(45),
                             np.radians(45),
                             np.radians(-135)])
        elif planner_flag == 3:
            self.waypoints.type = 'fillet'
            wp_start = np.array([state.pn, state.pe, -state.h])
            wp_end = np.array([map.city_width, map.city_width, -state.h])

            waypoints = self.rrt.planPath(wp_start, wp_end, map)
            self.waypoints.ned = waypoints.ned
            self.waypoints.airspeed = waypoints.airspeed
            self.waypoints.num_waypoints = waypoints.num_waypoints

        elif planner_flag == 4:
            self.waypoints.type = 'dubins'
            wp_start = np.array([state.pn, state.pe, -state.h])
            wp_end = np.array([map.city_width, map.city_width, -state.h])

            waypoints = self.rrt.planPath(wp_start, wp_end, map, self.waypoints.type)
            self.waypoints.ned = waypoints.ned
            self.waypoints.course = waypoints.course
            self.waypoints.airspeed = waypoints.airspeed
            self.waypoints.num_waypoints = waypoints.num_waypoints

        else:
            print("Error in Path Planner: Undefined planner type.")

        return self.waypoints
