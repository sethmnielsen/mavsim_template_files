import numpy as np
import sys
sys.path.append('..')
from chap11.dubins_parameters import dubins_parameters
from message_types.msg_path import msg_path

class path_manager:
    def __init__(self):
        # message sent to path follower
        self.path = msg_path()
        # pointers to previous, current, and next waypoints
        self.ptr_prev = 0
        self.ptr_current = 1
        self.ptr_next = 2
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones(3)
        self.halfspace_r = np.inf * np.ones(3)
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        self.dubins_path = dubins_parameters()

    def update(self, waypoints, radius, state):
        # this flag is set for one time step to signal a redraw in the viewer
        if self.path.flag_path_changed == True:
            self.num_waypoints = waypoints.num_waypoints
            self.flag_need_new_waypoints = False
            self.initialize_pointers()
            waypoints.flag_waypoints_changed = False
            self.manager_state = 1
            
        if waypoints.num_waypoints == 0:
            waypoints.flag_manager_requests_waypoints = True
        else:
            if waypoints.type == 'straight_line':
                self.line_manager(waypoints, state)
            elif waypoints.type == 'fillet':
                self.fillet_manager(waypoints, radius, state)
            elif waypoints.type == 'dubins':
                self.dubins_manager(waypoints, radius, state)
            else:
                print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def line_manager(self, waypoints, state):
        w_prev = waypoints.ned[self.ptr_prev]
        wi     = waypoints.ned[self.ptr_current]
        w_next = waypoints.ned[self.ptr_next]
        if np.all(np.isinf(self.halfspace_n)):

            q_prev = wi - w_prev
            q_prev /= np.linalg.norm(w_next - wi)
            qi = w_next - wi
            qi /= np.linalg.norm(qi)

            n = q_prev + qi
            n /= np.linalg.norm(n)
            
            self.halfspace_r = wi
            self.halfspace_n = n

        p = np.array([state.pn, state.pe, -state.h])

        if self.inHalfSpace(p):
            self.increment_pointers()
            self.path.flag_path_changed = True
            self.path.line_origin = waypoints.ned[self.ptr_prev]
            self.path.line_direction = w_next - wi
            self.halfspace_n = np.inf * np.ones(3)
        
    def fillet_manager(self, waypoints, radius, state):
        pass
        
    def dubins_manager(self, waypoints, radius, state):
        pass

    def initialize_pointers(self):
        self.ptr_prev = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.manager_state = 1

    def increment_pointers(self):
        self.ptr_prev += 1
        self.ptr_current += 1
        self.ptr_next += 1

        if self.ptr_prev >= self.num_waypoints:
            self.ptr_prev = 0
        if self.ptr_current >= self.num_waypoints:
            self.ptr_current = 0
        if self.ptr_next >= self.num_waypoints:
            self.ptr_next = 0
        
    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r) @ self.halfspace_n >= 0:
            return True
        else:
            return False

