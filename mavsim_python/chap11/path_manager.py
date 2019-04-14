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
        self.ptrs_updated = True
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones(3)
        self.halfspace_r = np.inf * np.ones(3)
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        self.dubins_path = dubins_parameters()
        self.state_changed = True

    def update(self, waypoints, radius, state):
        # this flag is set for one time step to signal a redraw in the viewer
        if waypoints.flag_waypoints_changed == True:
            self.num_waypoints = waypoints.num_waypoints
            self.flag_need_new_waypoints = False
            self.initialize_pointers()
            waypoints.flag_waypoints_changed = False
            self.manager_state = 1
        
        if self.path.flag_path_changed:
            self.path.flag_path_changed = False
            
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
        p = np.array([state.pn, state.pe, -state.h])
        w_prev = waypoints.ned[self.ptr_prev]
        wi     = waypoints.ned[self.ptr_current]
        w_next = waypoints.ned[self.ptr_next]
        
        q_prev = wi - w_prev
        q_prev /= np.linalg.norm(wi - w_prev)
        qi = w_next - wi
        qi /= np.linalg.norm(qi)

        n = q_prev + qi
        n /= np.linalg.norm(n)
        
        self.halfspace_r = wi
        self.halfspace_n = n

        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed[self.ptr_current]
        self.path.line_origin = w_prev
        self.path.line_direction = q_prev

        if self.inHalfSpace(p):
            self.increment_pointers()
            self.path.flag_path_changed = True
            self.path.line_origin = waypoints.ned[self.ptr_prev]
            self.path.line_direction = qi
        
    def fillet_manager(self, waypoints, radius, state):
        p = np.array([state.pn, state.pe, -state.h])
        w_prev = waypoints.ned[self.ptr_prev]
        wi     = waypoints.ned[self.ptr_current]
        w_next = waypoints.ned[self.ptr_next]

        q_prev = wi - w_prev
        q_prev /= np.linalg.norm(wi - w_prev)
        qi = w_next - wi
        qi /= np.linalg.norm(qi)
        
        var_phi = np.arccos(-q_prev @ qi)

        if self.manager_state == 1:
            self.path.flag_path_changed = self.state_changed
            self.state_changed = False
            self.path.type = 'line'
            self.path.line_origin = w_prev
            self.path.line_direction = q_prev
            self.path.airspeed = waypoints.airspeed[self.ptr_current]

            z = wi - (radius/np.tan(var_phi/2))*q_prev
            self.halfspace_r = z
            self.halfspace_n = q_prev

            switch = self.inHalfSpace(p)
            if switch:
                self.manager_state = 2
                self.state_changed = True

        elif self.manager_state == 2:
            self.path.flag_path_changed = self.state_changed
            self.state_changed = False
            qn = q_prev - qi
            qn /= np.linalg.norm(qn)
            c = wi - (radius/np.sin(var_phi/2.0))*qn
            lam = np.sign(q_prev[0]*qi[1] - q_prev[1]*qi[0])

            self.path.type = 'orbit'
            self.path.orbit_center = c
            self.path.orbit_radius = radius
            self.path.airspeed = waypoints.airspeed[self.ptr_current]
            if lam > 0:
                self.path.orbit_direction = 'CW'
            else:
                self.path.orbit_direction = 'CCW'

            z = wi + (radius/np.tan(var_phi/2))*qi
            self.halfspace_r = z
            self.halfspace_n = qi

            if self.inHalfSpace(p):
                self.increment_pointers()
                self.manager_state = 1
                self.state_changed = True

    def dubins_manager(self, waypoints, radius, state):
        p = np.array([state.pn, state.pe, -state.h])
        self.path.airspeed = waypoints.airspeed[self.ptr_current]

        if self.ptrs_updated:
            self.ptrs_updated = False
            ps = waypoints.ned[self.ptr_prev]
            pe = waypoints.ned[self.ptr_current]
            chis = waypoints.course[self.ptr_previous]
            chie = waypoints.course[self.ptr_current]
            self.dubins_path.update(ps, chis, pe, chie, radius)
        
        if self.manager_state == 1:
            self.path.flag_path_changed = self.state_changed
            self.state_changed = False
            self.path.type = 'orbit'
            self.path.orbit_center = self.dubins_path.center_s
            self.path.orbit_radius = radius
            if self.dubins_path.dir_s > 0:
                self.path.orbit_direction = 'CW'
            else:
                self.path.orbit_direction = 'CCW'
            self.check_for_switchstate(self.dubins_path.r1, self.dubins_path.n1, p)
        elif self.manager_state == 2:
            self.check_for_switchstate(self.dubins_path.r1, self.dubins_path.n1, p)

        elif self.manager_state == 3:
            self.path.flag_path_changed = self.state_changed
            self.state_changed = False
            self.path.type = 'line'
            self.path.line_origin = self.dubins_path.r1
            self.path.line_direction = self.dubins_path.n1
            self.check_for_switchstate(self.dubins_path.r2, self.dubins_path.n1, p)
        elif self.manager_state == 4:
            self.path.flag_path_changed = self.state_changed
            self.state_changed = False
            self.path.type = 'orbit'
            self.path.orbit_center = self.dubins_path.center_e
            if self.dubins_path.dir_e > 0:
                self.path.orbit_direction = 'CW'
            else:
                self.path.orbit_direction = 'CCW'
            self.check_for_switchstate(self.dubins_path.r3, self.dubins_path.n3, p)
        elif self.manager_state == 5:
            self.path.flag_path_changed = self.state_changed
            self.state_changed = False
            if self.check_for_switchstate(self.dubins_path.r3, self.dubins_path.n3, p):
                self.increment_pointers()


    def check_for_switchstate(self, r, n, p):
        self.halfspace_r = r
        self.halfspace_n = n
        if self.inHalfSpace(p):
            self.manager_state += 1
            if self.manager_state > 5:
                self.manager_state = 1
            self.state_changed = True
            return True
        
        return False

    def initialize_pointers(self):
        self.ptr_prev = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.manager_state = 1
        self.ptrs_updated = True

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
        self.ptrs_updated = True
        
    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r) @ self.halfspace_n >= 0:
            return True
        else:
            return False

