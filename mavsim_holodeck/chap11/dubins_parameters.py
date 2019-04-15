# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB

import numpy as np
import sys
sys.path.append('..')


class dubins_parameters:
    def __init__(self):
        self.p_s = np.inf*np.ones(3)  # the start position in re^3
        self.chi_s = np.inf               # the start course angle
        self.p_e = np.inf*np.ones(3)  # the end position in re^3
        self.chi_e = np.inf               # the end course angle
        self.radius = np.inf              # turn radius
        self.length = np.inf              # length of the Dubins path
        self.center_s = np.inf*np.ones(3)  # center of the start circle
        self.dir_s = np.inf                    # direction of the start circle
        self.center_e = np.inf*np.ones(3)  # center of the end circle
        self.dir_e = np.inf              # direction of the end circle
        self.r1 = np.inf*np.ones(3)  # vector in re^3 defining half plane H1
        self.r2 = np.inf*np.ones(3)  # vector in re^3 defining position of half plane H2
        self.r3 = np.inf*np.ones(3)  # vector in re^3 defining position of half plane H3
        self.n1 = np.inf*np.ones(3)  # unit vector in re^3 along straight line path
        self.n3 = np.inf*np.ones(3)  # unit vector defining direction of half plane H3

    def update(self, ps, chis, pe, chie, R):
        ell = np.linalg.norm(ps - pe)
        if ell < 2 * R:
            str1 = 'Error in Dubins Parameters: ' + \
                   'The distance between nodes must be larger than 2R'
            str2 = '\nCurrent dist: {:2f} | needed dist: {:2f}'.format(ell, 2*R)
            raise ValueError(str1 + str2)
        else:
            cxs = np.cos(chis)
            sxs = np.sin(chis)
            cxe = np.cos(chie)
            sxe = np.sin(chie)
            x = R*rotz(np.pi/2)
            y = np.array([cxs,sxs,0])
            c_rs = ps + x @ y
            c_ls = ps + R*rotz(-np.pi/2) @ np.array([cxs,sxs,0])
            c_re = pe + R*rotz(np.pi/2)  @ np.array([cxe,sxe,0])
            c_le = pe + R*rotz(-np.pi/2) @ np.array([cxe,sxe,0])
            
            # compute L1
            theta = np.arctan2(c_re[1]-c_rs[1],c_re[0]-c_rs[0])
            L1 = np.linalg.norm(c_rs-c_re) + \
                    R*mod(2*np.pi+mod(theta-np.pi/2)-mod(chis-np.pi/2)) + \
                    R*mod(2*np.pi+mod(chie-np.pi/2)-mod(theta-np.pi/2))

            # compute L2
            theta = np.arctan2(c_le[1]-c_rs[1],c_le[0]-c_rs[0])
            ell = np.linalg.norm(c_rs - c_le)
            sqrt = np.sqrt(ell**2 - 4*R**2)
            theta2 = theta - (np.pi/2) + np.arcsin(2*R/ell)
            L2 = sqrt + R*mod(2*np.pi+mod(theta-theta2)-mod(chis-np.pi/2)) +\
                          R*mod(2*np.pi+mod(theta2+np.pi)-mod(chie+np.pi/2))

            # compute L3
            theta = np.arctan2(c_re[1]-c_ls[1],c_re[0]-c_ls[0])
            ell = np.linalg.norm(c_ls - c_re)
            sqrt = np.sqrt(ell**2 - 4*R**2)
            theta2 = np.arccos(2*R/ell)
            L3 = sqrt + R*mod(2*np.pi+mod(chis+np.pi/2)-mod(theta+theta2)) + \
                    R*mod(2*np.pi+mod(chie-np.pi/2)-mod(theta+theta2-np.pi))

            # compute L4
            theta = np.arctan2(c_le[1]-c_ls[1],c_le[0]-c_ls[0])
            L4 = np.linalg.norm(c_ls-c_le) + \
                    R*mod(2*np.pi+mod(chis+np.pi/2)-mod(theta+np.pi/2)) + \
                    R*mod(2*np.pi+mod(theta+np.pi/2)-mod(chie+np.pi/2))

            L_list = [L1, L2, L3, L4]
            index = np.argmin(L_list)
            
            e1 = np.array([1,0,0])
            z3 = pe
            q3 = rotz(chie) @ e1
            if index == 0:
                cs = c_rs
                ce = c_re
                lam_s = 1
                lam_e = 1
                q1 = ce - cs
                q1 /= np.linalg.norm(q1)
                R_Rz_n1 = R*rotz(-np.pi/2) @ q1
                z1 = cs + R_Rz_n1
                z2 = ce + R_Rz_n1
            elif index == 1:
                cs = c_rs
                ce = c_le
                lam_s = 1
                lam_e = -1
                diff = ce - cs
                ell = np.linalg.norm(diff)
                vartheta = np.arctan2(diff[1],diff[0])
                vartheta2 = vartheta - np.pi/2 + np.arcsin(2*R/ell)
                q1 = rotz(vartheta2+np.pi/2) @ e1
                z1 = cs + R*rotz(vartheta2) @ e1
                z2 = ce + R*rotz(vartheta2+np.pi) @ e1
            elif index == 2:
                cs = c_ls
                ce = c_re
                lam_s = -1
                lam_e = 1
                diff = ce - cs
                ell = np.linalg.norm(diff)
                vartheta = np.arctan2(diff[1],diff[0])
                vartheta2 = np.arccos(2*R/ell)
                q1 = rotz(vartheta+vartheta2-np.pi/2) @ e1
                z1 = cs + R*rotz(vartheta+vartheta2) @ e1
                z2 = ce + R*rotz(vartheta+vartheta2-np.pi) @ e1
            elif index == 3:
                cs = c_ls
                ce = c_le
                lam_s = -1
                lam_e = -1
                q1 = ce - cs
                q1 /= np.linalg.norm(q1)
                R_Rz_n1 = R*rotz(np.pi/2) @ q1
                z1 = cs + R_Rz_n1
                z2 = ce + R_Rz_n1
            else:
                Exception("Bad dubins index")

            self.p_s = ps
            self.chi_s = chis
            self.p_e = pe
            self.chi_e = chie
            self.radius = R
            self.length = ell
            self.center_s = cs
            self.dir_s = lam_s
            self.center_e = ce
            self.dir_e = lam_e
            self.r1 = z1
            self.n1 = q1
            self.r2 = z2
            self.r3 = z3
            self.n3 = q3


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    # make x between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


