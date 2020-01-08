"""
Extended Kalman Filter SLAM example
author: Atsushi Sakai (@Atsushi_twi)
modified: Craig Reid (@CraigReid42)
"""

from __future__ import division
from __future__ import absolute_import
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from autonomous_sim.msg import DriveCmd

class EkfSlam(object):
    # EKF state covariance
    Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2

    #  Simulation parameter
    Qsim = np.diag([0.2, np.deg2rad(1.0)])**2
    Rsim = np.diag([1.0, np.deg2rad(10.0)])**2

    DT = 0.01  # time tick [s]
    SIM_TIME = 50.0  # simulation time [s]
    MAX_RANGE = 20.0  # maximum observation range
    M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
    STATE_SIZE = 3  # State size [x,y,yaw]
    LM_SIZE = 2  # LM state size [x,y]

    show_animation = True

    v = 0.0
    yawrate = 0.0


    def ekf_slam(self, xEst, PEst, u, z):

        # Predict
        S = EkfSlam.STATE_SIZE
        xEst[0:S] = EkfSlam.motion_model(self, xEst[0:S], u)
        G, Fx = EkfSlam.jacob_motion(self, xEst[0:S], u)
        PEst[0:S, 0:S] = G.T * PEst[0:S, 0:S] * G + Fx.T * EkfSlam.Cx * Fx
        initP = np.eye(2)

        # Update
        for iz in xrange(len(z[:, 0])):  # for each observation
            minid = EkfSlam.search_correspond_LM_ID(self, xEst, PEst, z[iz, 0:2])

            nLM = EkfSlam.calc_n_LM(self, xEst)
            if minid == nLM:
                # Extend state and covariance matrix
                xAug = np.vstack((xEst, EkfSlam.calc_LM_Pos(self, xEst, z[iz, :])))
                PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), EkfSlam.LM_SIZE)))),
                                np.hstack((np.zeros((EkfSlam.LM_SIZE, len(xEst))), initP))))
                xEst = xAug
                PEst = PAug
            lm = EkfSlam.get_LM_Pos_from_state(self, xEst, minid)
            y, S, H = EkfSlam.calc_innovation(self, lm, xEst, PEst, z[iz, 0:2], minid)

            K = (PEst.dot(H.T)).dot(np.linalg.inv(S))
            xEst = xEst + (K.dot(y))
            PEst = (np.eye(len(xEst)) - (K.dot(H))).dot(PEst)

        xEst[2] = EkfSlam.pi_2_pi(self, xEst[2])

        return xEst, PEst


    def calc_input(self):
        #v = 1.0  # [m/s]
        #yawrate = 0.1  # [rad/s]
        u = np.array([[self.v, self.yawrate]]).T
        return u


    def observation(self, xTrue, xd, u, RFID):

        xTrue = EkfSlam.motion_model(self, xTrue, u)

        # add noise to gps x-y
        z = np.zeros((0, 3))

        for i in xrange(len(RFID[:, 0])):

            dx = RFID[i, 0] - xTrue[0, 0]
            dy = RFID[i, 1] - xTrue[1, 0]
            d = math.sqrt(dx**2 + dy**2)
            angle = EkfSlam.pi_2_pi(self, math.atan2(dy, dx) - xTrue[2, 0])
            if d <= EkfSlam.MAX_RANGE:
                dn = d + np.random.randn() * EkfSlam.Qsim[0, 0]  # add noise
                anglen = angle + np.random.randn() * EkfSlam.Qsim[1, 1]  # add noise
                zi = np.array([dn, anglen, i])
                z = np.vstack((z, zi))

        # add noise to input
        ud = np.array([[
            u[0, 0] + np.random.randn() * EkfSlam.Rsim[0, 0],
            u[1, 0] + np.random.randn() * EkfSlam.Rsim[1, 1]]]).T

        xd = EkfSlam.motion_model(self, xd, ud)
        return xTrue, z, xd, ud


    def motion_model(self, x, u):

        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])

        B = np.array([[EkfSlam.DT * math.cos(x[2, 0]), 0],
                    [EkfSlam.DT * math.sin(x[2, 0]), 0],
                    [0.0, EkfSlam.DT]])

        x = (F.dot(x)) + (B.dot(u))
        return x


    def calc_n_LM(self, x):
        n = int((len(x) - EkfSlam.STATE_SIZE) / EkfSlam.LM_SIZE)
        return n


    def jacob_motion(self, x, u):

        Fx = np.hstack((np.eye(EkfSlam.STATE_SIZE), np.zeros(
            (EkfSlam.STATE_SIZE, EkfSlam.LM_SIZE * EkfSlam.calc_n_LM(self, x)))))

        jF = np.array([[0.0, 0.0, -EkfSlam.DT * u[0] * math.sin(x[2, 0])],
                    [0.0, 0.0, EkfSlam.DT * u[0] * math.cos(x[2, 0])],
                    [0.0, 0.0, 0.0]])

        G = np.eye(EkfSlam.STATE_SIZE) + Fx.T * jF * Fx

        return G, Fx,


    def calc_LM_Pos(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
        #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
        #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

        return zp


    def get_LM_Pos_from_state(self, x, ind):

        lm = x[EkfSlam.STATE_SIZE + EkfSlam.LM_SIZE * ind: EkfSlam.STATE_SIZE + EkfSlam.LM_SIZE * (ind + 1), :]

        return lm


    def search_correspond_LM_ID(self, xAug, PAug, zi):
        """
        Landmark association with Mahalanobis distance
        """

        nLM = EkfSlam.calc_n_LM(self, xAug)

        mdist = []

        for i in xrange(nLM):
            lm = EkfSlam.get_LM_Pos_from_state(self, xAug, i)
            y, S, H = EkfSlam.calc_innovation(self, lm, xAug, PAug, zi, i)
            mdist.append(y.T.dot(np.linalg.inv(S)).dot(y))

        mdist.append(EkfSlam.M_DIST_TH)  # new landmark

        minid = mdist.index(min(mdist))

        return minid


    def calc_innovation(self, lm, xEst, PEst, z, LMid):
        delta = lm - xEst[0:2]
        q = (delta.T.dot(delta))[0, 0]
        zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
        zp = np.array([[math.sqrt(q), EkfSlam.pi_2_pi(self, zangle)]])
        y = (z - zp).T
        y[1] = EkfSlam.pi_2_pi(self, y[1])
        H = EkfSlam.jacobH(self, q, delta, xEst, LMid + 1)
        S = H.dot(PEst).dot(H.T) + EkfSlam.Cx[0:2, 0:2]

        return y, S, H


    def jacobH(self, q, delta, x, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                    [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

        G = G / q
        nLM = EkfSlam.calc_n_LM(self, x)
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

        F = np.vstack((F1, F2))

        H = G.dot(F)

        return H


    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


    def main(self):
        print __file__ + " start!!"

        time = 0.0

        # RFID positions [x, y]
        RFID = np.array([[10.0, 10.0],
                        [11.0, 11.0]])

        # State Vector [x y yaw v]'
        xEst = np.zeros((EkfSlam.STATE_SIZE, 1))
        xTrue = np.zeros((EkfSlam.STATE_SIZE, 1))
        PEst = np.eye(EkfSlam.STATE_SIZE)

        xDR = np.zeros((EkfSlam.STATE_SIZE, 1))  # Dead reckoning

        # history
        hxEst = xEst
        hxTrue = xTrue
        hxDR = xTrue

        while EkfSlam.SIM_TIME >= time:
            time += EkfSlam.DT
            u = EkfSlam.calc_input(self)

            xTrue, z, xDR, ud = EkfSlam.observation(self, xTrue, xDR, u, RFID)

            xEst, PEst = EkfSlam.ekf_slam(self, xEst, PEst, ud, z)

            x_state = xEst[0:EkfSlam.STATE_SIZE]

            # store data history
            hxEst = np.hstack((hxEst, x_state))
            hxDR = np.hstack((hxDR, xDR))
            hxTrue = np.hstack((hxTrue, xTrue))

            if EkfSlam.show_animation:  # pragma: no cover
                plt.cla()

                plt.plot(RFID[:, 0], RFID[:, 1], "*k")
                plt.plot(xEst[0], xEst[1], ".r")

                # plot landmark
                for i in xrange(EkfSlam.calc_n_LM(self, xEst)):
                    plt.plot(xEst[EkfSlam.STATE_SIZE + i * 2],
                            xEst[EkfSlam.STATE_SIZE + i * 2 + 1], "xg")

                plt.plot(hxTrue[0, :],
                        hxTrue[1, :], "-b")
                plt.plot(hxDR[0, :],
                        hxDR[1, :], "-k")
                plt.plot(hxEst[0, :],
                        hxEst[1, :], "-r")
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.001)


def update_input_via_twist(twist, ekf):
    ekf.v = twist.linear.x
    ekf.yawrate = twist.angular.z


def update_input_via_drive_cmd(driveCmd, ekf):
    ekf.v = driveCmd.rpm
    ekf.yawrate = driveCmd.steer_pct


def setup_cmd_vel_listener(ekf):
    rospy.Subscriber("cmd_vel", Twist, update_input_via_twist, ekf)


def setup_drive_cmd_listener(ekf):
    rospy.Subscriber("gazebo/driver/drive_cmd", DriveCmd, update_input_via_drive_cmd, ekf)


if __name__ == u'__main__':
    ekf = EkfSlam()
    rospy.init_node("ekf", anonymous=True)
    setup_cmd_vel_listener(ekf)
    setup_drive_cmd_listener(ekf)
    EkfSlam.main(ekf)
