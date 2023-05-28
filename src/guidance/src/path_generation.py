#!/usr/bin/env python3
import rospy
import numpy as np
import math

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy


def rad2pipi(x):  # Maps angle from (-inf, inf) to [-pi, pi)
    y = np.arctan2(np.sin(x), np.cos(x))
    return y

class PATH:
    def __init__(self):
         
        self.eta_d = np.zeros(3)
        self.eta_d_dt = np.zeros(3)
            #eta_d_dt2 = np.zeros(3)

        
        self.test = ""

        self.pub = rospy.Publisher(f"/{vessel_name}/reference", Float64MultiArray, queue_size=1)
        self.joy_sub = rospy.Subscriber(f"/joy", Joy, self.joyCallback, queue_size=1)
        self.pathgen_msg = Float64MultiArray()


    def nom_straightline_path(self):
        s = self.s
        U_ref = 0.05
        startP = [1, 0]
        endP = [7,0]
        eps = 0.00001
        startP = np.array([0, 0])
        endP = np.array([10, 0])
        # eta_d and its 1st and 2nd derivatives wrt s
        eta_d = np.zeros(3)
        eta_ds = np.zeros(3)
        eta_ds2 = np.zeros(3)
        eta_d[0] = s * endP[0] + (1 - s) * startP[0]
        eta_d[1] = s * endP[1] + (1 - s) * startP[1]
        eta_d[2] = math.atan2(endP[1] - startP[1], endP[0] - startP[0])

        eta_ds[0] = (endP[0] - startP[0])
        eta_ds[1] = (endP[1] - startP[1])
        eta_ds[2] = 0
        # wrap it with rad2pipi
        eta_d[2] = rad2pipi(eta_d[2])

        # Ramp for U_ref to avoid big initial acceleration
        t_ramp = 2  # duration of the ramp

        t = rospy.get_time() - self.t0
        if t < t_ramp:
            U_ref = U_ref * t / t_ramp
        else:
            U_ref = U_ref

        v_s = np.abs(U_ref) / math.sqrt(eta_ds[0] ** 2 + eta_ds[1] ** 2 + eps)
        s_dot = v_s
        #vs_ds = U_ref * (eta_ds2[0] * eta_ds[0] + eta_ds2[1] * eta_ds[1]) / (
        #        (eta_ds[0] ** 2 + eta_ds[1] ** 2) ** (3 / 2))

        #s_dt2 = vs_ds * s_dot
        # Calculate the 1st and 2nd time derivatives of eta_d
        eta_d_dt = np.zeros(3)
        #eta_d_dt2 = np.zeros(3)
        eta_d_dt = eta_ds * s_dot
        #eta_d_dt2 = eta_ds2 * s_dot ** 2 + eta_ds * s_dt2


        self.eta_d = eta_d
        self.eta_d_dt = eta_d_dt
        """
        eta_d = np.array([eta_d]).T
        eta_d_dt = np.array([eta_d_dt]).T
        #eta_d_dt2 = np.array([eta_d_dt2]).T
            y.eta_d = [2.0, 0.0, 0]
        eta_ds = np.array([eta_ds]).T
        #eta_ds2 = np.array([eta_ds2]).T
        """
        # Integrate s
        self.s = s + 0.01 * s_dot



    def nom_ellipsoidal_path(self):
        """
        Generates desired eta in an ellipsoidal path
        """
        U_ref = 0.05
        s = self.s
        rx = 0.75
        ry = 0.75
        offset = np.array([2.25, 0.65])

        psi_s = math.pi / 2
        eta_d = np.zeros(3)
        eta_ds = np.zeros(3)
        eta_ds2 = np.zeros(3)
        #eta_ds3 = np.zeros(3)
        eps = 0.0001

        # Reference eta
        eta_d[0] = offset[0] + rx * math.cos(2 * math.pi * s + psi_s)
        eta_d[1] = offset[1] + ry * math.sin(2 * math.pi * s + psi_s)

        # first deriative of eta_d with respect to s
        eta_ds[0] = - 2 * math.pi * rx * math.sin(2 * math.pi * s + psi_s)
        eta_ds[1] = 2 * math.pi * ry * math.cos(2 * math.pi * s + psi_s)
        eta_d[2] = math.atan2(eta_ds[1], eta_ds[0])
        eta_ds[2] = (-4 * math.pi ** 2 * ry * math.sin(2 * math.pi * s + psi_s) * eta_d[0] - eta_ds[
            1] * -4 * math.pi ** 2 * rx * math.cos(2 * math.pi * s + psi_s)) / (eta_ds[0] ** 2 + eta_ds[1] ** 2 + eps)
        # eta_ds[2] = (2*rx*ry*math.pi)/(ry**2 + rx**2*math.sin(2*math.pi*s + psi_s)**2 - ry**2*math.cos(2*math.pi*s + psi_s)**2 + eps)

        # Reference eta
        eta_ds2[0] = -4 * math.pi ** 2 * rx * math.cos(2 * math.pi * s + psi_s)
        eta_ds2[1] = -4 * math.pi ** 2 * ry * math.sin(2 * math.pi * s + psi_s)

        #eta_ds3[0] = 8 * math.pi ** 3 * rx * math.sin(2 * math.pi * s + psi_s)
        #eta_ds3[1] = -8 * math.pi ** 3 * ry * math.cos(2 * math.pi * s + psi_s)

        #eta_ds2[2] = (eta_ds[0] ** 3 * eta_ds3[1] - eta_ds[1] ** 3 * eta_ds3[0] + 2 * eta_ds[0] * eta_ds[1] * eta_ds2[
        #    0] ** 2 - 2 * eta_ds[0] * eta_ds[1] * eta_ds2[1] ** 2 - eta_ds[0] ** 2 * eta_ds[1] * eta_ds3[0] - 2 *
        #                eta_ds[
        #                    0] ** 2 * eta_ds2[0] * eta_ds2[1] + 2 * eta_ds[1] ** 2 * eta_ds2[0] * eta_ds2[1] + eta_ds[0] *
        #                eta_ds[1] ** 2 * eta_ds3[1]) / (eta_ds[0] ** 2 + eta_ds[1] ** 2 + eps) ** 2

        # Ramp for U_ref to avoid big initial acceleration
        t_ramp = 2  # duration of the ramp

        t = rospy.get_time() - self.t0
        if t < t_ramp:
            U_ref = U_ref * t / t_ramp
        else:
            U_ref = U_ref

        v_s = np.abs(U_ref) / math.sqrt(eta_ds[0] ** 2 + eta_ds[1] ** 2 + eps)
        s_dot = v_s
        #vs_ds = U_ref * (eta_ds2[0] * eta_ds[0] + eta_ds2[1] * eta_ds[1]) / (
        #        (eta_ds[0] ** 2 + eta_ds[1] ** 2) ** (3 / 2))

        #s_dt2 = vs_ds * s_dot
        # Calculate the 1st and 2nd time derivatives of eta_d
        eta_d_dt = np.zeros(3)
        #eta_d_dt2 = np.zeros(3)
        eta_d_dt = eta_ds * s_dot
        #eta_d_dt2 = eta_ds2 * s_dot ** 2 + eta_ds * s_dt2


        self.eta_d = eta_d
        self.eta_d_dt = eta_d_dt
        """
        eta_d = np.array([eta_d]).T
        eta_d_dt = np.array([eta_d_dt]).T
        #eta_d_dt2 = np.array([eta_d_dt2]).T
        eta_ds = np.array([eta_ds]).T
        #eta_ds2 = np.array([eta_ds2]).T
        """
        # Integrate s
        self.s = s + 0.01 * s_dot



        #return eta_d, eta_d_dt, eta_d_dt2, eta_ds, eta_ds2

    def joyCallback(self, msg):
        if msg.axes[10] == 1:
            self.test = "circle"
            rospy.logwarn_once("circle")
        elif msg.axes[10] == -1:
            self.test = "stop"
            rospy.logwarn_once("stop")

        


    def publish(self):
        self.pathgen_msg.data = [self.eta_d[0], self.eta_d[1], self.eta_d[2], self.eta_d_dt[0], self.eta_d_dt[1], self.eta_d_dt[2]]
        self.pub.publish(self.pathgen_msg)



if __name__ == '__main__':
    vessel_name = "CSEI"
    rospy.init_node(f"{vessel_name}_guidanceand path_type == 1_node")
    rospy.loginfo(f"INITIALIZING {vessel_name} guidance NODE")
    r = rospy.Rate(100)
    # Initialize
    y = PATH()
    path_type = 1 # 0 for straight line, 1 for ellipsoidal
    if path_type == 0:
        y.s = 0
    if path_type == 1:
        y.s = 0.5

    # start point
    y.eta_d = [2.25, -0.1, 0]

    y.t0 = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        if y.test == "straightline":
            y.nom_straightline_path()
        if y.test == "circle":
            y.nom_ellipsoidal_path()
        if y.test == "stop":
            y.eta_d = [2.25, -0.1, 0]
            y.eta_d_dt = [0, 0, 0]


        # Publish message
        y.publish()


        r.sleep()

    rospy.spin()