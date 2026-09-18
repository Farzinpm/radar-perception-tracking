import rospy
from rospy.core import logdebug, logwarn, rospyinfo

import numpy as np

from simulation_groundtruth.msg import ObjectHypothesis as ObjectHypothesisMsg
from simulation.utils.geometry import Point

#delta_t = current_time - self.last_update_ts

# KF taken from http://ros-developer.com/2019/04/10/kalman-filter-explained-with-python-code-from-scratch/
# as basis and developed further
class KalmanFilter():


    # control input - e.g. ego motion
    U = np.array([0])
   

    """ Class providing a Kalman Filter

    Attributes
    X_t: initial state,
    P_t: initial cov matrix,
    A_t: system matrix, transition matrix A*x + B*u, 
    B_t: control matrix A*x + B*u,
    H_t: sensor model H*x = predicted measurement
    Q_t: process noise - design parameter
    U_t: control input vector
    """  
    def __init__(self, X_t, P_t, A_t, B_t, Q_t, H_t, timestamp):  
        self.X_t = X_t     # predicted state
        self.X_hat_t = X_t    # estimated state
        
        self.P_t = P_t  # state cov matrix
        self.P_hat_t = P_t  # predicted state cov matrix
    
        self.B_t = B_t  # control matrix
        self.U_t = np.zeros(4) # control input
        self.A_t = A_t  # system matrix, transition matrix
        self.Q_t = Q_t  # process noise - design parameter
        
        self.H_t = H_t  # sensor model
    
        self.K_gain = [0]

        self.last_update_ts_s = timestamp
   
        return


    def to_ObjectHypothesisMsg(self):
        obj = ObjectHypothesisMsg()

        #rospy.logdebug(f"Statevector:\n {self.X_t}")
        if (self.X_t.size > 3):
            obj.position = Point(self.X_t[0, 0], self.X_t[1, 0], 0)
            obj.velocity = Point(self.X_t[2, 0], self.X_t[3, 0], 0)
            obj.orientation = Point(0,0,0)
            obj.last_update_ts_s = self.last_update_ts_s
               
        return obj

    def X_from_ObjectHypothesisMsg(obj : ObjectHypothesisMsg):
        return np.array([obj.position.x, obj.position.y, obj.velocity.x, obj.velocity.y])

    # control input u_t
    def predict(self, U_t):

        # x_pred = A * x_t-1 + B * u
        # Hinweis: B*u können Sie weglassen in dieser Aufgabe. Das führt erstmal nur zu unnötigen
        # Schwierigkeiten mit der Abstimmung der Matrixdimensionen... 
        self.X_hat_t = self.A_t @ self.X_t 

        if self.X_hat_t.size > 4:
            rospy.logwarn("implausible dimension of x_hat_t")

        # p_pred = A * P_t-1 * At + Q
        self.P_hat_t = self.A_t @ self.P_t @ np.transpose(self.A_t) + self.Q_t

        # write results into state values in case track does not get updated
        # they get overwritten in case of an update via measurement
        self.X_t = self.X_hat_t
        self.P = self.P_hat_t
        self.U_t = U_t
        return
    
    def updateObjPos(self, obj : ObjectHypothesisMsg, R_t, timestamp):
        z_t = np.array([ [obj.position.x], [obj.position.y]])
        self.updateMeas(z_t, R_t, timestamp)
        return
    
    def updateObjVel(self, obj : ObjectHypothesisMsg, R_t, timestamp):
        z_t = np.array([[obj.position.x], [obj.position.y], [obj.velocity.x], [obj.velocity.y]])
        self.updateMeas(z_t, R_t, timestamp)
        return

    # measurement z_t, 
    def updateMeas(self, Z_t, R_t, timestamp):

        S = self.H_t @ self.P_hat_t @ np.transpose(self.H_t) + R_t

        # k = p_pred*Ht / (H * p_pred * Ht + R)
        self.K_gain = self.P_hat_t @ np.transpose(self.H_t) @ np.linalg.inv(S)

        # x = x_pred + K * (z - H*x)
        self.X_t = self.X_hat_t + self.K_gain @ (Z_t - self.H_t @ self.X_t)
        if self.X_t.size > 4:
            rospy.logwarn("implausible dimension of x_t")

        # p = p_pred - K*H*p_pred
        self.P_t = self.P_hat_t - self.K_gain @ self.H_t @ self.P_hat_t

        self.last_update_ts_s = timestamp
        return

