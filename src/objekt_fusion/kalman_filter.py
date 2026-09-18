
from simulation_groundtruth.msg import ObjectHypothesis as ObjectHypothesisMsg
from simulation_groundtruth.msg import ObjectHypothesisList as ObjectHypothesisListMsg
import rospy
from rospy.core import logdebug, logwarn, rospyinfo
from simulation.utils.geometry import Polygon, Pose, Transform, Vector, Point
import numpy as np

# KF taken from http://ros-developer.com/2019/04/10/kalman-filter-explained-with-python-code-from-scratch/
# as basis and developed further
class KalmanFilter():
    """ Class providing a Kalman Filter

    Attributes
    X_t: initial state,
    P_t: initial cov matrix,
    A_t: system matrix, transition matrix A*x + B*u, 
    B_t: control matrix A*x + B*u,
    H_t: sensor model H*x = predicted measurement
    Q_t: process noise - design parameter
    
    """  
    def __init__(self, X_t, P_t, A_t, B_t, Q_t, H_t, timestamp):  
        self.X_t = X_t     # predicted state
        self.X_hat_t = X_t    # estimated state

        self.P_t = P_t  # state cov matrix
        self.P_hat_t = P_t  # predicted state cov matrix

        self.B_t = B_t  # control matrix
        self.A_t = A_t  # system matrix, transition matrix
        self.Q_t = Q_t  # process noise - design parameter

        self.H_t = H_t  # sensor model

        self.U_t = np.array([0, 0, 0, 0,0,0])

        self.K_gain = np.array([0])

        self.last_update_ts_s = timestamp
        
        return


    def to_ObjectHypothesisMsg(self):
        obj = ObjectHypothesisMsg()

        
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
        self.U_t = U_t # store for debugging
        # x_pred = A * x_t-1 + B * u
        self.X_hat_t = self.A_t @ self.X_t #+ self.B_t.reshape(1,4) @ U_t.reshape(4,1)
        #if self.X_hat_t.size > 4:
        #    rospy.logwarn("implausible dimension of x_hat_t")
        # p_pred = A * P_t-1 * At + Q
        #self.P_hat_t = np.diag(np.diag(self.A_t.dot(self.P_t).dot(self.A_t.transpose())))+self.Q_t
        self.P_hat_t = self.A_t @ self.P_t @ self.A_t.transpose() + self.Q_t
        #rospy.logdebug(f"Process Cov p_hat\n {self.P_hat_t}")
        return

    # measurement z_t, 
    def update(self, Z_t, R_t, timestamp):
        # k = p_pred*Ht / (H * p_pred * Ht + R)
        S = np.linalg.inv (self.H_t @ self.P_hat_t @ self.H_t.transpose() + R_t )
        self.K_gain = self.P_hat_t @ self.H_t.transpose() @ S  
        #rospy.logdebug(f"Kalman gain:\n {K_gain}")
        # x = x_pred + K * (z - H*x)
        self.X_t = self.X_hat_t+self.K_gain @ (Z_t-self.H_t @ self.X_hat_t)
        #if self.X_t.size > 4:
        #    rospy.logwarn("implausible dimension of x_t")
        # p = p_pred - K*H*p_pred
        self.P_t = self.P_hat_t-self.K_gain.dot(self.H_t).dot(self.P_hat_t)

        self.last_update_ts_s = timestamp
        return
