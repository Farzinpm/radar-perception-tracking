import functools
from logging import debug
import math
from pickle import APPEND
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

from numpy.core.records import array

import geometry_msgs.msg
import rospy

import numpy as np
from numpy.linalg import eig
from tf.transformations import quaternion_from_euler

from rospy.core import logdebug, logwarn, rospyinfo
from simulation.utils import geometry
import std_msgs
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from simulation_groundtruth.msg import ObjectHypothesis as ObjectHypothesisMsg
from simulation_groundtruth.msg import ObjectHypothesisList as ObjectHypothesisListMsg
from src.lab_perception.src.object_fusion.kalman_filter import KalmanFilter
from lab_perception.msg import KalmanFilter_State as KalmanFilter_StateMsg

from simulation_groundtruth.srv import LabeledPolygonSrv, SectionSrv

from simulation.utils.geometry import Polygon, Pose, Transform, Vector, Point
from simulation.utils.ros_base.node_base import NodeBase


class ObjectFusionNode(NodeBase):
    """ROS node to fuse radar and camera objects

    Output is the fused objectlist ahead.

    Attributes:
    """

    def __init__(self):

        super().__init__(
            name="object_fusion_node", log_level=rospy.DEBUG
        )  # Name can be overwritten in launch file

        self.run(function=self.update, rate=float(self.param.rate))

    def start(self):
        """Start node."""

        # publisher
        self.pub_objectlist_fused = rospy.Publisher(
            self.param.topics.objectlist.fused,
            ObjectHypothesisListMsg,
            queue_size=1,
        )
        

        self.pub_objectlist_vis_fused_pos = rospy.Publisher(
            self.param.topics.visualization.obstacle.fused + "/pos",
            Marker,
            queue_size=1,
        )

        self.pub_objectlist_vis_fused_vel = rospy.Publisher(
            self.param.topics.visualization.obstacle.fused + "/vel",
            MarkerArray,
            queue_size=1,
        )
        self.pub_objectlist_vis_fused_cov = rospy.Publisher(
            self.param.topics.visualization.obstacle.fused + "/cov",
            MarkerArray,
            queue_size=1,
        )

        self.pub_kalman_state = rospy.Publisher(
            "/radar_tracking/kalman/state",
            KalmanFilter_StateMsg,
            queue_size=1
        )

        self.pub_kalman_state_cov = rospy.Publisher(
            "/radar_tracking/kalman/state_cov",
            KalmanFilter_StateMsg,
            queue_size=1
        )

        self.pub_kalman_control_input = rospy.Publisher(
            "/radar_tracking/kalman/control_input",
            KalmanFilter_StateMsg,
            queue_size=1
        )

        #subscriber
        self.get_model_twist_subscriber = rospy.Subscriber(
            self.param.topics.model_plugin.namespace
            + "/"
            + self.param.car_name
            + "/"
            + self.param.topics.model_plugin.get.twist,
            geometry_msgs.msg.Twist,
            callback=self.receive_model_twist_cb,
            queue_size=1,
        )

        self.sub_objectlist_radar = rospy.Subscriber(
            self.param.topics.objectlist_emulation.objectlist.radar,
            ObjectHypothesisListMsg,
            callback=self.receive_radar_objects,
            queue_size=1,
        )
        self.sub_objectlist_cam = rospy.Subscriber(
            self.param.topics.objectlist_emulation.objectlist.camera,
            ObjectHypothesisListMsg,
            callback=self.receive_camera_objects,
            queue_size=1,
        )

        self.latest_camera_objects = ObjectHypothesisListMsg() 
        self.latest_radar_objects = ObjectHypothesisListMsg()
        self.fused_object_list = []
        self.last_update_ts = 0

        super().start()

    def stop(self):
        self.pub_objectlist_fused.unregister()
        self.pub_objectlist_vis_fused_pos.unregister()
        self.pub_objectlist_vis_fused_vel.unregister()
        self.pub_objectlist_vis_fused_cov.unregister()
        self.pub_kalman_state.unregister()
        self.pub_kalman_state_cov.unregister()
        self.pub_kalman_control_input.unregister()
        
        self.sub_objectlist_radar.unregister()
        self.sub_objectlist_cam.unregister()
        
        self.get_model_twist_subscriber.unregister()

        super().stop()
    
    def receive_model_twist_cb(self, msg: geometry_msgs.msg.Twist):
        """Receive new model twist."""

        self.latest_twist = msg

    def receive_radar_objects(self, msg: ObjectHypothesisListMsg):
        """Receive new radar object list"""
        self.latest_radar_objects = msg

    def receive_camera_objects(self, msg: ObjectHypothesisListMsg):
        """Receive new camera object list"""
        
        self.latest_camera_objects = msg

    def update(self):
        """Calculate and publish fused objectlist"""
        # Update current time
        current_time = self.latest_radar_objects.timestamp
        
        # Ihr Code steht hier:
        # KF fusion
        # Ihre Fusionsergebnisse können Sie hier eintragen mit
        # fused_object_list.append(ObjectHypothesis)
        fused_object_list  = ObjectHypothesisListMsg() 
        

        ################################################################
        # Vervollständigen Sie hier die Matrizen für das Kalman-Filter #
        ################################################################ 

        #define KF parameters
        delta_t = current_time - self.last_update_ts
        if delta_t < 1e-6:
            return
        # rospy.logdebug(f"{delta_t}")
        # constant velocity model
        A = ...
        # disable control input
        B = np.array( [ [0] , [0], [0] , [0]])
        # also known as C-Matrix, without speed measurements - only x and y in measurements vector
        # H = np.array([ [1, 0, 0, 0], [ 0, 1, 0, 0]])
        # also known as C-Matrix, with position and speed measurements in x and y
        H = ...
        # measurement matrix for camera, measurement vector is x,y,vx,vy
        R_cam = ...
        # measurement matrix for radar, measurement vector is x,y,vx,vy
        R_radar = ...

        # process cov
        # https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
        delta_t_2 = delta_t * delta_t
        delta_t_3 = delta_t_2 * delta_t
        delta_t_4 = delta_t_2 * delta_t_2        
        
        Q = ...

        # initial state cov
        # higher uncertainty for velocity
        P = ...
        

        # predict entire list
        for fused_obj in self.fused_object_list:
            fused_obj : KalmanFilter = fused_obj # python trick for autocompletion
            U = np.zeros((4,1))
            fused_obj.predict(U)

        # associate to previous fused objects and update accordingly
        # all RADAR objects
        for obj_hypo in self.latest_radar_objects.objectlist:
            obj_hypo : ObjectHypothesisMsg = obj_hypo  # python trick for autocompletion
            associated = False

            # find 
            for fused_obj in self.fused_object_list:
                fused_obj : KalmanFilter = fused_obj # python trick for autocompletion

                # rospy.logdebug(f"{obj_track.to_ObjectHypothesisMsg().position}")
                if abs(Vector(fused_obj.to_ObjectHypothesisMsg().position) - obj_hypo.position) < 1.0:
                    # replace in original list!
                    id = self.fused_object_list.index(fused_obj)
                    new_measurement = np.array([[obj_hypo.position.x], [obj_hypo.position.y], [obj_hypo.velocity.x], [obj_hypo.velocity.y] ])
                    self.fused_object_list[id].update(new_measurement, R_radar, current_time)
                    associated = True
                    # rospy.logdebug("associated & updated radar")
            
            # add if not found 
            if associated == False:
                intial_state_vector = np.array([[obj_hypo.position.x], [obj_hypo.position.y], [obj_hypo.velocity.x], [obj_hypo.velocity.y] ])
                new_obj = KalmanFilter(intial_state_vector, P, A, B, Q, H, current_time)
                self.fused_object_list.append( new_obj )
                #rospy.logdebug(f"adding object with position {obj_hypo.position}")
                #rospy.logdebug(f"{self.fused_object_list[len(self.fused_object_list)-1].X_t}")
                #rospy.logdebug(f"{self.fused_object_list[len(self.fused_object_list)-1].to_ObjectHypothesisMsg().position}")

         # camera
        for obj_hypo in self.latest_camera_objects.objectlist:
            obj_hypo : ObjectHypothesisMsg = obj_hypo  # python trick for autocompletion
            associated = False

            # find 
            for fused_obj in self.fused_object_list:
                fused_obj : KalmanFilter = fused_obj # python trick for autocompletion

                # rospy.logdebug(f"{obj_track.to_ObjectHypothesisMsg().position}")
                if abs(Vector(fused_obj.to_ObjectHypothesisMsg().position) - obj_hypo.position) < 1.0:
                    # replace in original list!
                    id = self.fused_object_list.index(fused_obj)
                    new_measurement = np.array([[obj_hypo.position.x], [obj_hypo.position.y], [obj_hypo.velocity.x], [obj_hypo.velocity.y] ])
                    self.fused_object_list[id].update(new_measurement, R_cam, current_time)
                    associated = True
                    # rospy.logdebug("associated & updated camera")
            
            # add if not found 
            if associated == False:
                intial_state_vector = np.array([[obj_hypo.position.x], [obj_hypo.position.y], [obj_hypo.velocity.x], [obj_hypo.velocity.y] ])
                new_obj = KalmanFilter(intial_state_vector, P, A, B, Q, H, current_time)
                self.fused_object_list.append( new_obj )
                #rospy.logdebug(f"{obj_hypo.position}")
                #rospy.logdebug(f"{self.fused_object_list[len(self.fused_object_list)-1].X_t}")
                #rospy.logdebug(f"{self.fused_object_list[len(self.fused_object_list)-1].to_ObjectHypothesisMsg().position}")

        # monitor length of self.fused_object_list
        #rospy.logdebug("aufter association: number of tracked objects: %u", len(self.fused_object_list.objectlist))

        #house-keeping
        # delete outdated objects
        for fused_obj in self.fused_object_list:
            fused_obj : KalmanFilter = fused_obj # python trick for autocompletion
            if (current_time - fused_obj.last_update_ts_s) > 0.05:
                self.fused_object_list.remove(fused_obj)

                
        # rospy.logdebug("after house-keeping: number of tracked objects: %u", len(self.fused_object_list))

        # Ihr Ergebnis self.fused_object_list.objectlist wird hier verschickt
        for fused_obj in self.fused_object_list:
            fused_obj : KalmanFilter = fused_obj # python trick for autocompletion
            fused_object_list.objectlist.append(fused_obj.to_ObjectHypothesisMsg())
            
        # set timestamp
        fused_object_list.timestamp = self.last_update_ts

        self.pub_objectlist_fused.publish(fused_object_list)

        ###### don't touch ###########
        # visualize all object lists #
        ##############################

        # fused object list
        ObjectCenterPoints = [] 
        for obj in fused_object_list.objectlist:
            ObjectCenterPoints.append(obj.position)
        
        vis_objects = Marker(type=Marker.POINTS)
        vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
        vis_objects.color.a = 1.0
        vis_objects.color.r = 1.0
        vis_objects.color.g = 1.0
        vis_objects.color.b = 0.0
        vis_objects.scale.x = 0.2
        vis_objects.scale.y = 0.2
        vis_objects.scale.z = 0.3
        vis_objects.points = ObjectCenterPoints

        self.pub_objectlist_vis_fused_pos.publish(vis_objects)
        
        vis_objectList_vel = MarkerArray()

        for obj in fused_object_list.objectlist:
            vis_objects = Marker(type=Marker.ARROW)

            vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
            vis_objects.id = fused_object_list.objectlist.index(obj)
            vis_objects.color.a = 1.0
            vis_objects.color.r = 1.0
            vis_objects.color.g = 1.0
            vis_objects.color.b = 0.0
            vis_objects.scale.x = abs(obj.velocity)*1
            vis_objects.scale.y = 0.05
            vis_objects.scale.z = 0.05
            vis_objects.lifetime = rospy.Duration(0,100e6)
            vis_objects.pose.position.x = obj.position.x 
            vis_objects.pose.position.y = obj.position.y
            vis_objects.pose.position.z = obj.position.z
            quat = quaternion_from_euler(0, 0, Vector(obj.velocity.x, obj.velocity.y).get_angle())
            vis_objects.pose.orientation.x = quat[0]
            vis_objects.pose.orientation.y = quat[1]
            vis_objects.pose.orientation.z = quat[2]
            vis_objects.pose.orientation.w = quat[3]
            
            vis_objectList_vel.markers.append(vis_objects)

        self.pub_objectlist_vis_fused_vel.publish(vis_objectList_vel)


        vis_objectList_cov = MarkerArray()
        for obj in fused_object_list.objectlist:
            # only changed items
            pos_cov = np.array([ [self.fused_object_list[0].P_t[0,0], self.fused_object_list[0].P_t[0,1] ],
            [self.fused_object_list[0].P_t[1,0], self.fused_object_list[0].P_t[1,1] ] ])
            w,v = eig(pos_cov)

            angle = np.arctan2( w[0] - self.fused_object_list[0].P_t[0,0], self.fused_object_list[0].P_t[0,1] )


            vis_objects = Marker(type=Marker.CYLINDER)
            vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
            vis_objects.id = fused_object_list.objectlist.index(obj)
            vis_objects.color.a = 0.3
            vis_objects.color.r = 1.0
            vis_objects.color.g = 1.0
            vis_objects.color.b = 0.0
            vis_objects.scale.x =  np.sqrt(w[0])
            vis_objects.scale.y = np.sqrt(w[1])
            vis_objects.scale.z = 0.01
            vis_objects.lifetime = rospy.Duration(0,100e6)
            vis_objects.pose.position.x = obj.position.x 
            vis_objects.pose.position.y = obj.position.y
            vis_objects.pose.position.z = obj.position.z - 0.1
            quat = quaternion_from_euler(0, 0, angle)
            vis_objects.pose.orientation.x = quat[0]
            vis_objects.pose.orientation.y = quat[1]
            vis_objects.pose.orientation.z = quat[2]
            vis_objects.pose.orientation.w = quat[3]

            vis_objectList_cov.markers.append(vis_objects)


        
        self.pub_objectlist_vis_fused_cov.publish(vis_objectList_cov)


        # send out kalman debugging
        if len(self.fused_object_list) > 0:

            state = KalmanFilter_StateMsg()
            state.x = self.fused_object_list[0].X_t.flatten().tolist()[0]
            state.y = self.fused_object_list[0].X_t.flatten().tolist()[1]
            state.vx = self.fused_object_list[0].X_t.flatten().tolist()[2]
            state.vy = self.fused_object_list[0].X_t.flatten().tolist()[3]
            self.pub_kalman_state.publish(state)

            # state cov
            state_cov = KalmanFilter_StateMsg()
            state_cov.x = self.fused_object_list[0].P_t[0,0]
            state_cov.y = self.fused_object_list[0].P_t[1,1]
            state_cov.vx = self.fused_object_list[0].P_t[2,2]
            state_cov.vy = self.fused_object_list[0].P_t[3,3]

            self.pub_kalman_state_cov.publish(state_cov)

            # control input
            control_input = KalmanFilter_StateMsg()
            control_input.x = self.fused_object_list[0].U_t.flatten().tolist()[0]
            control_input.y = self.fused_object_list[0].U_t.flatten().tolist()[1]
            control_input.vx = self.fused_object_list[0].U_t.flatten().tolist()[2]
            control_input.vy = self.fused_object_list[0].U_t.flatten().tolist()[3]

            self.pub_kalman_control_input.publish(control_input)

        self.last_update_ts = current_time