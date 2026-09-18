from pyquaternion.quaternion import Quaternion
from tf.transformations import quaternion_from_euler

import geometry_msgs.msg
import rospy

import numpy as np
from numpy.linalg import eig

from rospy.core import logdebug, logwarn, rospyinfo
from simulation.utils import geometry
from src.lab_perception.src.radar_tracking.kalman_filter import KalmanFilter

from visualization_msgs.msg import Marker, MarkerArray
from simulation_groundtruth.msg import ObjectHypothesis as ObjectHypothesisMsg
from simulation_groundtruth.msg import ObjectHypothesisList as ObjectHypothesisListMsg
from lab_simulation_core.msg import RadarMeasurement as ReadarMeasurementMsg
from lab_simulation_core.msg import RadarMeasurementList as RadarMeasurementListMsg

from lab_perception.msg import KalmanFilter_State as KalmanFilter_StateMsg

from simulation.utils.geometry import Polygon, Pose, Transform, Vector, Point, transform
from simulation.utils.ros_base.node_base import NodeBase


class RadarTrackingNode(NodeBase):
    """ROS node to track radar raw measurements

    Output is the tracked object list.

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
        self.pub_objectlist_radar = rospy.Publisher(
            self.param.topics.objectlist.radar,
            ObjectHypothesisListMsg,
            queue_size=1,
        )
        

        self.pub_objectlist_vis_radar_pos = rospy.Publisher(
            self.param.topics.visualization.obstacle.radar + "/pos",
            Marker,
            queue_size=1,
        )
        self.pub_objectlist_vis_radar_vel = rospy.Publisher(
            self.param.topics.visualization.obstacle.radar + "/vel",
            MarkerArray,
            queue_size=1,
        )
        self.pub_objectlist_vis_radar_cov = rospy.Publisher(
            self.param.topics.visualization.obstacle.radar + "/cov",
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

        self.sub_rawdata_radar = rospy.Subscriber(
            self.param.topics.radar_rawdata_emulation.rawdata.radar,
            RadarMeasurementListMsg,
            callback=self.receive_radar_rawdata,
            queue_size=1,
        )
        
        
        self.latest_radar_rawdata = RadarMeasurementListMsg()
        self.tracked_radar_objects = [] 
        self.last_update_ts = 0

        super().start()

    def stop(self):
        self.pub_objectlist_radar.unregister()
        self.pub_objectlist_vis_radar_pos.unregister()
        self.pub_objectlist_vis_radar_vel.unregister()
        self.pub_objectlist_vis_radar_cov.unregister()
        self.pub_kalman_state.unregister()
        self.pub_kalman_state_cov.unregister()
        self.pub_kalman_control_input.unregister()
        
        self.sub_rawdata_radar.unregister()
        
        self.get_model_twist_subscriber.unregister()

        super().stop()

    def receive_model_twist_cb(self, msg: geometry_msgs.msg.Twist):
        """Receive new model twist."""

        self.latest_twist = msg

    def receive_radar_rawdata(self, msg: RadarMeasurementListMsg):
        """Receive new radar object list"""
        self.latest_radar_rawdata = msg

    def update(self):
        """Implementation of radar object tracking"""
        # Update the driving state
        current_time = self.latest_radar_rawdata.timestamp
        
        # Ihr Code steht hier:
        radar_measurements = self.latest_radar_rawdata.measurements

        # rospy.logdebug("radar tracking: received %u radar reflections", len(radar_measurements))

        # Ihr Ergebnis wird in diese Variable gefüllt:
        radar_object_list  = ObjectHypothesisListMsg() 
        radar_object_list.timestamp = current_time

        pose = Pose(Point(0.25, 0, 0.4), Quaternion(1,0,0,0))
        tf = Transform(pose)
        #rospy.logdebug(f"{tf}")
        

        #define KF parameters
        delta_t = current_time - self.last_update_ts
        if delta_t < 1e-6:
            return

        # constant velocity model
        A = np.array( [ [1, 0, delta_t, 0] , 
                        [0, 1, 0, delta_t], 
                        [0, 0, 1, 0] , 
                        [0, 0, 0, 1] ] )
        # disable control input
        # B = np.array( [ [0.5*delta_t_2, 0] , [0, 0.5*delta_t_2], [delta_t, 0 ] , [0, delta_t] ] )
        B = np.array ( [[0], [0], [0], [0]])
        # without speed measurements, also known as C-Matrix
        H = np.array( [ [1, 0, 0, 0] , 
                        [0, 1, 0, 0] ,
                        [0, 0, 1, 0] ,
                        [0, 0, 0, 1] ] )
        # measurement noise
        R = np.array( [ [2, 0, 0, 0] , 
                        [0, 4, 0, 0] ,
                        [0, 0, 2, 0] ,
                        [0, 0, 0, 4] ] )  #erstmal 2 angenommen aus Skript
        # process cov
        # https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
        delta_t_2 = delta_t * delta_t
        delta_t_3 = delta_t_2 * delta_t
        delta_t_4 = delta_t_2 * delta_t_2        
        
        Q = np.array( [ [ delta_t_4*0.25, 0, delta_t_3*0.5, 0] , 
                        [0, delta_t_4*0.25, 0, delta_t_3*0.5], 
                        [delta_t_3*0.5, 0, delta_t_2, 0] , 
                        [0, delta_t_3*0.5, 0, delta_t_2] ] ) * 0.7 #4x4
        # initial state cov
        # high uncertainty for velocity as not measured
        P = np.array( [ [1,  0,   0,   0] , 
                        [0, 11,   0,   0] , 
                        [0,  0, 100,   0] , 
                        [0,  0,   0, 400] ] )
        # control input - e.g. ego motion
        U = np.array([0])

        # predict entire list
        for obj_track in self.tracked_radar_objects:
            obj_track.predict(U)

        # convert radar measurements to Objecthypothesis    
        for radar_meas in radar_measurements:
            radar_meas : ReadarMeasurementMsg = radar_meas
            ObjectHypothesis = ObjectHypothesisMsg()

            phi = radar_meas.hor_angle 
            range = radar_meas.range
            speed = radar_meas.radial_velocity
            x = np.cos(phi) * range + 0.25
            y = np.sin(phi) * range 
            vx = np.cos(phi) * speed 
            vy = np.sin(phi) * speed 
            ObjectHypothesis.position = Point( x, y, 0)      #Ihr code aus vorheriger Aufgabe - done
            ObjectHypothesis.velocity = Point( vx, vy, 0)    #Ihr code aus vorheriger Aufgabe - done

            ObjectHypothesis.last_update_ts_s = self.latest_radar_rawdata.timestamp #Ihr code aus vorheriger Aufgabe - done
            
            radar_object_list.objectlist.append(ObjectHypothesis)


        # associate to previous radar objects
        for obj_hypo in radar_object_list.objectlist:
            associated = False

            # find 
            for obj_track in self.tracked_radar_objects:
                # rospy.logdebug(f"{obj_track.to_ObjectHypothesisMsg().position}")
                obj_track : KalmanFilter = obj_track
                if abs(Vector(obj_track.to_ObjectHypothesisMsg().position) - obj_hypo.position) < 1.3: #Ihr Wert aus vorheriger Aufgabe - done
                    # replace in original list!
                    id = self.tracked_radar_objects.index(obj_track)
                    self.tracked_radar_objects[id].updateObjVel(obj_hypo, R, current_time)
                    associated = True
                    break   # gaaaaaaanz wichtig! Keine weiteren Zuweisungen, sonst werden mehrere Tracks am Leben gehalten
                    rospy.logdebug("associated")
            
            # add if not found 
            if associated == False:
                self.tracked_radar_objects.append( KalmanFilter(np.array([[obj_hypo.position.x], [obj_hypo.position.y], [0], [0] ]),
                    P, A, B, Q, H, current_time))
                #rospy.logdebug(f"{obj_hypo.position}")
                #rospy.logdebug(f"{self.tracked_radar_objects[len(self.tracked_radar_objects)-1].X_t}")
                #rospy.logdebug(f"{self.tracked_radar_objects[len(self.tracked_radar_objects)-1].to_ObjectHypothesisMsg().position}")

        # monitor length of self.tracked_radar_objects
        # rospy.logdebug("aufter association: number of tracked objects: %u", len(self.tracked_radar_objects.objectlist))

        #house-keeping - Ihr Algorithmus (Datenstrukturen anpassen!) - done 
        
        for obj_track in self.tracked_radar_objects:
            if abs(obj_track.last_update_ts_s - current_time) > 0.2:
                self.tracked_radar_objects.remove(obj_track)
             
        rospy.logdebug("after house-keeping: number of tracked objects: %u", len(self.tracked_radar_objects))

        # Ihr Ergebnis self.tracked_radar_objects.objectlist wird hier verschickt
        obj_list = ObjectHypothesisListMsg()
        obj_list.timestamp = self.latest_radar_rawdata.timestamp
        for obj_track in self.tracked_radar_objects:
            obj_list.objectlist.append(obj_track.to_ObjectHypothesisMsg())

        self.pub_objectlist_radar.publish(obj_list)


        ##############################
        # visualize all object lists #
        ##############################

        # tracked object list
        ObjectCenterPoints = [] 
        for obj in obj_list.objectlist:
            ObjectCenterPoints.append(obj.position)
        
        vis_objects = Marker(type=Marker.POINTS)
        vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
        vis_objects.color.a = 1.0
        vis_objects.color.r = 0.2
        vis_objects.color.g = 0.2
        vis_objects.color.b = 0.2
        vis_objects.scale.x = 0.2
        vis_objects.scale.y = 0.2
        vis_objects.scale.z = 0.05
        vis_objects.points = ObjectCenterPoints

        self.pub_objectlist_vis_radar_pos.publish(vis_objects)
        
        vis_objectList_vel = MarkerArray()

        for obj in obj_list.objectlist:
            vis_objects = Marker(type=Marker.ARROW)

            vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
            vis_objects.id = obj_list.objectlist.index(obj)
            vis_objects.color.a = 1.0
            vis_objects.color.r = 0.2
            vis_objects.color.g = 0.2
            vis_objects.color.b = 0.2
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

        self.pub_objectlist_vis_radar_vel.publish(vis_objectList_vel)


        vis_objectList_cov = MarkerArray()
        for obj in obj_list.objectlist:
            # only changed items
            pos_cov = np.array([ [self.tracked_radar_objects[0].P_t[0,0], self.tracked_radar_objects[0].P_t[0,1] ],
            [self.tracked_radar_objects[0].P_t[1,0], self.tracked_radar_objects[0].P_t[1,1] ] ])
            w,v = eig(pos_cov)

            angle = np.arctan2( w[0] - self.tracked_radar_objects[0].P_t[0,0], self.tracked_radar_objects[0].P_t[0,1] )


            vis_objects = Marker(type=Marker.CYLINDER)
            vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
            vis_objects.id = obj_list.objectlist.index(obj)
            vis_objects.color.a = 0.1
            vis_objects.color.r = 0.2
            vis_objects.color.g = 0.2
            vis_objects.color.b = 0.2
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


        
        self.pub_objectlist_vis_radar_cov.publish(vis_objectList_cov)


        # send out kalman debugging
        if len(self.tracked_radar_objects) > 0:
            # data_to_send = KalmanFilterMsg()
            # data_to_send.State_X = self.tracked_radar_objects[0].X_t.flatten().tolist()
            # #data_to_send.State_Cov = [ self.tracked_radar_objects[0].P_t[0,0], self.tracked_radar_objects[0].P_hat_t[1,1], self.tracked_radar_objects[0].P_t[2,2], self.tracked_radar_objects[0].P_t[3,3] ]
            # data_to_send.KalmanGain = np.array(self.tracked_radar_objects[0].K_gain).flatten().tolist()
            # self.pub_kalman_state.publish(data_to_send)

            state = KalmanFilter_StateMsg()
            state.x = self.tracked_radar_objects[0].X_t.flatten().tolist()[0]
            state.y = self.tracked_radar_objects[0].X_t.flatten().tolist()[1]
            state.vx = self.tracked_radar_objects[0].X_t.flatten().tolist()[2]
            state.vy = self.tracked_radar_objects[0].X_t.flatten().tolist()[3]
            self.pub_kalman_state.publish(state)

            # state cov
            state_cov = KalmanFilter_StateMsg()
            state_cov.x = self.tracked_radar_objects[0].P_t[0,0]
            state_cov.y = self.tracked_radar_objects[0].P_t[1,1]
            state_cov.vx = self.tracked_radar_objects[0].P_t[2,2]
            state_cov.vy = self.tracked_radar_objects[0].P_t[3,3]

            self.pub_kalman_state_cov.publish(state_cov)

            # control input
            control_input = KalmanFilter_StateMsg()
            control_input.x = 0#self.tracked_radar_objects[0].U_t.flatten().tolist()[0]
            control_input.y = 0#self.tracked_radar_objects[0].U_t.flatten().tolist()[1]
            control_input.vx = 0#self.tracked_radar_objects[0].U_t.flatten().tolist()[2]
            control_input.vy = 0#self.tracked_radar_objects[0].U_t.flatten().tolist()[3]

            self.pub_kalman_control_input.publish(control_input)

        self.last_update_ts = current_time
