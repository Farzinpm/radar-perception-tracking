from pyquaternion.quaternion import Quaternion

import geometry_msgs.msg
import rospy

import numpy as np
from pyquaternion.quaternion import Quaternion
from tf.transformations import quaternion_from_euler

from rospy.core import logdebug, logwarn, rospyinfo

from visualization_msgs.msg import Marker, MarkerArray
from simulation_groundtruth.msg import ObjectHypothesis as ObjectHypothesisMsg
from simulation_groundtruth.msg import ObjectHypothesisList as ObjectHypothesisListMsg
from lab_simulation_core.msg import RadarMeasurement as ReadarMeasurementMsg
from lab_simulation_core.msg import RadarMeasurementList as RadarMeasurementListMsg

from simulation.utils.geometry import Pose, Transform, Vector, Point, transform
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
        self.tracked_radar_objects = ObjectHypothesisListMsg() # needed for task 2: association

        super().start()

    def stop(self):
        self.pub_objectlist_radar.unregister()
        self.pub_objectlist_vis_radar_pos.unregister()
        self.pub_objectlist_vis_radar_vel.unregister()
        
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
        current_time = rospy.Time.now().to_sec()
        
        ########################################################
        # Ihr Code steht hier: Radar Tracking
        # Ihre Tracking-Ergebnisse können Sie eintragen mit 
        # radar_object_list.objectlist.append(ObjectHypothesis)
        ########################################################
        #rospy.logdebug("LOL")
        radar_measurements = self.latest_radar_rawdata.measurements

        # Ihr Ergebnis wird in diese Variable gefüllt:
        radar_object_list  = ObjectHypothesisListMsg() 
        radar_object_list.timestamp = self.latest_radar_rawdata.timestamp

        # siehe Laborskript: Koordinaten-Transformation über Sensor-Verbauposition in Fahrzeugkoordinaten
        pose = Pose(Point(0.25, 0, 0), Quaternion(1,0,0,0))
        tf = Transform(pose)

        
        for radar_meas in radar_measurements:
            radar_meas : ReadarMeasurementMsg = radar_meas
            ObjectHypothesis = ObjectHypothesisMsg()

            #ToDo: ergänzen Sie hier die Umrechnungen in kartesische Koordinaten - ersetzen von 0, 0, 0

            phi = radar_meas.hor_angle 
            range = radar_meas.range
            speed = radar_meas.radial_velocity
            x = np.cos(phi) * range + 0.25
            y = np.sin(phi) * range 
            vx = np.cos(phi) * speed 
            vy = np.sin(phi) * speed 
            ObjectHypothesis.position = Point( x, y, 0)
            ObjectHypothesis.velocity = Point( vx, vy, 0)
            ObjectHypothesis.last_update_ts_s = self.latest_radar_rawdata.timestamp #ToDo: hier richtigen Zeitstempel ergänzen
            
            #ToDo: dann diese Zeile einkommentieren
            radar_object_list.objectlist.append(ObjectHypothesis)

        ### Aufgabe 1.2.1 : Assoziation

        for obj_hypo in radar_object_list.objectlist:
            associated = False

            for obj_track in self.tracked_radar_objects.objectlist:

                if abs(obj_track.position - Vector(obj_hypo.position)) < 1.3:
                    rospy.logdebug("associated")
                    associated = True 
                    id = self.tracked_radar_objects.objectlist.index(obj_track)
                    self.tracked_radar_objects.objectlist[id] = obj_hypo
                    break

            if associated == False:
                self.tracked_radar_objects.objectlist.append(obj_hypo)


        ### Aufgabe 1.2.2 : Housekeeping

        for obj_track in self.tracked_radar_objects.objectlist:

            if abs(obj_track.last_update_ts_s - radar_object_list.timestamp) > 0.2:
                self.tracked_radar_objects.objectlist.remove(obj_track)


         
        radar_object_list = self.tracked_radar_objects

        # Ihr Ergebnis wird hier verschickt
        self.pub_objectlist_radar.publish(radar_object_list)

        # Länge Objektliste
        rospy.logdebug("number of tracked objects: \%u", len(self.tracked_radar_objects.objectlist))

        ### GEÄNDERT ENDE

        ##############################
        # visualize all object lists #
        ##############################

        # fused object list
        ObjectCenterPoints = [] 
        for obj in radar_object_list.objectlist:
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
        
        vis_objectList = MarkerArray()

        for obj in radar_object_list.objectlist:
            vis_objects = Marker(type=Marker.ARROW)

            vis_objects.header.frame_id = self.param.vehicle_simulation_link.frame.vehicle
            vis_objects.id = radar_object_list.objectlist.index(obj)
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
            vis_objectList.markers.append(vis_objects)
        
        self.pub_objectlist_vis_radar_vel.publish(vis_objectList)