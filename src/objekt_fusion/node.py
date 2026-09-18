from logging import debug

import geometry_msgs.msg
import rospy

import numpy as np
from tf.transformations import quaternion_from_euler

from rospy.core import logdebug, logwarn, rospyinfo

from visualization_msgs.msg import Marker, MarkerArray
from simulation_groundtruth.msg import ObjectHypothesis as ObjectHypothesisMsg
from simulation_groundtruth.msg import ObjectHypothesisList as ObjectHypothesisListMsg


from simulation.utils.geometry import Vector, Point
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
        self.last_update_ts = 0

        super().start()

    def stop(self):
        self.pub_objectlist_fused.unregister()
        self.pub_objectlist_vis_fused_pos.unregister()
        self.pub_objectlist_vis_fused_vel.unregister()
        
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
        
        ########################################################
        # Ihr Code steht hier: simple fusion
        # Ihre Fusionsergebnisse können Sie eintragen mit 
        # fused_object_list.append(ObjectHypothesis)
        ########################################################

        # init variable for results
        fused_object_list  = ObjectHypothesisListMsg() 
        # Update current time
        current_time = self.latest_radar_objects.timestamp
        delta_t = current_time - self.last_update_ts
        if delta_t < 1e-6:
            return

        # Beispiel: Zugriff auf Radar und Kamera-Objektlisten:
        # Daten sind gespeichert in :
        # self.latest_radar_objects
        # self.latest_camera_objects
        # 
        # Beispiel: Zugriff in for-Schleife:
        # for cam_obj in self.latest_camera_objects.objectlist:
        #     cam_obj : ObjectHypothesisMsg = cam_obj     # trick to enable python autocompletion in loops
        #     # do someting here
        #     ObjectHypothesis = cam_obj                  # copy cam_obj data to ObjectHypothesis
        #     fused_object_list.append(ObjectHypothesis)  # append to output list

        fused_object_list = self.latest_radar_objects
        
        #
        #
        #
        #
        #
        #
        #

        fused_object_list.timestamp = current_time
        
        ###### don't touch ###########
        # visualize all object lists #
        ##############################
        self.pub_objectlist_fused.publish(fused_object_list)
        
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
            vis_objects.scale.x = abs(Vector(obj.velocity))*1
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

        self.last_update_ts = current_time