#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelState, ModelStates


def marker_callback(msg):
    modelmsg = ModelState()
    modelmsg.model_name = "drone"+str(id)
    modelmsg.pose.position.x = msg.pose.position.x #+ spawn_offset[0]
    modelmsg.pose.position.y = msg.pose.position.y #+ spawn_offset[1]
    modelmsg.pose.position.z = msg.pose.position.z #+ spawn_offset[2]

    modelmsg.pose.orientation.x = msg.pose.orientation.x
    modelmsg.pose.orientation.y = msg.pose.orientation.y
    modelmsg.pose.orientation.z = msg.pose.orientation.z
    modelmsg.pose.orientation.w = msg.pose.orientation.w
    
    pose_pub.publish(modelmsg)
    
if __name__ == '__main__':
    rospy.init_node('odom_vis')
    id = rospy.get_param('~id')
    spawn_pos_str = rospy.get_param('~spawn_pos', "(0,0,0)")
    spawn_offset = tuple(map(float, spawn_pos_str.strip("()").split(",")))
    pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.Subscriber('/odom_visualization_'+str(id)+'/robot', Marker, marker_callback)
    
    rospy.spin()