#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from scipy.spatial import Voronoi
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class VoronoiGraph:
    def __init__(self):
        rospy.init_node("voronoi_graph")

        self.positions = {}
        self.drones = ["drone1", "drone2", "drone3", "drone4"]
        
        for drone in self.drones:
            rospy.Subscriber("/" + drone + "/odom", Odometry, self.odom_callback, drone)

        self.voronoi_clipped_pub = rospy.Publisher("/voronoi_graph_clipped", Marker, queue_size=10)
        self.voronoi_pub = rospy.Publisher("/voronoi_graph", Marker, queue_size=10)
        self.voronoi_boundary_pub = rospy.Publisher("/voronoi_graph_boundary", Marker, queue_size=10)
        
        self.x_min = rospy.get_param("/exploration_node_1/sdf_map/box_min_x", -10)
        self.x_max = rospy.get_param("/exploration_node_1/sdf_map/box_max_x", 10)
        self.y_min = rospy.get_param("/exploration_node_1/sdf_map/box_min_y", -10)
        self.y_max = rospy.get_param("/exploration_node_1/sdf_map/box_max_y", 10)
        rospy.Timer(rospy.Duration(5.0), lambda event: self.update_map_bounds())

        self.rate = rospy.Rate(10)  
        rospy.sleep(2)
        self.run()

    def update_map_bounds(self):
        self.x_min = rospy.get_param("/exploration_node_1/sdf_map/box_min_x", -10)
        self.x_max = rospy.get_param("/exploration_node_1/sdf_map/box_max_x", 10)
        self.y_min = rospy.get_param("/exploration_node_1/sdf_map/box_min_y", -10)
        self.y_max = rospy.get_param("/exploration_node_1/sdf_map/box_max_y", 10)
    
    def odom_callback(self, msg, topic):
        self.positions[topic] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def voronoi_to_rviz(self, vor, frame_id="world"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "voronoi"
        marker.id = 0
        marker.type = Marker.LINE_LIST  
        marker.action = Marker.ADD
        marker.scale.x = 0.05  
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  

        marker_clipped = marker
        marker_clipped.id = 2
        marker_clipped.color.a = 0.2

        # Add finite Voronoi edges
        for simplex in vor.ridge_vertices:
            if -1 not in simplex:  # Skip infinite edges
                p1, p2 = vor.vertices[simplex[0]], vor.vertices[simplex[1]]
                point1 = Point()
                point2 = Point()
                point1.x, point1.y, point1.z = p1[0], p1[1], 1.0
                point2.x, point2.y, point2.z = p2[0], p2[1], 1.0
                marker.points.append(point1)
                marker.points.append(point2)
                # point1, point2 = self.clip_edge_to_square(point1, point2)
                marker_clipped.points.append(point1)
                marker_clipped.points.append(point2)

        # Add infinite Voronoi edges (dashed lines in RViz)
        center = vor.points.mean(axis=0)
        ptp_bound = np.ptp(vor.points, axis=0)
        for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
            simplex = np.asarray(simplex)
            if np.any(simplex < 0):  # Handle infinite edges
                i = simplex[simplex >= 0][0]  # Get the finite vertex
                t = vor.points[pointidx[1]] - vor.points[pointidx[0]]  # Direction vector
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])  # Normal vector to the tangent
                midpoint = vor.points[pointidx].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                far_point = vor.vertices[i] + direction * ptp_bound.max() * 2  # Far point to make the edge visible
                
                point1 = Point()
                point2 = Point()
                point1.x, point1.y, point1.z = vor.vertices[i][0], vor.vertices[i][1], 1.0
                point2.x, point2.y, point2.z = far_point[0], far_point[1], 1.0
                marker.points.append(point1)
                marker.points.append(point2)
                # point1, point2 = self.clip_edge_to_square(point1, point2)
                marker_clipped.points.append(point1)
                marker_clipped.points.append(point2)

        return marker, marker_clipped

    def generate_voronoi(self):
         # boundary of map
        boundary_marker = Marker()
        boundary_marker.header.frame_id = "world"
        boundary_marker.header.stamp = rospy.Time.now()
        boundary_marker.ns = "boundary"
        boundary_marker.id = 1
        boundary_marker.type = Marker.LINE_LIST
        boundary_marker.action = Marker.ADD
        boundary_marker.scale.x = 0.05
        boundary_marker.color.r = 1.0  # Red color for boundary
        boundary_marker.color.g = 0.0
        boundary_marker.color.b = 0.0
        boundary_marker.color.a = 0.5  

        # Define the four corners of the square
        corners = [(self.x_min, self.y_min), (self.x_min, self.y_max),
                  (self.x_max, self.y_max), (self.x_max, self.y_min), (self.x_min, self.y_min)]
        
        # Create lines between the corners
        for i in range(len(corners) - 1):
            point1 = Point()
            point2 = Point()
            point1.x, point1.y, point1.z = corners[i][0], corners[i][1], 1.0
            point2.x, point2.y, point2.z = corners[i + 1][0], corners[i + 1][1], 1.0
            boundary_marker.points.append(point1)
            boundary_marker.points.append(point2)

        self.voronoi_boundary_pub.publish(boundary_marker)

        if len(self.positions) == 2:  
            # create line between the two positions manually
            drones = list(self.positions.values())
            mid_x = (drones[0][0] + drones[1][0]) / 2
            mid_y = (drones[0][1] + drones[1][1]) / 2
            
            dx = drones[1][0] - drones[0][0]
            dy = drones[1][1] - drones[0][1]
            
            if dx == 0:
                slope = None
            else:
                slope = -dx / dy if dy != 0 else None
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "boundary"
            marker.id = 0
            marker.type = Marker.LINE_LIST  
            marker.action = Marker.ADD
            marker.scale.x = 0.05  
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            if slope is None:  # Vertical line
                p1 = Point(mid_x, self.y_min, 1.0)
                p2 = Point(mid_x, self.y_max, 1.0)
            else:
                p1_x = self.x_min
                p1_y = mid_y + slope * (p1_x - mid_x)
                p2_x = self.x_max
                p2_y = mid_y + slope * (p2_x - mid_x)
                p1 = Point(p1_x, p1_y, 1.0)
                p2 = Point(p2_x, p2_y, 1.0)
            
            marker.points.append(p1)
            marker.points.append(p2)
            self.voronoi_pub.publish(marker)
            return

        if len(self.positions) < 2:
            return

        points = np.array(list(self.positions.values()))
        vor = Voronoi(points)

        marker, marker_clipped = self.voronoi_to_rviz(vor)
        self.voronoi_pub.publish(marker)
        self.voronoi_clipped_pub.publish(marker_clipped)


    def clip_edge_to_square(self, p1, p2):
        # Clips the Voronoi edge between points p1 and p2 to the boundaries of the square.
        
        def clip_line(p1, p2, min_val, max_val):
            new_p1 = Point()
            new_p1.x, new_p1.y, new_p1.z = p1.x,p1.y, 1.0
            new_p2 = Point()
            new_p2.x, new_p2.y, new_p2.z = p2.x, p2.y, 1.0

            if not (self.x_min <= p1.x <= self.x_max): # move first point
                slope = (p2.y - p1.y) / (p2.x - p1.x)
                intercept = p1.y - slope * p1.x
                # Find intersection points with the boundary
                if p1 > max_val:
                    new_p1_x = max_val
                    new_p1_y = slope * max_val + intercept

                if p1 < min_val:
                    new_p1_x = min_val
                    new_p1_y = slope * min_val + intercept

                new_p1.x, new_p1.y = new_p1_x, new_p1_y
                

            if not (self.x_min <= p2.x <= self.x_max): # move second point
                slope = (p2.y - p1.y) / (p2.x - p1.x)
                intercept = p1.y - slope * p1.x
                # Find intersection points with the boundary
                if p2 > max_val:
                    new_p2_x = max_val
                    new_p2_y = slope * max_val + intercept

                if p2 < min_val:
                    new_p2_x = min_val
                    new_p2_y = slope * min_val + intercept
                
                new_p2.x, new_p2.y = new_p2_x, new_p2_y

            if not (self.y_min <= p1.y <= self.y_max): # move first point
                slope = (p2.x - p1.x) / (p2.y - p1.y)
                intercept = p1.x - slope * p1.y
                # Find intersection points with the boundary
                if p1 > max_val:
                    new_p1_y = max_val
                    new_p1_x = slope * max_val + intercept

                if p1 < min_val:
                    new_p1_y = min_val
                    new_p1_x = slope * min_val + intercept

                new_p1.x, new_p1.y = new_p1_x, new_p1_y

            if not(self.y_min <= p2.y <= self.y_max): # move second point
                slope = (p2.x - p1.x) / (p2.y - p1.y)
                intercept = p1.x - slope * p1.y
                # Find intersection points with the boundary
                if p2 > max_val:
                    new_p2_y = max_val
                    new_p2_x = slope * max_val + intercept

                if p2 < min_val:
                    new_p1_y = min_val
                    new_p1_x = slope * min_val + intercept

                new_p2.x, new_p2.y = new_p2_x, new_p2_y
        

            return new_p1, new_p2

        # if both points outside boundary (visualize nothing)
        if not (self.y_min <= p1.y <= self.y_max and self.y_min <= p2.y <= self.y_max) and not (self.x_min <= p1.x <= self.x_max and self.x_min <= p2.x <= self.x_max):
                p1 = Point(0.0,0.0,0.0)
                p2 = Point(0.0,0.0,0.0)
                return p1,p2
    
        # if atleast one point outside boundary (cut one side of line)
        p1, p2 = clip_line(p1, p2, self.x_min, self.x_max)

        return p1, p2

    def run(self):
        while not rospy.is_shutdown():
            self.generate_voronoi()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        VoronoiGraph()
    except rospy.ROSInterruptException:
        pass
