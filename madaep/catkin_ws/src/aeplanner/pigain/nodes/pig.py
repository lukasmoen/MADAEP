#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Point

from pigain.msg import Node, Score, Nodes
from pigain.srv import Query, QueryResponse
from pigain.srv import BestNode, BestNodeResponse
from aeplanner.srv import Reevaluate

import numpy as np
from rtree import index

#Visualization of dynamic_gain
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib as mpl
import gp
from geometry_msgs.msg import Quaternion


class PIGain:
    def __init__(self):
        self.s = rospy.Service('gp_query_server', Query, self.query_server)
        self.nn_yaw_srv = rospy.Service('nn_yaw_query_server', Query, self.nn_yaw_angle_srv_callback)

        self.best_node_srv = rospy.Service('best_node_server', BestNode, self.best_node_srv_callback)
        self.reevaluate_client = rospy.ServiceProxy('reevaluate', Reevaluate)

        self.visualize_mean  = rospy.get_param('~visualize/mean',  False)
        self.visualize_sigma = rospy.get_param('~visualize/sigma', False)
        self.visualize_pts   = rospy.get_param('~visualize/pts',   False)
        self.resolution      = rospy.get_param('~visualize/resolution',   1)

        self.gain_sub = rospy.Subscriber('gain_node', Node, self.gain_callback)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.marker_pub = rospy.Publisher('pig_markers', MarkerArray, queue_size=10)
        self.yaw_marker_pub = rospy.Publisher('yaw_markers', MarkerArray, queue_size=10)
        self.mean_pub = rospy.Publisher('mean_markers', MarkerArray, queue_size=10)
        self.sigma_pub = rospy.Publisher('sigma_markers', MarkerArray, queue_size=10)

        self.timer_share = rospy.Timer(rospy.Duration(5), self.share_gain_callback)
        self.gain_pub = rospy.Publisher('/shared_gain', Nodes, queue_size=10)
        self.shared_gain_sub = rospy.Subscriber('/shared_gain', Nodes, self.shared_gain_callback)
        self.removed_gain_pub = rospy.Publisher("/shared_removed_gain", Nodes, queue_size=10)
        self.removed_gain_sub = rospy.Subscriber("/shared_removed_gain", Nodes, self.shared_removed_gain_callback)

        rospy.Timer(rospy.Duration(5), self.cluster_timer_callback)

        self.namespace = "NotYetSet"
        self.removed_nodes = set()
        self.removed_nodes_obj = Nodes()

        # DAEP
        self.cache_node_threshold = rospy.get_param('daep/gain/cache_node_threshold', 30)
        self.node_gain_threshold = rospy.get_param('daep/gain/node_gain_threshold', 10)

        if self.visualize_pts:
            rospy.Timer(rospy.Duration(0.5), self.rviz_callback)
        if self.visualize_mean or self.visualize_sigma:
            rospy.Timer(rospy.Duration(5), self.evaluate)

        # Get environment boundaries 
        try:
            self.min = rospy.get_param('boundary/min')
            self.max = rospy.get_param('boundary/max')
        except KeyError:
            rospy.logwarn("Boundary parameters not specified")
            rospy.logwarn("Defaulting to (-100, -100, 0), (100, 100, 3)...")
            self.min = [-100, -100, 0]
            self.max = [ 100,  100, 3]

        try:
            self.range = rospy.get_param('aep/gain/r_max') * 2
        except KeyError:
            rospy.logwarn("Range max parameter not specified")
            rospy.logwarn("Defaulting to 8 m...")

        self.bbx = (self.min[0], self.min[1], self.min[2], self.max[0], self.max[1], self.max[2])

        self.x = None
        self.y = None
        self.z = None

        self.hyperparam = gp.HyperParam(l = 1, sigma_f = 1, sigma_n = 0.1)

        # Create r-tree
        p = index.Property()
        p.dimension = 3
        self.idx = index.Index(properties = p)
        self.id = 0

        rospy.Timer(rospy.Duration(0.5), self.reevaluate_timer_callback)

    def cluster_timer_callback(self, event):
        # return
        self.cluster_nodes()
    
    def cluster_nodes(self, threshold_distance=1.5):
        """
        Improved clustering algorithm with better gain handling and more aggressive merging
        """
        # Get all nodes from the R-tree
        all_nodes = list(self.idx.intersection(self.bbx, objects=True))
        
        if len(all_nodes) <= 1:
            return
            
        # Sort nodes by gain (highest first) to prioritize high-gain nodes as cluster centers
        all_nodes.sort(key=lambda item: item.object.gain, reverse=True)
        
        # Keep track of nodes we've already clustered
        processed_ids = set()
        clusters = []
        
        # Process each node
        for item in all_nodes:
            if item.id in processed_ids:
                continue
                
            # This node becomes a cluster center
            cluster_nodes = [item]
            processed_ids.add(item.id)
            
            # Find nearby nodes to add to this cluster
            center_pos = (item.object.position.x, item.object.position.y, item.object.position.z)
            
            # Define bounding box around this node
            nearby_box = (
                center_pos[0] - threshold_distance, 
                center_pos[1] - threshold_distance, 
                center_pos[2] - threshold_distance,
                center_pos[0] + threshold_distance, 
                center_pos[1] + threshold_distance, 
                center_pos[2] + threshold_distance
            )
            
            # Find neighboring nodes
            for neighbor in self.idx.intersection(nearby_box, objects=True):
                if neighbor.id in processed_ids:
                    continue
                    
                neighbor_pos = (neighbor.object.position.x, neighbor.object.position.y, neighbor.object.position.z)
                
                # Calculate Euclidean distance
                dist = np.sqrt(
                    (center_pos[0] - neighbor_pos[0])**2 + 
                    (center_pos[1] - neighbor_pos[1])**2 + 
                    (center_pos[2] - neighbor_pos[2])**2
                )
                
                if dist <= threshold_distance:
                    cluster_nodes.append(neighbor)
                    processed_ids.add(neighbor.id)
            
            clusters.append(cluster_nodes)
        
        # Now rebuild the R-tree with clustered nodes
        # Create a new R-tree
        p = index.Property()
        p.dimension = 3
        new_idx = index.Index(properties=p)
        new_id = 0
        
        # For each cluster, create a new representative node
        for cluster in clusters:
            if len(cluster) == 1:
                # If cluster has only one node, just add it directly
                item = cluster[0]
                new_idx.insert(new_id, 
                            (item.object.position.x, item.object.position.y, item.object.position.z), 
                            obj=item.object)
                new_id += 1
            else:
                # Calculate the average based on a gain-weighted approach
                total_gain = sum(item.object.gain for item in cluster)
                
                # Skip cluster if total gain is 0
                if total_gain <= 0:
                    continue
                    
                # Initialize values for weighted average
                avg_x, avg_y, avg_z = 0, 0, 0
                avg_yaw_sin, avg_yaw_cos = 0, 0  # For proper circular averaging of yaw
                max_gain = 0
                max_gain_item = None
                avg_gain = 0
                
                # Calculate weighted contributions
                for item in cluster:
                    weight = item.object.gain / total_gain
                    avg_gain += item.object.gain * weight
                    avg_x += item.object.position.x * weight
                    avg_y += item.object.position.y * weight
                    avg_z += item.object.position.z * weight
                    
                    # Handle yaw properly using trigonometry for circular values
                    avg_yaw_sin += np.sin(item.object.yaw) * weight
                    avg_yaw_cos += np.cos(item.object.yaw) * weight
                    
                    # Track the item with maximum gain
                    if item.object.gain > max_gain:
                        max_gain = item.object.gain
                        max_gain_item = item
                
                # Create a new node with the averaged values
                new_node = Node()
                new_node.position = Point(avg_x, avg_y, avg_z)
                new_node.gain = avg_gain  # Use weighted average gain instead of maximum
                
                # Calculate average yaw correctly
                new_node.yaw = np.arctan2(avg_yaw_sin, avg_yaw_cos)
                
                # Use time info from highest gain node
                if max_gain_item:
                    new_node.time = max_gain_item.object.time
                    new_node.time_since = max_gain_item.object.time_since
                    new_node.dynamic_gain = max_gain_item.object.dynamic_gain
                
                # Only insert if gain is above threshold
                if new_node.gain >= self.node_gain_threshold:
                    new_idx.insert(new_id, (avg_x, avg_y, avg_z), obj=new_node)
                    new_id += 1
        
        # Replace the old R-tree with the new one
        self.idx = new_idx
        self.id = new_id


    def shared_gain_callback(self, msg):
        """
        Insert nodes from other agents with better deduplication and updating
        """
        if self.namespace == "NotYetSet" or msg.name == self.namespace:
            return
            
        # Convert removed nodes to a more efficient data structure
        removed_positions = {(round(n.position.x, 3), round(n.position.y, 3), round(n.position.z, 3)) 
                            for n in self.removed_nodes_obj.nodes}
        
        # Track nodes that need clustering after update
        modified_nodes = False
        
        for node in msg.nodes:
            # Skip if gain too low
            if node.gain < self.node_gain_threshold:
                continue
                
            # Round positions for consistent comparison
            pos = (round(node.position.x, 3), round(node.position.y, 3), round(node.position.z, 3))
            
            # Skip if in removed nodes
            if pos in removed_positions:
                continue
            
            # Define a generous search bounding box to find potentially related nodes
            search_bbox = (
                pos[0] - 2.0,  # Increased search radius
                pos[1] - 2.0,
                pos[2] - 2.0,
                pos[0] + 2.0,
                pos[1] + 2.0,
                pos[2] + 2.0
            )
            
            nearby_nodes = list(self.idx.intersection(search_bbox, objects=True))
            
            # If no nearby nodes, just add this one
            if not nearby_nodes:
                self.idx.insert(self.id, (node.position.x, node.position.y, node.position.z), obj=node)
                self.id += 1
                modified_nodes = True
                continue
            
            # First look for exact position matches (within a small epsilon)
            exact_match_found = False
            for item in nearby_nodes:
                item_pos = (item.object.position.x, item.object.position.y, item.object.position.z)
                
                # Check for almost identical position (with epsilon)
                if (abs(pos[0] - item_pos[0]) < 0.1 and 
                    abs(pos[1] - item_pos[1]) < 0.1 and 
                    abs(pos[2] - item_pos[2]) < 0.1):
                    
                    # Update gain if appropriate - take lowest gain between nodes
                    if abs(node.gain - item.object.gain) > 5.0:  # Only update if significant difference
                        if node.gain < item.object.gain:
                            # Update with higher gain
                            self.idx.delete(item.id, item_pos)
                            self.idx.insert(item.id, 
                                        (node.position.x, node.position.y, node.position.z), 
                                        obj=node)
                            modified_nodes = True
                    
                    exact_match_found = True
                    break
            
            # If we found an exact position match, continue to next node
            if exact_match_found:
                continue
                
            # No exact match found, check for nearby nodes (within 1.5m)
            merge_candidate = None
            min_dist = 1.5  # Threshold for merging
            
            for item in nearby_nodes:
                item_pos = (item.object.position.x, item.object.position.y, item.object.position.z)
                
                # Calculate Euclidean distance
                dist = np.sqrt(
                    (pos[0] - item_pos[0])**2 + 
                    (pos[1] - item_pos[1])**2 + 
                    (pos[2] - item_pos[2])**2
                )
                
                if dist < min_dist:
                    min_dist = dist
                    merge_candidate = item
            
            # If we have a merge candidate, potentially merge the nodes
            if merge_candidate:
                item = merge_candidate
                
                # Decide which node to keep based on gain
                if node.gain < item.object.gain - 10:  # Significantly better gain
                    # Replace with new node
                    self.idx.delete(item.id, 
                                (item.object.position.x, item.object.position.y, item.object.position.z))
                    self.idx.insert(item.id, 
                                (node.position.x, node.position.y, node.position.z), 
                                obj=node)
                    modified_nodes = True
                elif abs(node.gain - item.object.gain) <= 10:  # Similar gain, merge them
                    # Create merged node with weighted position based on gain
                    total_gain = node.gain + item.object.gain
                    weight1 = node.gain / total_gain
                    weight2 = item.object.gain / total_gain
                    
                    merged_node = Node()
                    merged_node.position = Point(
                        node.position.x * weight1 + item.object.position.x * weight2,
                        node.position.y * weight1 + item.object.position.y * weight2,
                        node.position.z * weight1 + item.object.position.z * weight2
                    )
                    merged_node.gain = max(node.gain, item.object.gain)  # Take max gain
                    
                    # Weighted yaw calculation
                    merged_node.yaw = node.yaw * weight1 + item.object.yaw * weight2
                    merged_node.time = max(node.time, item.object.time)  # Most recent time
                    merged_node.time_since = min(node.time_since, item.object.time_since)
                    
                    # Replace old node with merged node
                    self.idx.delete(item.id, 
                                (item.object.position.x, item.object.position.y, item.object.position.z))
                    self.idx.insert(item.id, 
                                (merged_node.position.x, merged_node.position.y, merged_node.position.z), 
                                obj=merged_node)
                    modified_nodes = True
                # Otherwise keep existing node (it has better gain)
            else:
                # No nearby nodes, add this one
                self.idx.insert(self.id, (node.position.x, node.position.y, node.position.z), obj=node)
                self.id += 1
                modified_nodes = True
    
        # After processing all nodes, cluster if changes were made
        if modified_nodes:
            self.cluster_nodes(threshold_distance=1.5)  # More aggressive clustering

    """ Keep only the most recent N removed nodes """
    def prune_removed_nodes(self):
        max_removed_nodes = 1000
        if len(self.removed_nodes) > max_removed_nodes:
            # Remove oldest entries
            nodes_to_remove = len(self.removed_nodes) - max_removed_nodes
            self.removed_nodes = set(list(self.removed_nodes)[nodes_to_remove:])
            self.removed_nodes_obj.nodes = self.removed_nodes_obj.nodes[nodes_to_remove:]

    def shared_removed_gain_callback(self, msg):
        """
        More effective handling of nodes cleared by other agents
        """
        if self.namespace == "NotYetSet":
            return
            
        for node in msg.nodes:
            # Define search box around the position
            pos = (node.position.x, node.position.y, node.position.z)
            
            # Use a larger search box to catch any nearby nodes
            search_box = (
                pos[0] - 0.5,  # 0.5m radius search
                pos[1] - 0.5,
                pos[2] - 0.5,
                pos[0] + 0.5,
                pos[1] + 0.5,
                pos[2] + 0.5
            )
            
            # Find nodes in this area
            nearby_nodes = list(self.idx.intersection(search_box, objects=True))
            
            # Remove all nodes in that area
            for item in nearby_nodes:
                item_pos = (item.object.position.x, item.object.position.y, item.object.position.z)
                
                # Calculate distance to verify it's really close
                dist = np.sqrt(
                    (pos[0] - item_pos[0])**2 + 
                    (pos[1] - item_pos[1])**2 + 
                    (pos[2] - item_pos[2])**2
                )
                
                if dist <= 0.5:  # Only delete if truly close
                    self.idx.delete(item.id, item_pos)
                    
                    # Add to our own removed nodes list to prevent re-adding
                    rounded_pos = (round(item_pos[0], 3), round(item_pos[1], 3), round(item_pos[2], 3))
                    self.removed_nodes.add(rounded_pos)
                    self.removed_nodes_obj.nodes.append(item.object)

    # """ Send nodes to other agents """
    def share_gain_callback(self, event):
        # Retrieve objects that intersect with the map's bounding box
        if self.namespace != "NotYetSet":
            self.cluster_nodes()
            self.prune_removed_nodes()

            hits = self.idx.intersection(self.bbx, objects=True)

            nodes = Nodes()
            nodes.name = self.namespace

            for item in hits:
                nodes.nodes.append(item.object)
            self.gain_pub.publish(nodes)

            nodes = Nodes()
            nodes.name = self.namespace
            nodes.nodes = self.removed_nodes_obj.nodes
            self.removed_gain_pub.publish(nodes)

    """ Save current pose of agent """
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
    
    """ Extract the nearest neighbour yaw angle"""
    def nn_yaw_angle_srv_callback(self, req):
       bbx = (req.point.x-2, req.point.y-2, req.point.z-2,
              req.point.x+2, req.point.y+2, req.point.z+2)
       # Find the nearest neighbor
       nearest = self.idx.nearest(bbx, 1, objects=True)
       # yaw of nearest neighbor
       yaw = 0
       for item in nearest:
           yaw = item.object.yaw

       response = QueryResponse()
       response.mu = 0
       response.sigma = 0
       response.yaw = yaw
       return response

    def reevaluate_timer_callback(self, event):
        """
        Improved node reevaluation with better gain management and node removal
        """
        if self.x is None or self.y is None or self.z is None:
            rospy.logwarn("No position received yet...")
            rospy.logwarn("Make sure that 'pose' has been correctly mapped and that it is being published")
            return

        # Define search area around current position 
        bbx = (self.x-self.range, self.y-self.range, self.z-self.range, 
            self.x+self.range, self.y+self.range, self.z+self.range)

        hits = self.idx.intersection(bbx, objects=True)

        reevaluate_list = []
        reevaluate_position_list = []
        nodes_to_remove = []

        for item in hits:
            if item.object.gain > self.cache_node_threshold:
                reevaluate_position_list.append(item.object.position)
                reevaluate_list.append(item)
            else:
                nodes_to_remove.append(item)

        # Handle node removal after the iteration is complete
        for item in nodes_to_remove:
            pos = (item.object.position.x, item.object.position.y, item.object.position.z)
            self.idx.delete(item.id, pos)
            rounded_pos = (round(pos[0], 3), round(pos[1], 3), round(pos[2], 3))
            self.removed_nodes.add(rounded_pos)
            self.removed_nodes_obj.nodes.append(item.object)
        
        # Skip the rest if nothing to reevaluate
        if not reevaluate_list:
            return
            
        try:
            res = self.reevaluate_client(reevaluate_position_list)
        except rospy.ServiceException as e:
            rospy.logerr("Calling reevaluate service failed: %s", str(e))
            return

        for i, item in enumerate(reevaluate_list):
            item.object.gain = res.gain[i]
            item.object.yaw = res.yaw[i]

            pos = (item.object.position.x, item.object.position.y, item.object.position.z)

            self.idx.delete(item.id, pos)
            if item.object.gain >= self.node_gain_threshold:
                self.idx.insert(item.id, pos, obj=item.object)
            else:
                # Add to removed nodes
                rounded_pos = (round(pos[0], 3), round(pos[1], 3), round(pos[2], 3))
                self.removed_nodes.add(rounded_pos)
                self.removed_nodes_obj.nodes.append(item.object)
        
        # Run a clustering pass after reevaluation to merge similar nodes
        # self.cluster_nodes(threshold_distance=1.5)


    """ Insert node with estimated gain in rtree """
    def gain_callback(self, msg):
        if(msg.gain > self.node_gain_threshold):
            self.idx.insert(self.id, (msg.position.x, msg.position.y, msg.position.z), obj=msg)
            self.id += 1

    """ Handle query to Gaussian Process """    
    def query_server(self, req):
        # The bounding box with offset 2 from the drone
        bbx = (req.point.x-2, req.point.y-2, req.point.z-2, 
               req.point.x+2, req.point.y+2, req.point.z+2)
        y = np.empty((0))
        x = np.empty((0,3))

        # Retrieve all objects that intersects with the bbox
        hits = self.idx.intersection(bbx, objects=True)

        # Get the gain and position of every goal
        for item in hits:
            # We havent evaluated this node in the last time_threshold seconds
            time_threshold = 15 # Fix me
            time_since = item.object.time_since
            if time_since < time_threshold:
                dynamic_gain = item.object.dynamic_gain
                time_of_arrival = item.object.time
                pos = [item.object.position.x, item.object.position.y, item.object.position.z]
                y = np.append(y, [item.object.gain], axis=0)
                x = np.append(x, [pos], axis = 0)

        # yaw of nearest neighbor
        nearest = self.idx.nearest(bbx, 1, objects=True)
        yaw = 0
        for item in nearest:
            yaw = item.object.yaw

        # No hits
        if y.shape[0] == 0:
            response = QueryResponse()
            response.mu = 0
            response.sigma = 1
            return response

        xstar = np.array([[req.point.x, req.point.y, req.point.z]])
        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)

        response = QueryResponse()
        response.mu = mean
        response.sigma = sigma
        response.yaw = yaw

        return response

    """ Return all nodes with gain higher than req.threshold """
    def best_node_srv_callback(self, req):
        # Retrieve objects that intersects with the bbox
        hits = self.idx.intersection(self.bbx, objects=True)

        best_gain = -1
        response = BestNodeResponse()
        for item in hits:
            if item.object.gain > req.threshold:
                response.best_node.append(item.object.position)
            if item.object.gain > best_gain:
                best_gain = item.object.gain

        response.gain = best_gain
        return response

    """ Evaluate potential information gain function over grid and publish it in rviz """
    def evaluate(self, event):
        y = np.empty((0))
        x = np.empty((0,3))
        xstar = np.empty((0,3))
        # Retrieve objects that intersects with the bbox
        hits = self.idx.intersection(self.bbx, objects=True)
        
        for item in hits:
            y = np.append(y, [item.object.gain], axis=0)
            x = np.append(x, [[item.object.position.x, item.object.position.y, item.object.position.z]], axis = 0)

        xt = np.arange(self.min[0], self.max[0], self.resolution)
        yt = np.arange(self.min[1], self.max[1], self.resolution)
        zt = [1]

        for xx in xt:
            for yy in yt:
                for zz in zt:
                    xstar = np.append(xstar, [[xx, yy, zz]], axis = 0)

        mean, sigma = gp.gp(y, x, xstar, self.hyperparam, gp.sqexpkernel)
        mean_markers = MarkerArray()
        sigma_markers = MarkerArray()
        for id, pts in enumerate(zip(xstar, mean, sigma)):
            mean_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[1], max(1-pts[2], 0)))
            # sigma_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[2] * 2))
        
        self.mean_pub.publish(mean_markers)
        self.sigma_pub.publish(sigma_markers)

    """ Publish all cached nodes in rviz """
    def rviz_callback(self, event):
        # Clear old markers by setting all existing ones to DELETE
        delete_markers = MarkerArray()
        for i in range(10000):  # Use a reasonable upper bound
            marker = Marker()
            marker.header.frame_id = "world"
            marker.id = i
            marker.action = Marker.DELETE
            delete_markers.markers.append(marker)
        
        self.marker_pub.publish(delete_markers)
        self.yaw_marker_pub.publish(delete_markers)
        
        # Then add new markers
        markers = MarkerArray()
        yaw_markers = MarkerArray()
        
        # Use a tracking set to ensure unique IDs
        used_ids = set()
        id_counter = 0
        
        hits = self.idx.intersection(self.bbx, objects=True)
        
        for item in hits:
            # Ensure unique IDs
            while id_counter in used_ids:
                id_counter += 1
            used_ids.add(id_counter)
            
            markers.markers.append(self.node_to_marker(id_counter, item.object))
            yaw_markers.markers.append(self.node_to_marker_yaw(id_counter, item.object))
            id_counter += 1

        self.marker_pub.publish(markers)
        self.yaw_marker_pub.publish(yaw_markers)

    def np_array_to_marker(self, id, p, v=0, a=0):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.1
        marker.color.r = v / 72.0
        marker.color.g = 0 
        marker.color.b = 0.5
        marker.color.a = a
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = p[0]
        marker.pose.position.y = p[1]
        marker.pose.position.z = p[2]
        marker.lifetime = rospy.Time(10)

        return marker


    def node_to_marker(self, id, node):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = node.gain / 72.0
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0

        marker.text = str(np.round(node.gain, 1))
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = node.position.x
        marker.pose.position.y = node.position.y
        marker.pose.position.z = node.position.z
        marker.lifetime = rospy.Time(1.2)

        return marker

    def node_to_marker_yaw(self, id, node):
        yaw = node.yaw
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = node.gain / 72.0
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        q = np.array([0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)])
        marker.pose.orientation = Quaternion(*q)
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = node.position.x
        marker.pose.position.y = node.position.y
        marker.pose.position.z = node.position.z + 0.5
        marker.lifetime = rospy.Time(1.2)

        return marker

if __name__ == '__main__':
    rospy.init_node('pigain', anonymous=True)
    pigain = PIGain()
    pigain.namespace = rospy.get_namespace()
    rospy.spin()
