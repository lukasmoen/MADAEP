#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped
from pigain.srv import QueryDFM, QueryDFMResponse
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelState, ModelStates
from matplotlib import colors
from mpl_toolkits.mplot3d import Axes3D
from visualization_msgs.msg import Marker, MarkerArray

class DynamicFrequencyMap:
    def __init__(self):
        # Existing parameters
        self.dfm_query_srv = rospy.Service('dfm_query_server', QueryDFM, self.dfm_callback)
        self.min_x = rospy.get_param('/drone1/boundary/min', [-15, -15, 0])[0]
        self.max_x = rospy.get_param('/drone1/boundary/max', [15, 15, 2.5])[0]
        self.min_y = rospy.get_param('/drone1/boundary/min', [-15, -15, 0])[1]
        self.max_y = rospy.get_param('/drone1/boundary/max', [15, 15, 2.5])[1]
        self.min_z = rospy.get_param('/drone1/boundary/min', [-15, -15, 0])[2]
        self.max_z = rospy.get_param('/drone1/boundary/max', [15, 15, 2.5])[2]
        
        self.visualizeDFM = rospy.get_param('/drone1/visualizeDFM', True)
        self.square_size = rospy.get_param('/drone1/drone1_dfm/square_size', 0.5)
        self.max_val = rospy.get_param('/drone1/dfm_max_value',1)
        
        # Round all boundaries
        self.round_min_x = self.round(self.min_x)
        self.round_max_x = self.round(self.max_x)
        self.round_min_y = self.round(self.min_y)
        self.round_max_y = self.round(self.max_y)
        self.round_min_z = self.round(self.min_z)
        self.round_max_z = self.round(self.max_z)
        
        # Calculate sizes
        self.x_size = self.round_max_x - self.round_min_x
        self.y_size = self.round_max_y - self.round_min_y
        self.z_size = self.round_max_z - self.round_min_z
        
        # Calculate array dimensions
        self.array_size_x = int(self.x_size / self.square_size)
        self.array_size_y = int(self.y_size / self.square_size)
        self.array_size_z = max(1, int(self.z_size / self.square_size))  # Ensure at least 1 cell in z direction
        
        # Create 3D grid
        self.grid = np.zeros(shape=(self.array_size_x, self.array_size_y, self.array_size_z))
        
        # Rest of initialization
        self.dynamic_objects = {}
        self.agents = {}
        
        # Movement tracking for both agents and dynamic objects
        self.agent_last_positions = {}  # Store last known position of each agent
        self.agent_stationary_since = {}  # Timestamp when agent became stationary
        self.human_last_positions = {}  # Store last known position of each human
        self.human_stationary_since = {}  # Timestamp when human became stationary
        self.stationary_threshold = 0.01  # Movement threshold (in meters) to consider as stationary
        self.stationary_time_limit = 20.0 
        
        self.point_sub = rospy.Subscriber('updateDFM', PointStamped, self.update_frequency_map_callback)
        self.dynamic_object_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.update_human_positions)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_timer_callback)
        self.namespace = rospy.get_namespace()
        topic_name = self.namespace + 'dfm_visualization'
        self.marker_publisher = rospy.Publisher(topic_name, MarkerArray, queue_size=10)
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)  # Update markers every 1 second

        # Set up 3D visualization
        if self.visualizeDFM:
            self.setup_3d_visualization()

  
    def publish_markers(self, timer):
        """Publish the grid as markers in Gazebo"""
        marker_array = MarkerArray()
        normalized_grid = self.normalize_grid()
        
        marker_id = 0
        threshold = 0.1  # Only show cells with values above this threshold
        
        # Create markers for each significant cell in the grid
        for x in range(self.array_size_x):
            for y in range(self.array_size_y):
                for z in range(self.array_size_z):
                    value = normalized_grid[x, y, z]
                    if abs(value) > threshold:
                        # Create a marker for this cell
                        marker = Marker()
                        marker.header.frame_id = "world"  # Use the appropriate frame_id
                        marker.header.stamp = rospy.Time.now()
                        marker.ns = "dfm_grid"
                        marker.id = marker_id
                        marker_id += 1
                        
                        # Cube marker
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD
                        
                        # Position (convert from grid to world coordinates)
                        marker.pose.position.x = self.round_min_x + (x + 0.5) * self.square_size
                        marker.pose.position.y = self.round_min_y + (y + 0.5) * self.square_size
                        marker.pose.position.z = self.round_min_z + (z + 0.5) * self.square_size
                        
                        # Orientation (identity quaternion)
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0
                        
                        # Scale - make markers 1m x 1m
                        target_size = 1.0  # 1m x 1m markers
                        # Scale based on the square_size to get 1m x 1m markers
                        scale_factor = target_size / self.square_size
                        marker.scale.x = self.square_size * scale_factor
                        marker.scale.y = self.square_size * scale_factor
                        marker.scale.z = self.square_size * scale_factor
                        
                        # Color based on value (blue for humans, red for agents)
                        marker.color.a = min(0.5, abs(value) * 0.5)  # Low opacity (max 0.3)
                        
                        if value > 0:  # Human presence (blue)
                            marker.color.r = 0.0
                            marker.color.g = 0.0
                            marker.color.b = 1.0
                        else:  # Agent presence (red)
                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                        
                        # Set marker lifetime
                        marker.lifetime = rospy.Duration(1.5)  # Slightly longer than update interval
                        
                        marker_array.markers.append(marker)
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)

    # Round map boundaries 
    def round(self, num):
        if num < 0:
            return int(np.floor(num))
        else:
            return int(np.ceil(num))
    
    def setup_3d_visualization(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initial empty plot for voxels
        self.voxel_plot = None
        self.scatter_plot = None
        
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.set_title('3D Dynamic Frequency Map')
        
        # Set axis limits
        self.ax.set_xlim([0, self.x_size])
        self.ax.set_ylim([0, self.y_size])
        self.ax.set_zlim([0, self.z_size])
        
        self.ani = FuncAnimation(self.fig, self.update_3d_plot, interval=500)
        plt.show()

    def update_3d_plot(self, frame):
        # Clear previous plot
        if hasattr(self, 'voxel_collection') and self.voxel_collection:
            for collection in self.ax.collections:
                self.ax.collections.remove(collection)
        
        # Normalize grid for visualization
        normalized_grid = self.normalize_grid()
        
        # Create mask for which voxels to display (those with values above threshold)
        voxels_to_show = np.abs(normalized_grid) > 0.1
        
        # Create color array directly from normalized grid
        colors = np.zeros(normalized_grid.shape + (4,))  # Add dimension for RGBA
        
        # Set blue for positive values (humans)
        pos_mask = normalized_grid > 0.0
        colors[pos_mask, 2] = 1.0  # B
        colors[pos_mask, 3] = np.clip(normalized_grid[pos_mask], 0.1, 1.0)  # Alpha
        
        # Set red for negative values (agents)
        neg_mask = normalized_grid < 0.0
        colors[neg_mask, 0] = 1.0  # R
        colors[neg_mask, 3] = np.clip(-normalized_grid[neg_mask], 0.1, 1.0)  # Alpha
        
        # Plot voxels directly using the mask and colors
        voxel_plot = self.ax.voxels(
            voxels_to_show,
            facecolors=colors,
            edgecolor='k',
            linewidth=0.3
        )
        
        self.voxel_collection = voxel_plot
        
        return voxel_plot

    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        return np.sqrt((pos1[0] - pos2[0])**2 + 
                       (pos1[1] - pos2[1])**2 + 
                       (pos1[2] - pos2[2])**2)

    def update_human_positions(self, model_states):
        # Current timestamp
        current_time = rospy.Time.now().to_sec()
        
        # Store previous positions before clearing
        previous_humans = self.dynamic_objects.copy()
        previous_agents = self.agents.copy()
        
        # Clear current positions
        self.dynamic_objects.clear()
        self.agents.clear()
        
        # Process humans
        for model in model_states.name:
            if "person_walking" in model:
                person_idx = model_states.name.index(model)
                person_pose = model_states.pose[person_idx]
                x = person_pose.position.x
                y = person_pose.position.y
                z = person_pose.position.z

                if (self.min_x <= x <= self.max_x and 
                    self.min_y <= y <= self.max_y and
                    self.min_z <= z <= self.max_z):
                    self.dynamic_objects[model] = person_pose
                    
                    # Get current position as tuple for easy comparison
                    current_pos = (x, y, z)
                    
                    # If this is the first time we see this human, initialize tracking
                    if model not in self.human_last_positions:
                        self.human_last_positions[model] = current_pos
                        # Initialize as moving
                        self.human_stationary_since[model] = None
                    else:
                        # Check if the human has moved
                        last_pos = self.human_last_positions[model]
                        distance_moved = self.calculate_distance(current_pos, last_pos)
                        
                        if distance_moved > self.stationary_threshold:
                            # Human is moving
                            self.human_last_positions[model] = current_pos
                            self.human_stationary_since[model] = None  # Reset stationary timer
                        elif self.human_stationary_since[model] is None:
                            # Human just became stationary
                            self.human_stationary_since[model] = current_time
                            self.human_last_positions[model] = current_pos  # Update position anyway
                        else:
                            # Human was already stationary, update position for accuracy
                            self.human_last_positions[model] = current_pos
        
        # Process agents and track their movement
        for model in model_states.name:
            if "drone" in model:
                agent_idx = model_states.name.index(model)
                agent_pose = model_states.pose[agent_idx]
                x = agent_pose.position.x
                y = agent_pose.position.y
                z = agent_pose.position.z

                if (self.min_x <= x <= self.max_x and 
                    self.min_y <= y <= self.max_y and
                    self.min_z <= z <= self.max_z):
                    self.agents[model] = agent_pose
                    
                    # Get current position as tuple for easy comparison
                    current_pos = (x, y, z)
                    
                    # If this is the first time we see this agent, initialize tracking
                    if model not in self.agent_last_positions:
                        self.agent_last_positions[model] = current_pos
                        # Initialize as moving
                        self.agent_stationary_since[model] = None
                    else:
                        # Check if the agent has moved
                        last_pos = self.agent_last_positions[model]
                        distance_moved = self.calculate_distance(current_pos, last_pos)
                        
                        if distance_moved > self.stationary_threshold:
                            # Agent is moving
                            self.agent_last_positions[model] = current_pos
                            self.agent_stationary_since[model] = None  # Reset stationary timer
                        elif self.agent_stationary_since[model] is None:
                            # Agent just became stationary
                            self.agent_stationary_since[model] = current_time
                            self.agent_last_positions[model] = current_pos  # Update position anyway
                        else:
                            # Agent was already stationary, update position for accuracy
                            self.agent_last_positions[model] = current_pos
        
        # Clean up tracking for objects that disappeared
        # Check for humans that were present before but are no longer in the list
        for model in previous_humans:
            if model not in self.dynamic_objects:
                # Human disappeared, remove from tracking
                if model in self.human_last_positions:
                    del self.human_last_positions[model]
                if model in self.human_stationary_since:
                    del self.human_stationary_since[model]
                    
        # Check for agents that were present before but are no longer in the list
        for model in previous_agents:
            if model not in self.agents:
                # Agent disappeared, remove from tracking
                if model in self.agent_last_positions:
                    del self.agent_last_positions[model]
                if model in self.agent_stationary_since:
                    del self.agent_stationary_since[model]

    def update_frequency_map_callback(self, msg):
        # Check if the PointStamped has a z component
        try:
            observation = (msg.point.x, msg.point.y, msg.point.z)
            has_z = True
        except AttributeError:
            # Fall back to 2D if no z coordinate
            observation = (msg.point.x, msg.point.y)
            has_z = False
            
        x = int((observation[0] - self.round_min_x) / self.square_size)
        y = int((observation[1] - self.round_min_y) / self.square_size)
        
        if has_z:
            z = int((observation[2] - self.round_min_z) / self.square_size)
        else:
            z = 0  # Default to ground level
        
        # Ensure indices are within bounds
        if (0 <= x < self.array_size_x and 
            0 <= y < self.array_size_y and 
            0 <= z < self.array_size_z):
            self.grid[x, y, z] += 1

    def is_stationary_too_long(self, model_name, is_human, current_time):
        """Check if an object has been stationary for more than the time limit"""
        if is_human:
            # Check for human stationary status
            if model_name in self.human_stationary_since and self.human_stationary_since[model_name] is not None:
                stationary_duration = current_time - self.human_stationary_since[model_name]
                return stationary_duration > self.stationary_time_limit
        else:
            # Check for agent stationary status
            if model_name in self.agent_stationary_since and self.agent_stationary_since[model_name] is not None:
                stationary_duration = current_time - self.agent_stationary_since[model_name]
                return stationary_duration > self.stationary_time_limit
        return False

    def update_timer_callback(self, timer):
        current_time = rospy.Time.now().to_sec()
        
        # Process humans
        for model, person in self.dynamic_objects.items():
            observation = (person.position.x, person.position.y, person.position.z)
            x = int((observation[0] - self.round_min_x) / self.square_size)
            y = int((observation[1] - self.round_min_y) / self.square_size)
            z = int((observation[2] - self.round_min_z) / self.square_size)
            
            if (0 <= x < self.array_size_x and 
                0 <= y < self.array_size_y and 
                0 <= z < self.array_size_z):
                
                # Only update frequency map if human is not stationary for too long
                if not self.is_stationary_too_long(model, True, current_time):
                    self.grid[x, y, z] += 1
        
        # Process agents
        for model, agent in self.agents.items():
            observation = (agent.position.x, agent.position.y, agent.position.z)
            x = int((observation[0] - self.round_min_x) / self.square_size)
            y = int((observation[1] - self.round_min_y) / self.square_size)
            z = int((observation[2] - self.round_min_z) / self.square_size)
            
            # Check if agent is within grid bounds
            if (0 <= x < self.array_size_x and 
                0 <= y < self.array_size_y and 
                0 <= z < self.array_size_z):
                
                # Only apply penalties if the agent is not stationary for too long
                if not self.is_stationary_too_long(model, False, current_time):
                    if self.grid[x,y,z] > -50:
                        self.grid[x, y, z] -= 1  # Increased penalty for agent positions
                            
                    # Apply -0.1 to all 26 neighbors
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            for dz in [-1, 0, 1]:
                                if dx == 0 and dy == 0 and dz == 0:
                                    continue  # Skip the center voxel
                                nx, ny, nz = x + dx, y + dy, z + dz
                                if (0 <= nx < self.array_size_x and 
                                    0 <= ny < self.array_size_y and 
                                    0 <= nz < self.array_size_z):
                                    if self.grid[nx, ny, nz] > -70:
                                        self.grid[nx, ny, nz] -= 0.1

    def normalize_grid(self):
        normalized_grid = np.zeros_like(self.grid)

        # Separate masks for positive and negative values
        pos_mask = self.grid > 0
        neg_mask = self.grid < 0

        # Get max positive and max negative values
        max_pos = np.max(self.grid[pos_mask]) if np.any(pos_mask) else 1.0
        max_neg = np.min(self.grid[neg_mask]) if np.any(neg_mask) else -1.0  # Note: min because it's negative

        # Normalize separately
        if max_pos != 0:
            normalized_grid[pos_mask] = self.grid[pos_mask] / max_pos
        if max_neg != 0:
            normalized_grid[neg_mask] = (self.grid[neg_mask] / abs(max_neg)) * 1.5
        return normalized_grid


    def dfm_callback(self, req):
        normalized_grid = self.normalize_grid()
        
        # Check if PointStamped has z coordinate
        try:
            z_coord = req.point.z
            has_z = True
        except AttributeError:
            has_z = False
            z_coord = 0  # Default to ground level
        
        x = int((req.point.x - self.round_min_x) / self.square_size)
        y = int((req.point.y - self.round_min_y) / self.square_size)
        
        if has_z:
            z = int((z_coord - self.round_min_z) / self.square_size)
        else:
            z = 0
        
        response = QueryDFMResponse()
        if (0 <= x < self.array_size_x and 
            0 <= y < self.array_size_y and 
            0 <= z < self.array_size_z):
            response.score = normalized_grid[x, y, z]
        else:
            response.score = 0.0  # Default score for out-of-bounds queries
        
        return response

if __name__ == '__main__':
    rospy.init_node('dynamic_frequency_map', anonymous=True)
    # Set logging level
    DFM = DynamicFrequencyMap()
    rospy.spin()