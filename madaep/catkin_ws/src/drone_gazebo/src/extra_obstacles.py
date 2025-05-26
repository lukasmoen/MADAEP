#!/usr/bin/env python

import uuid
import rospy
import random
import re
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState, ModelStates
from visualization_msgs.msg import Marker, MarkerArray
import ast

class Object():
    def __init__(self, name, speed, min_x, min_y, max_x, max_y, object_type, start_x=0.0, start_y=0.0, model_id=0):
        self.model_id = model_id
        self.model_name = "person_walking{}".format(model_id)
        self.name = name
        self.uuid = str(uuid.uuid4())
        self.x = start_x
        self.y = start_y
        self.z = 0.0
        self.speed = speed
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.object_type = object_type
        self.dx = speed if object_type == "dx" else 0.0
        self.dy = speed if object_type == "dy" else 0.0

        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.wait_timer = 0

    def tick(self, dt):
        if self.wait_timer > 0:
            self.wait_timer -= dt
            if self.wait_timer <= 0:
                if self.x >= self.max_x and self.object_type == "dx":
                    self.dx = -self.speed
                    self.qz = -0.707
                    self.qw = 0.707
                elif self.x <= self.min_x and self.object_type == "dx":
                    self.dx = self.speed
                    self.qz = 0.0
                    self.qw = 1.0
                if self.y >= self.max_y and self.object_type == "dy":
                    self.dy = -self.speed
                    self.qz = -1.0
                    self.qw = 0.0
                elif self.y <= self.min_y and self.object_type == "dy":
                    self.dy = self.speed
                    self.qz = 0.0
                    self.qw = 1.0
            return

        self.x += self.dx * dt
        self.y += self.dy * dt

        if self.x > self.max_x and self.object_type == "dx":
            self.x = self.max_x
            self.wait_timer = 5.0
        elif self.x < self.min_x and self.object_type == "dx":
            self.x = self.min_x
            self.wait_timer = 5.0

        if self.y > self.max_y and self.object_type == "dy":
            self.y = self.max_y
            self.wait_timer = 5.0
        elif self.y < self.min_y and self.object_type == "dy":
            self.y = self.min_y
            self.wait_timer = 5.0

    def get_model_state(self):
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.reference_frame = "world"
        model_state.pose.position.x = self.x
        model_state.pose.position.y = self.y
        model_state.pose.position.z = self.z
        model_state.pose.orientation.x = self.qx
        model_state.pose.orientation.y = self.qy
        model_state.pose.orientation.z = self.qz
        model_state.pose.orientation.w = self.qw
        model_state.twist.linear.x = self.dx
        model_state.twist.linear.y = self.dy
        model_state.twist.linear.z = 0.0
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0
        return model_state


class AnimateObjects():
    def __init__(self):
        rospy.init_node("animate_objects")

        self.human_linear_velocity = rospy.get_param('~human_linear_velocity', 0.35)
        self.human_angular_velocity = rospy.get_param('~human_angular_velocity', 1.0)
        min_boundary = rospy.get_param('/boundary/min', [-10.0, -10.0, 0.0])
        max_boundary = rospy.get_param('/boundary/max', [10.0, 10.0, 0.0])
        safe_limit = 5
        self.min_x = min_boundary[0] + safe_limit # shift limit to allow safe spawn position
        self.max_x = max_boundary[0] - safe_limit
        self.min_y = min_boundary[1] + safe_limit
        self.max_y = max_boundary[1] - safe_limit
        self.num_obstacles = rospy.get_param('~num_obstacles', 10)
        self.world = rospy.get_param('~world', 10)
        self.dt = rospy.get_param('~period_time', 0.1)

        print(self.num_obstacles)

        print("World boundaries: min_x: {}, max_x: {}, min_y: {}, max_y: {}".format(
            self.min_x, self.max_x, self.min_y, self.max_y))
        print("Number of dynamic obstacles: {}".format(self.num_obstacles))
        print("Human speed: {}".format(self.human_linear_velocity))

        # Publishers and Subscribers
        self.set_model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.marker_pub = rospy.Publisher("/dynamic_obstacles_markers", MarkerArray, queue_size=10)
        self.marker_lifetime = rospy.Duration(0.2)

        # Subscribe to model states to detect existing ones
        self.existing_model_names = []
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        # Wait until we have model states
        rospy.wait_for_message("/gazebo/model_states", ModelStates)

        self.existing_ids = self.extract_ids_from_models(self.existing_model_names)
        rospy.loginfo("Existing person_walking IDs: {}".format(self.existing_ids))

        self.objects = []
        self.initialize_objects(self.num_obstacles)

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update_timer_callback)

    def model_states_callback(self, msg):
        self.existing_model_names = msg.name

    def extract_ids_from_models(self, model_names):
        """Extract numerical IDs from model names like 'person_walking1', 'person_walking2'."""
        pattern = re.compile(r'person_walking(\d+)')
        existing_ids = set()
        for name in model_names:
            match = pattern.match(name)
            if match:
                try:
                    existing_ids.add(int(match.group(1)))
                except ValueError:
                    pass
        return existing_ids

    def initialize_objects(self, num_obstacles):
        direction_types = ["dx", "dy"]
        
        current_id = 1
        if num_obstacles == 0:
            return
            
        while len(self.objects) < num_obstacles:
            # if current_id not in self.existing_ids:
            start_x = random.uniform(self.min_x, self.max_x)
            start_y = random.uniform(self.min_y, self.max_y)
            object_type = random.choice(direction_types)
            name = "person_walking{}".format(current_id)

            new_obj = Object(name, self.human_linear_velocity, self.min_x, self.min_y,
                                self.max_x, self.max_y, object_type, start_x, start_y, current_id)
            self.objects.append(new_obj)
            # rospy.loginfo("Created {} at position ({}, {})".format(new_obj.model_name, start_x, start_y))

            current_id += 1

    def update_timer_callback(self, event):
        for obj in self.objects:
            obj.tick(self.dt)
            model_state = obj.get_model_state()
            self.set_model_state_pub.publish(model_state)
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, obj in enumerate(self.objects):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "dynamic_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obj.x
            marker.pose.position.y = obj.y
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = obj.qx
            marker.pose.orientation.y = obj.qy
            marker.pose.orientation.z = obj.qz
            marker.pose.orientation.w = obj.qw
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.7
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.lifetime = self.marker_lifetime
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main():
    try:
        AnimateObjects()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
