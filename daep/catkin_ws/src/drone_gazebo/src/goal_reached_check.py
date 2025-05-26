#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from rpl_exploration.msg import Goal  # Corrected to use the correct message type
from rpl_exploration.msg import Goals  # Corrected to use the correct message type
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import math

id = 0

# Thresholds for position and orientation difference (from move and rotate)
POSITION_DIFF = 1 #0.02  # Minimum distance between the drone and goal (m)
ANGLE_DIFF = 1     # Minimum difference in orientation (radians)

# Global variables to store the goal and the drone's state
goal_pose = Goal()  # Changed to Goal to match the message type
drone_pose = Pose()

# goal switch
new_goal = False
prev_goal = Goal()

# Callback for receiving the model state (drone's position and orientation)
def model_state_callback(msg):
    global drone_pose
    global id

    # Find the index for drone1 in the model states list
    try:
        drone_index = msg.name.index('drone'+str(id)) 
        drone_pose = msg.pose[drone_index]
    except ValueError:
        # rospy.logwarn("drone"+str(id)+" not found in model states.")
        return

# Callback for receiving the goal position
def goal_callback(msg):
    global goal_pose

    goal_pose = msg.goals[-1]  # Store the goal (rpl_exploration/Goal)

# Function to check if the drone has reached the goal
def check_goal_reached():
    global drone_pose, goal_pose, prev_goal, new_goal

    # Compute position distance
    position_diff = math.sqrt(
        (drone_pose.position.x - goal_pose.x) ** 2 +  # Changed to match Goal message type
        (drone_pose.position.y - goal_pose.y) ** 2 +
        (drone_pose.position.z - goal_pose.z) ** 2
    )

    # Compute orientation difference (using quaternion to angle conversion)
    drone_orientation = drone_pose.orientation
    goal_yaw = goal_pose.yaw

    # Convert quaternions to Euler angles (yaw, pitch, roll)
    drone_euler = euler_from_quaternion(drone_orientation)

    yaw_diff = abs(drone_euler[2] - goal_yaw)

    # Normalize yaw difference to be between 0 and pi
    yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

    # print("----------------------")
    # print("drone_pose: ", drone_pose.position.x,",",drone_pose.position.y,",",drone_pose.position.z)
    # print("goal_pose: ", goal_pose.x,",",goal_pose.y,",",goal_pose.z)
    # print("GOAL_DIFF: ", position_diff)
    
    # print("drone_yaw: ",drone_euler)
    # print("goal_yaw: ",goal_yaw)
    # print("YAW_DIFF: ", yaw_diff)
    # print("----------------------")


    # Check if both position and orientation are within threshold
    if position_diff <= POSITION_DIFF and (yaw_diff <= ANGLE_DIFF):
        if prev_goal != goal_pose and new_goal == False:
            prev_goal = goal_pose
            new_goal = True
            return True
    new_goal = False
    return False

# Convert quaternion to Euler angles (yaw, pitch, roll)
def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    # Roll, pitch, yaw (counterclockwise rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # Return roll, pitch, yaw

# Main function to initialize the ROS node and set up subscribers and publishers
def main():
    rospy.init_node('goal_reach_checker', anonymous=True)
    global id
    id = rospy.get_param("~id")

    # Subscribe to the model states and goal topics
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
    rospy.Subscriber('/drone'+str(id)+'/goal', Goals, goal_callback)  # Corrected to Goal

    # Publisher for the goal reached status
    goal_reached_pub = rospy.Publisher('drone'+str(id)+'/goal_reached', Bool, queue_size=10)

    # Loop to check the goal condition periodically
    rate = rospy.Rate(10)  # 10 Hz loop
    while not rospy.is_shutdown():
        goal_reached = Bool()
        
        # Check if the goal is reached
        goal_reached.data = check_goal_reached()

        # Publish the result
        goal_reached_pub.publish(goal_reached)

        # Sleep until the next loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
