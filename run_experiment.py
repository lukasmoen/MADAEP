import subprocess
import time
import yaml
import shutil
import os
import signal
import math
import select
import ast
import random

# Command to open a new terminal tab and run a command
NEW_TAB_CMD = 'gnome-terminal --tab -- {}'

map_size_x =0
map_size_y =0
map_size_z =0
box_min_x  =0
box_min_y  =0
box_min_z  =0 
box_max_x  =0
box_max_y  =0
box_max_z  =0

# Builds a docker image from a image name
def build_image(image_name):
    docker_build_command = ["./dev_env.sh", "build", image_name]
    subprocess.run(docker_build_command, check=True)
    print(f"Docker image {image_name} has been built")


# Open a new terminal tab, run the docker image and run simulation
def simulation(image_name, world_value, mode_value, human_avoidance, drone_avoidance, drone_num, spawn_pos1, spawn_pos2,spawn_pos3,spawn_pos4,spawn_pos5, spawn_pos6, spawn_pos7, spawn_pos8, num_obstacles):
    # Corrected format: Added proper quoting for spawn positions
    DOCKER_RUN_CMD = "./dev_env.sh start {0} simulation.sh {1} {2} {3} {4} {5} \"{6}\" \"{7}\" \"{8}\" \"{9}\" \"{10}\" \"{11}\" \"{12}\" \"{13}\" {14}".format(
        image_name, world_value, mode_value, human_avoidance, drone_avoidance, drone_num, spawn_pos1, spawn_pos2, spawn_pos3, spawn_pos4, spawn_pos5, spawn_pos6, spawn_pos7, spawn_pos8, num_obstacles
    )
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD), shell=True)

# Open a new terminal tab, open a new instance of the docker image and run exploration
def exploration(image_name, config_file, drone_num):
    # Command to execute a command inside a running Docker container
    DOCKER_EXEC_CMD = "./dev_env.sh exec {0} exploration.sh {1} {2}".format(image_name, config_file, drone_num)
    # Open a new terminal tab and execute the exploration command inside the running Docker container
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD), shell=True)


# Open a new terminal tab, open a new instance of the docker image and run exploration
def exploration_dep(image_name, config_file, no_replan):
    # Command to execute a command inside a running Docker container
    DOCKER_EXEC_CMD_VOXBLOX = "./dev_env.sh exec {0} voxblox.sh".format(image_name)
    DOCKER_EXEC_CMD_PLANNER = "./dev_env.sh exec {0} exploration.sh {1} {2}".format(image_name, config_file, no_replan)
    # Open a new terminal tab and execute the exploration command inside the running Docker container
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_VOXBLOX), shell=True)
    time.sleep(3)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_PLANNER), shell=True)

# Open a new terminal tab, open a new instance of the docker image and run exploration
def exploration_racer(image_name, config_file, drone_num):
    # Command to execute a command inside a running Docker container
    DOCKER_EXEC_CMD = "./dev_env.sh exec {0} exploration.sh {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18} {19} {20} {21} {22} {23} {24} {25} {26} {27} {28} {29} {30} {31} {32} {33} {34} {35}".format(image_name, config_file, drone_num, map_size_x, map_size_y, map_size_z, box_min_x,
     box_min_y, box_min_z, box_max_x, box_max_y, box_max_z, 
     spawn_pos1_x, spawn_pos1_y, spawn_pos1_z+0.8,
     spawn_pos2_x, spawn_pos2_y, spawn_pos2_z+0.8,
     spawn_pos3_x, spawn_pos3_y, spawn_pos3_z+0.8,
     spawn_pos4_x, spawn_pos4_y, spawn_pos4_z+0.8,
     spawn_pos5_x, spawn_pos5_y, spawn_pos5_z+0.8,
     spawn_pos6_x, spawn_pos6_y, spawn_pos6_z+0.8,
     spawn_pos7_x, spawn_pos7_y, spawn_pos7_z+0.8,
     spawn_pos8_x, spawn_pos8_y, spawn_pos8_z+0.8)
    print("line: ", DOCKER_EXEC_CMD)
    # Open a new terminal tab and execute the exploration command inside the running Docker container
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD), shell=True)


# Open a new terminal tab,  open a new instance of the docker image and run save_octomap
def save_octomap(image_name, octomap_name):
     # Command to execute a command inside a running Docker container
     DOCKER_EXEC_CMD_OCTOMAP = "./dev_env.sh exec {0} save_octomap.sh {1}".format(image_name, octomap_name)
     # Open a new terminal tab and execute the save_octomap command inside the running Docker container
     subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_OCTOMAP), shell=True)


def kill_simulation(image_name):
    DOCKER_EXEC_CMD_SHUTDOWN = "./dev_env.sh exec {0} terminate.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_SHUTDOWN), shell=True)

def check_planning_complete(timeout, image_name, drone_num):
    command = ['./dev_env.sh', 'topic', image_name, 'exploration_completed.sh']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=False, preexec_fn=os.setsid)
    previous_minutes_remaining = None
    poll_obj = select.poll()
    poll_obj.register(process.stdout, select.POLLIN)
    
    drone_status = {f'aeplanner{i+1}': False for i in range(drone_num)}
    while True:
        minutes_remaining = math.floor((timeout - time.time()) / 60)
        if minutes_remaining != previous_minutes_remaining:
            print("Minutes remaining:", minutes_remaining)
            previous_minutes_remaining = minutes_remaining
        
        poll_result = poll_obj.poll(0)
        if poll_result:
            output = process.stdout.readline().decode("utf-8").strip()
            
            if "data:" in output:
                data = output.split("data: ")[1]  # Extract message content
                parts = data.split(",")  # Expecting format "aeplannerX, true/false"
                if len(parts) == 2:
                    drone_id, status = parts[0], parts[1].lower()[:-1] == "true"
                    drone_id = drone_id[2:]
                    if drone_id in drone_status:
                        if (not drone_status[drone_id]):
                            drone_status[drone_id] = status
                    
                    # Check if all drones have reported "true"
                    if all(drone_status.values()):
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                        return True  
        
        if time.time() > timeout:
            # If timeout is reached, terminate the process
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            return False
        
        # Wait for the subprocess to finish
        process.poll()
        if process.returncode is not None:
            return False


# Function to process each spawn_pos and add 1 to the third element (z)
def modify_spawn_pos(spawn_pos):
    # Remove parentheses, split by comma, and convert to a list of floats
    coords = spawn_pos.strip('()').split(',')
    coords = [float(coord) for coord in coords]
    
    # Add 1 to the third element (z)
    coords[2] += 0.8
    
    # Convert back to a tuple-like string format
    return f"({coords[0]},{coords[1]},{coords[2]})"

# Read the YAML file
with open('experiments.yaml', 'r') as file:
    try:
        data = yaml.safe_load(file)
    except yaml.YAMLError as e:
        print(e)


# Extract experiment parameters settings
print("Extracting data from experiments.yaml")
run_time = data['max_time']
N_simulations = data['simulation_runs']
planners = data['planners']
worlds = data['worlds']
modes = data['modes']
human_avoidance = data['human_avoidance']
drone_avoidance = data['drone_avoidance']
no_replan = data['no_replan']
drone_num = data['drone_num']
number_of_obstacles = data['number_of_obstacles']

print("\n--------- EXPERIMENT SETUP ---------")
print(f"Running every experiment for {run_time/60} Minutes ({run_time} Seconds)")
print(f"Running with the following planners: {' '.join(planners)}")
print(f"Running with the following worlds: {' '.join(worlds)}")
print(f"Running with the following modes: {' '.join(modes)}")
print(f"Moving objects tries to avoid the drone: {human_avoidance}")
print(f"Drone tries to avoid the moving objects: {drone_avoidance}")
print(f"Running simulation with {drone_num} drones")
print(f"Running simulation with {number_of_obstacles} number_of_obstacles")


print("------------------------------------")

# Run experiments
print("\n---------------- Starting experiments -----------------")
for num_obst in range(number_of_obstacles, number_of_obstacles+1, 10):
    for n in range(drone_num,drone_num+1,2):
        for planner in planners:
            print(f"--- Starting planning with {planner} ---")

            # Docker image name
            image_name = planners[planner]['image']

            # Build docker image
            build_image(image_name)
            

            for world in worlds:
                print(f"-Simulation world: {world}")
                # Config
                config_file = f"{world}_exploration.yaml"
                spawn_points = data["spawn_positions"][world]

                for mode in modes:
                    print(f"-Environment mode: {mode}")
                    for iter in range(N_simulations):
                        print(f"------- Iteration {iter} -------")

                        # Extract the spawn position for this iteration
                        # spawn_position = spawn_points[iter%5]
                        # print(f"Drone starts at position (x,y,z) : {spawn_position}")

                        indices = list(range(8))  # [0, 1, 2, 3, 4, 5, 6, 7]
                        random_indices = random.sample(indices, 8)  # Pick 4 random unique indices

                        spawn_pos1 = spawn_points[random_indices[0]]
                        spawn_pos2 = spawn_points[random_indices[1]]
                        spawn_pos3 = spawn_points[random_indices[2]]
                        spawn_pos4 = spawn_points[random_indices[3]]
                        spawn_pos5 = spawn_points[random_indices[4]]
                        spawn_pos6 = spawn_points[random_indices[5]]
                        spawn_pos7 = spawn_points[random_indices[6]]
                        spawn_pos8 = spawn_points[random_indices[7]]

                        print(spawn_pos5,spawn_pos6,spawn_pos7,spawn_pos8)

                        spawn_pos1_x = ast.literal_eval(spawn_pos1)[0]
                        spawn_pos1_y = ast.literal_eval(spawn_pos1)[1]
                        spawn_pos1_z = ast.literal_eval(spawn_pos1)[2]

                        spawn_pos2_x = ast.literal_eval(spawn_pos2)[0]
                        spawn_pos2_y = ast.literal_eval(spawn_pos2)[1]
                        spawn_pos2_z = ast.literal_eval(spawn_pos2)[2]

                        spawn_pos3_x = ast.literal_eval(spawn_pos3)[0]
                        spawn_pos3_y = ast.literal_eval(spawn_pos3)[1]
                        spawn_pos3_z = ast.literal_eval(spawn_pos3)[2]

                        spawn_pos4_x = ast.literal_eval(spawn_pos4)[0]
                        spawn_pos4_y = ast.literal_eval(spawn_pos4)[1]
                        spawn_pos4_z = ast.literal_eval(spawn_pos4)[2]

                        spawn_pos5_x = ast.literal_eval(spawn_pos5)[0]
                        spawn_pos5_y = ast.literal_eval(spawn_pos5)[1]
                        spawn_pos5_z = ast.literal_eval(spawn_pos5)[2]

                        spawn_pos6_x = ast.literal_eval(spawn_pos6)[0]
                        spawn_pos6_y = ast.literal_eval(spawn_pos6)[1]
                        spawn_pos6_z = ast.literal_eval(spawn_pos6)[2]

                        spawn_pos7_x = ast.literal_eval(spawn_pos7)[0]
                        spawn_pos7_y = ast.literal_eval(spawn_pos7)[1]
                        spawn_pos7_z = ast.literal_eval(spawn_pos7)[2]

                        spawn_pos8_x = ast.literal_eval(spawn_pos8)[0]
                        spawn_pos8_y = ast.literal_eval(spawn_pos8)[1]
                        spawn_pos8_z = ast.literal_eval(spawn_pos8)[2]
                        
                        print(f"Drone starts at position (x,y,z) : {spawn_points}")

                        # Start simulation
                        simulation(image_name, world, mode, human_avoidance, drone_avoidance, n, spawn_pos1, spawn_pos2, spawn_pos3, spawn_pos4, spawn_pos5, spawn_pos6, spawn_pos7, spawn_pos8, num_obst)
                        print("Waiting 40 seconds for gazebo simulation to open up")

                        # Wait for the simulation to start up
                        time.sleep(40)
                        
                        # Start Exploration
                        if planner == "dep":
                            exploration_dep(image_name, config_file, no_replan)
                        elif planner == "racer" or planner=="madaep" or planner=="daep":
                            if world == "cafe":
                                box_min_x= -5
                                box_min_y= -11
                                box_min_z= 0.0
                                box_max_x= 4
                                box_max_y= 12
                                box_max_z= 2.5
                                map_size_x= 30
                                map_size_y= 30
                                map_size_z= 3.5
                            if world == "apartment":
                                box_min_x= -13.5
                                box_min_y= -11.2
                                box_min_z= 0.0
                                box_max_x= 14.7
                                box_max_y= 16
                                box_max_z= 2.5
                                map_size_x= 40
                                map_size_y= 40
                                map_size_z= 3.5
                            if world == "apartment2":
                                box_min_x= -13.5
                                box_min_y= -11.2
                                box_min_z= 0.0
                                box_max_x= 14.7
                                box_max_y= 16
                                box_max_z= 6
                                map_size_x= 40
                                map_size_y= 40
                                map_size_z= 7
                            if world == "pillar":
                                box_min_x= -7
                                box_min_y= -14
                                box_min_z= 0.0
                                box_max_x= 7
                                box_max_y= 14
                                box_max_z= 2.5
                                map_size_x= 14
                                map_size_y= 28
                                map_size_z= 3.5
                            if world == "square":
                                box_min_x= -20
                                box_min_y= -20
                                box_min_z= 0.0
                                box_max_x= 20
                                box_max_y= 20
                                box_max_z= 2
                                map_size_x= 41
                                map_size_y= 41
                                map_size_z= 3
                            # if world == "maze":
                            #     box_min_x= -6.5
                            #     box_min_y= -16
                            #     box_min_z= 0.0
                            #     box_max_x= 13.5
                            #     box_max_y= 1.3
                            #     box_max_z= 2.5
                            #     map_size_x= 20
                            #     map_size_y= 17.3
                            #     map_size_z= 2.5
                            # if world == "patrol":
                            #     box_min_x= -10
                            #     box_min_y= -10
                            #     box_min_z= 0.0
                            #     box_max_x= 10
                            #     box_max_y= 10
                            #     box_max_z= 2
                            #     map_size_x= 20
                            #     map_size_y= 20
                            #     map_size_z= 2
                            # if world == "rays":
                            #     box_min_x= -7.5
                            #     box_min_y= -7.5
                            #     box_min_z= 0.0
                            #     box_max_x= 7.5
                            #     box_max_y= 7.5
                            #     box_max_z= 2.5
                            #     map_size_x= 15
                            #     map_size_y= 15
                            #     map_size_z= 2.5
                            # if world == "tunnel":
                            #     box_min_x= -5
                            #     box_min_y= 0
                            #     box_min_z= 0
                            #     box_max_x= 5
                            #     box_max_y= 22
                            #     box_max_z= 5
                            #     map_size_x= 10
                            #     map_size_y= 22
                            #     map_size_z= 5
                            # if world == "auditorium":
                            #     box_min_x= -2.5
                            #     box_min_y= -8.8
                            #     box_min_z= 0.0
                            #     box_max_x= 16.5
                            #     box_max_y= 5.9
                            #     box_max_z= 3
                            #     map_size_x= 19
                            #     map_size_y= 14.7
                            #     map_size_z= 3
                            # if world == "crosswalks":
                            #     box_min_x= -7.5
                            #     box_min_y= -7.5
                            #     box_min_z= 0.0
                            #     box_max_x= 7.5
                            #     box_max_y= 7.5
                            #     box_max_z= 2
                            #     map_size_x= 15
                            #     map_size_y= 15
                            #     map_size_z= 2
                            # if world == "exhibition":
                            #     box_min_x= -7.5
                            #     box_min_y= -7.5
                            #     box_min_z= 0.0
                            #     box_max_x= 7.5
                            #     box_max_y= 7.5
                            #     box_max_z= 2
                            #     map_size_x= 15
                            #     map_size_y= 15
                            #     map_size_z= 2
                            # if world == "field":
                            #     box_min_x= -12
                            #     box_min_y= -12
                            #     box_min_z= 0.0
                            #     box_max_x= 12
                            #     box_max_y= 12
                            #     box_max_z= 2.5
                            #     map_size_x= 24
                            #     map_size_y= 24
                            #     map_size_z= 2.5
                            if world == "granso":
                                box_min_x= -35
                                box_min_y= -30
                                box_min_z= 0.0
                                box_max_x= 40
                                box_max_y= 30
                                box_max_z= 5
                                map_size_x= 80
                                map_size_y= 60
                                map_size_z= 6
                            # if world == "granso2":
                            #     box_min_x= -52.5
                            #     box_min_y= -55
                            #     box_min_z= 0.5
                            #     box_max_x= 52.5
                            #     box_max_y= 54
                            #     box_max_z= 4
                            #     map_size_x= 110
                            #     map_size_y= 110
                            #     map_size_z= 5
                            # if world == "cave":
                            #     box_min_x= -20
                            #     box_min_y= -15
                            #     box_min_z= 0
                            #     box_max_x= 20
                            #     box_max_y= 15
                            #     box_max_z= 30
                            #     map_size_x= 40
                            #     map_size_y= 40
                            #     map_size_z= 30
                            if world == "house":
                                box_min_x= -17
                                box_min_y= -17
                                box_min_z= 0
                                box_max_x= 17
                                box_max_y= 17
                                box_max_z= 15
                                map_size_x= 34
                                map_size_y= 34
                                map_size_z= 16

                            exploration_racer(image_name, config_file, n)
                        else:

                            exploration(image_name, config_file, n)
                        
                        # Wait for exploration planner to start
                        print("Waiting 10 seconds for planner to start up")
                        time.sleep(10)

                        # Start timer
                        print(f"Running experiment for {run_time} seconds")
                        timeout = time.time() + run_time
                        completed = check_planning_complete(timeout, image_name, n)
                        if completed:
                            print("planner finished early")
                        else:
                            print("Time is up")

                        #time.sleep(run_time)
                        
                        # Save octomap
                        # octomap_name = f"{planner}_{world}_{mode}_{iter}.bt"
                        # save_octomap(image_name, octomap_name)
                        # print(f"Saving octomap inside {planner}/octomaps")
                        
                        # time.sleep(5)

                        # Kill the ROS nodes and shut down the simulation
                        kill_simulation(image_name)

                        print("Experiment Done!")

                        # # Save experiment data
                        # src_folder = f"./{image_name}/data/"
                        # dst_folder = f"./visualization/experiment_data/{planner}_{world}_{mode}_{iter}/"
                        # print(f"Saving saving data from {src_folder} to {dst_folder}")
                        
                        # # Create the destination directory if it doesn't exist
                        # if not os.path.exists(dst_folder):
                        #     os.makedirs(dst_folder)

                        # # Copy the files from the source directory to the destination directory
                        # # for file_name in os.listdir(src_folder):
                        # #     src_file = os.path.join(src_folder, file_name)
                        # #     dst_file = os.path.join(dst_folder, file_name)
                        # #     shutil.copytree(src_file, dst_file)
                        # # Delete any existing folders inside the destination folder
                        # for item in os.listdir(dst_folder):
                        #     item_path = os.path.join(dst_folder, item)
                        #     if os.path.isdir(item_path):
                        #         shutil.rmtree(item_path)  # Remove the folder and its contents

                        # for file_name in os.listdir(src_folder):
                        #     src_file = os.path.join(src_folder, file_name)
                        #     dst_file = os.path.join(dst_folder, file_name)

                        #     # If it's a folder, use copytree; if it's a file, use copy
                        #     if os.path.isdir(src_file):
                        #         # Only copy the folder if the destination doesn't exist
                        #         # if not os.path.exists(dst_file):
                        #         shutil.copytree(src_file, dst_file)
                        #         # else:
                        #         #     print(f"Folder {dst_file} already exists, skipping copy.")
                        #     else:
                        #         # If it's a file, just copy it
                        #         shutil.copy(src_file, dst_file)

                        # Define source and destination folders
                        src_folder = f"./{image_name}/data/"
                        dst_folder = f"./visualization/experiment_data/"

                        # Create the destination directory if it doesn't exist
                        if not os.path.exists(dst_folder):
                            os.makedirs(dst_folder)

                        # Only copy folders for the agents that were actually run (1 to n)
                        for i in range(0, n+1):
                            folder_name = f"aeplanner{i}"
                            src_path = os.path.join(src_folder, folder_name)
                            
                            # Check if this folder exists in the source directory
                            if os.path.isdir(src_path):
                                # Construct the new folder name
                                new_folder_name = f"{planner}{i}_{world}_{mode}_{n}_{iter}_{num_obst}"
                                dst_path = os.path.join(dst_folder, new_folder_name)
                                
                                # Copy the folder with the new name
                                if os.path.exists(dst_path):
                                    shutil.rmtree(dst_path)  # Remove the existing directory
                                shutil.copytree(src_path, dst_path)
                                print(f"Copied {src_path} to {dst_path}")


                        time.sleep(10)
                        print("waiting 10 seconds for gazebo simulation to close properly")
                
