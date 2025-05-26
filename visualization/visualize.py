import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate

# Variables
files = ["collision.csv", "coverage.csv", "logfile.csv", "path.csv"]
worlds = ["cafe","apartment", "apartment2","granso","house","pillar","square"]
modes = ["static", "dynamic"]
planners = ["daep0", "racer0","madaep0"]
planners_individual = ["daep0", "racer0","madaep0","daep1","daep2","daep3","daep4", "racer1","racer2","racer3","racer4","madaep1","madaep2","madaep3","madaep4"]

N = 1 #Should be the same as simulation_runs in experiments.yaml (Parametrize)
num_agents = 2 #Should be the same as simulation_runs in experiments.yaml (Parametrize)
num_obstacles = 0 #Should be the same as simulation_runs in experiments.yaml (Parametrize)
#################################### LABELS ###########################################################

def get_legend_name(df_key):
    if "madaep0" in df_key:
        return "MA-DAEP TOTAL"
    elif "madaep" in df_key:
        return "MA-DAEP"
    elif "daep0" in df_key:
        return "DAEP TOTAL"
    elif "daep" in df_key:
        return "DAEP"
    elif "racer0" in df_key:
        return "RACER TOTAL"
    # elif "racer" in df_key:
    #     return "RACER"
    else:
        return "unknown"


#################################### COVERAGE ###########################################################

def plot_coverage_average(world, data_frames, max_time):
    N_interpolation_points = int(np.ceil(max_time*2))
    
    plt.figure()
    for key in data_frames.keys():
        # get all the simulation for certain world and planner
        dfs = data_frames[key]
        N_df = len(dfs)
        Y = np.zeros((N_df, N_interpolation_points))
        max_t = 0
        for i in range(len(dfs)):
            if dfs[i]['Time'].iloc[-1] > max_t:
                max_t = dfs[i]['Time'].iloc[-1]
        if "madaep0" in key:
            color = "green"
        elif "madaep" in key:
            color = "lime"
        elif "daep0" in key:
            color = "royalblue"
        elif "daep" in key:
            color = "cyan"
        elif "racer0" in key:
            color = "firebrick"
        # elif "racer" in key:
        #     color = "red"
        else:
            color = "green" 
        for j, df in enumerate(dfs):
            y = df[' Coverage (m3)']
            
            x = np.linspace(0, max_t, N_interpolation_points)
            interpolate_df = np.interp(x, np.linspace(0, max_t, len(y)), y)
            Y[j, :] = interpolate_df

        mean_Y = np.mean(Y, axis=0)
        std_Y = np.std(Y, axis=0)

        plt.plot(x, mean_Y, color=color, label=get_legend_name(key), linewidth=2)
        plt.plot(x, mean_Y + std_Y, color=color, linestyle='--', linewidth=2)
        plt.plot(x, mean_Y - std_Y, color=color, linestyle='--', linewidth=2)
        # Fill the area between the lines
        plt.fill_between(x, mean_Y - std_Y, mean_Y + std_Y, color=color, alpha=0.2)

        plt.xlabel('Time [s]')
        plt.ylabel('Coverage [m3]')
        plt.title(f'World: {world} - Mean and Standard deviation')
        plt.legend()

    # create the subdirectory if it does not exist
    if not os.path.exists(volume_avg_path):
        os.makedirs(volume_avg_path)
        
    # save the plot in the subdirectory
    plt.savefig(volume_avg_path + f"/{world}")



def visualize_coverage_avg(show,n,num_obst):
    #for file in files:
    file = "coverage.csv"
    for world in worlds:
        dfs = {}
        longest_exploration_time = None
        for mode in ["dynamic"]:
            for planner in planners:
                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                    if os.path.isfile(file_path): 
                        #Check if file exists
                        df = pd.read_csv(file_path)

                        # print(world)
                        # print(mode)
                        # print(planner)
                        # print(iteration)

                        if longest_exploration_time is None:
                            longest_exploration_time = df['Time'].iloc[-1]

                        elif df['Time'].iloc[-1] > longest_exploration_time:
                            longest_exploration_time = df['Time'].iloc[-1]
                        
                        key = f"{mode}_{planner}"
                        if key in dfs:
                            dfs[key].append(df)
                        else:
                            dfs[key] = [df]
        
        if longest_exploration_time is not None:
            
            plot_coverage_average(world, dfs, longest_exploration_time)
    if show:
        plt.show()



# Save plot of coverage (%) for the best simulation of every planner and mode for a certain world
def plot_coverage_percent(world, data_frames, interval_dfs):
    plt.figure()
    ax = plt.subplot(111)
    paintOnceHuman = True
    paintOnceAgent = True
    colors = ["blue", "red", "green", "purple"]

    for i, df_key in enumerate(data_frames):
        if "madaep0" in df_key:
            color = "green"
        elif "madaep" in df_key:
            color = "lime"
        elif "daep0" in df_key:
            color = "royalblue"
        elif "daep" in df_key:
            color = "cyan"
        elif "racer0" in df_key:
            color = "firebrick"
        else:
            color = "green"       

        if "static" in df_key:
            linestyle = '-'
        else:
            linestyle = '--'

        # Coverage data
        cov_key = data_frames[df_key].columns[2]
        coverage_col = data_frames[df_key][cov_key]
        time_col = data_frames[df_key]['Time']

        # Plot coverage line
        plt.plot(time_col, coverage_col, label=get_legend_name(df_key), linestyle=linestyle, color=color)

        # Collision data
        start_col = interval_dfs[df_key]["start-time"]
        end_col = interval_dfs[df_key]["end-time"]
        collision_types = interval_dfs[df_key]["type-of-collision"]
        # Show crosses at collision start and end
        crosses_start = np.interp(start_col, time_col, coverage_col)
        crosses_end = np.interp(end_col, time_col, coverage_col)
        plt.plot(start_col, crosses_start, 'x', color="red")        
        plt.plot(end_col, crosses_end, 'x', color="green") 

        # Plot collision segments with different colors based on type
        for j in range(len(start_col)):
            duration = np.linspace(start_col[j], end_col[j], 500)
            duration_interpol = np.interp(duration, time_col, coverage_col)
            
            if collision_types[j] == "Agent-Human":
                if paintOnceHuman:
                    plt.plot(duration, duration_interpol, color="orange", linewidth=5, label="Human-Collision")
                    paintOnceHuman = False
                else:
                    plt.plot(duration, duration_interpol, color="orange", linewidth=5)
            elif collision_types[j] == "Agent-Agent":
                if paintOnceAgent:
                    plt.plot(duration, duration_interpol, color="magenta", linewidth=5, label="Agent-Collision")
                    paintOnceAgent = False
                else:
                    plt.plot(duration, duration_interpol, color="magenta", linewidth=5)

    ax.axhline(y=100, color='purple', linestyle="--")
    plt.title(f"World: {world} - Coverage [%]")
    plt.xlabel('Time [s]')
    plt.ylabel('Coverage [%]')
    plt.ylim(0, 110)
    plt.legend()
    
    # create the subdirectory if it does not exist
    if not os.path.exists(volume_p_path):
        os.makedirs(volume_p_path)
        
    # save the plot in the subdirectory
    plt.savefig(volume_p_path + f"/{world}")

# Save plot of coverage (%) for the best simulation of every planner and mode for a certain world
def plot_volumes(world, planner, data_frames):
    line_styles = ['-', '--', '-.', ':']
    colors = ['blue', 'red']
    #plt.figure()
    fig, ax = plt.subplots(figsize=(10,6)) # larger figure size

    for i, df_key in enumerate(data_frames):
        cov_m3_key = data_frames[df_key].columns[1]
        cov_free_key = data_frames[df_key].columns[3]
        cov_occupied_key = data_frames[df_key].columns[4]
        cov_unmapped_key = data_frames[df_key].columns[5]

        cov_m3_col = data_frames[df_key][cov_m3_key]
        cov_free_col = data_frames[df_key][cov_free_key]
        cov_occupied_col = data_frames[df_key][cov_occupied_key]
        cov_unmapped_col = data_frames[df_key][cov_unmapped_key]

        time_col = data_frames[df_key]['Time']

        plt.plot(time_col, cov_m3_col, label="Mapped", linestyle=line_styles[0], color=colors[i])
        plt.plot(time_col, cov_free_col, label="Free", linestyle=line_styles[1], color=colors[i])
        plt.plot(time_col, cov_occupied_col, label="Occupied", linestyle=line_styles[2], color=colors[i])
        plt.plot(time_col, cov_unmapped_col, label="Unmapped", linestyle=line_styles[3], color=colors[i])

    plt.title(f"World: {world} - Coverage [m3]")
    plt.xlabel('Time [s]')
    plt.ylabel('Coverage [m3]')

    # Shrink current axis by 20%
    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.7, box.height])

    # Put a legend to the right of the current axis
    ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    #plt.legend()
    # create the subdirectory if it does not exist
    if not os.path.exists(volume_v_path):
        os.makedirs(volume_v_path)
        
    # save the plot in the subdirectory
    plt.savefig(volume_v_path + f"/{world}_{planner}")


def visualize_coverage_p(show, n, num_obst):
    file = "coverage.csv"
    for world in worlds:
        best_dfs = {}
        interval_dfs = {}
        for mode in modes:
            for planner in planners:
                best_coverage = 0
                best_df = None
                best_simulation = 0
                
                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                    if os.path.isfile(file_path): 
                        # Check if file exists
                        df = pd.read_csv(file_path)
                        
                        if best_df is None:
                            best_df = df
                            best_coverage = df[' Coverage (%)'].iloc[-1]
                            best_simulation = iteration
                        elif df[' Coverage (%)'].iloc[-1] > best_coverage:
                            best_coverage = df[' Coverage (%)'].iloc[-1]
                            best_df = df
                            best_simulation = iteration
                
                if best_df is not None:
                    key = f"{mode}_{planner}"
                    best_dfs[key] = best_df
                    
                    # Create a new combined intervals dataframe
                    combined_intervals = pd.DataFrame(columns=["start-time", "duration", "end-time", "type-of-collision"])
                    
                    # Loop through all possible agents (0 to n-1)
                    for agent_id in range(1,N+1):
                        # Use the corrected file path format
                        agent_interval_path = f"/home/visualization/experiment_data/{planner[0:-1]}{agent_id}_{world}_{mode}_{n}_{best_simulation}_{num_obst}/intervals.csv"
                        print(agent_interval_path)
                        # If this agent's interval file exists
                        if os.path.isfile(agent_interval_path):
                            agent_intervals = pd.read_csv(agent_interval_path)
                            # Append this agent's intervals to the combined dataframe
                            combined_intervals = pd.concat([combined_intervals, agent_intervals], ignore_index=True)
                    
                    # Sort the combined intervals by start time
                    if not combined_intervals.empty:
                        combined_intervals = combined_intervals.sort_values(by="start-time").reset_index(drop=True)
                    interval_dfs[key] = combined_intervals
        
        if len(best_dfs) != 0:
            plot_coverage_percent(world, best_dfs, interval_dfs)
            
    if show:
        plt.show()


def visualize_coverage_v(show,n,num_obst):
    file = "coverage.csv"
    for world in worlds:
        for planner in planners:
            best_dfs = {}
            for mode in modes:
                best_coverage = 0
                best_df = None
                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                    if os.path.isfile(file_path): 
                        #Check if file exists
                        df = pd.read_csv(file_path)
                        if best_df is None:
                            best_df = df
                            best_coverage = df[' Coverage (%)'].iloc[-1]
                        
                        elif df[' Coverage (%)'].iloc[-1] > best_coverage:
                                best_coverage = df[' Coverage (%)'].iloc[-1]
                                best_df = df

                if best_df is not None:
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{num_obst}/"
                    key = f"{mode}_{planner}"
                    best_dfs[key] = best_df
            
            if len(best_dfs) != 0:
                plot_volumes(world, planner, best_dfs) 
    if show:
        plt.show()


#################################### LOGFILE ###########################################################


def visualize_logfile(n,num_obst):
    # Create data frame and corresponding headers
    logfile_headers = ["World", "Planner", "Mode", "Total Time (mean)", "Total Time (std)", "Path Length (mean)", "Path Length (std)", "Total Planning time (mean)", "Total Planning time (std)", 
                     "Total Flying time (mean)", "Total Flying time (std)"]
    logfile_df = pd.DataFrame(columns=logfile_headers)
    file = files[2]

    for world in worlds:
        for mode in modes:
            for planner in planners:
                
                total_times = []
                total_planning_times = []
                total_flying_times = []
                total_path_lengths = []
                iteration_exists = False

                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                    if os.path.isfile(file_path): #Check if file exists

                        iteration_exists = True
                        df = pd.read_csv(file_path)



                        if planner != "depr" and planner != "dep":
                            #Add the total planning and flyging time for each iteration
                            total_times.append(df[" Time"].iloc[-1])
                            total_path_lengths.append(df[" Path length"].iloc[-1])
                            total_planning_times.append(df[" Planning"].iloc[-1])
                            total_flying_times.append(df[" Flying"].iloc[-1])
                        else:
                            #If we use the DEP planner, Flying is unavailable.
                            total_times.append(df[" Time"].iloc[-1])
                            total_path_lengths.append(df[" Path length"].iloc[-1])
                            total_planning_times.append(df[" Planning"].iloc[-1])
                            total_flying_times.append(0)
                    
                # If a certain experiment does not exist, overwrite all metrics with zeros
                if not iteration_exists:
                    total_times.append(0)
                    total_path_lengths.append(0)
                    total_planning_times.append(0)
                    total_flying_times.append(0)
                    
                # Extract metrics after all iterations
                total_time_mean = np.round(np.mean(total_times), 2)
                total_time_std = np.round(np.std(total_times), 2)
                total_path_lengths_mean = np.round(np.mean(total_path_lengths), 2)
                total_path_lengths_std = np.round(np.std(total_path_lengths), 2)
                total_planning_times_mean = np.round(np.mean(total_planning_times), 2)
                total_planning_times_std = np.round(np.std(total_planning_times), 2)
                total_flying_times_mean = np.round(np.mean(total_flying_times), 2)
                total_flying_times_std = np.round(np.std(total_flying_times), 2)


                # Create a row and insert at the end of the data frame
                list_row = [world, planner, mode, total_time_mean, total_time_std, total_path_lengths_mean, total_path_lengths_std, total_planning_times_mean, 
                            total_planning_times_std, total_flying_times_mean, total_flying_times_std]
                logfile_df.loc[len(logfile_df)] = list_row
                    

    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #collision_df = collision_df.sort_values(by=[logfile_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(logfile_df, headers=logfile_headers, tablefmt="fancy_grid")

    # Print table if needed, otherwise look in logfile.txt
    #print(table)
                
    with open(logfile_path, 'w') as f:
        f.write(table)


def visualize_coverage(n,num_obst):
    # Create data frame and corresponding headers
    headers = ["World", "Planner", "Mode", "Coverage (%) mean", "Coverage (%) std"]
    coverage_df = pd.DataFrame(columns=headers)
    file = "coverage.csv"

    for world in worlds:
        for mode in modes:
            for planner in planners:
                
                total_coverages = []
                iteration_exists = False

                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                    if os.path.isfile(file_path): #Check if file exists

                        iteration_exists = True
                        df = pd.read_csv(file_path)
                        total_coverages.append(df[" Coverage (%)"].iloc[-1])
                    
                # If a certain experiment does not exist, overwrite all metrics with zeros
                if not iteration_exists:
                    total_coverages.append(0)
                    
                # Extract metrics after all iterations
                total_coverages_mean = np.round(np.mean(total_coverages), 2)
                total_coverages_std = np.round(np.std(total_coverages), 2)

                # Create a row and insert at the end of the data frame
                list_row = [world, planner, mode, total_coverages_mean, total_coverages_std]
                coverage_df.loc[len(coverage_df)] = list_row
                    

    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #coverage_df = coverage_df.sort_values(by=[headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(coverage_df, headers=headers, tablefmt="fancy_grid")

    # Print table if needed, otherwise look in logfile.txt
    #print(table)
                
    with open(coverage_path, 'w') as f:
        f.write(table)


#################################### COLLISION ###########################################################

def visualize_collision(n,num_obst):

    # Create data frame and corresponding headers
    collision_headers = ["World", "Planner", "Mode", \
                        "Number of Collision human (mean)", \
                        "Number of Collision agent (mean)", \
                        "Number of Collision human (std)", \
                        "Number of Collision agent (std)", \
                        "Total collision duration human (mean)", \
                        "Total collision duration agent (mean)", \
                        "Total collision duration human (std)", \
                        "Total collision duration agent (std)", \
                        "Average Collision Time human", \
                        "Average Collision Time agent"]
    collision_df = pd.DataFrame(columns=collision_headers)
    file = files[0]
    for world in worlds:
        for mode in modes:
            for planner in planners_individual:
                collisions_human = []
                collisions_agent = []
                total_duration_times_human = []
                total_duration_times_agent = []
                iteration_exists = False

                for iteration in range(N):
                    file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                    if os.path.isfile(file_path): #Check if file exists
                        iteration_exists = True

                        # print(world)
                        # print(mode)
                        # print(planner)
                        # print(iteration)

                        df = pd.read_csv(file_path)

                        for idx, row in df.iterrows():
                            if row['type_of_collision'] == "Agent-Human":
                                collisions_human.append(row['nr_of_collisions'])
                                total_duration_times_human.append(row['Total duration'])
                            elif row['type_of_collision'] == "Agent-Agent":
                                collisions_agent.append(row['nr_of_collisions'])
                                total_duration_times_agent.append(row['Total duration'])
                    
                # If a certain experiment does not exist, overwrite all metrics with zeros
                if not iteration_exists:
                    collisions_human.append(0)
                    total_duration_times_human.append(0)    
                    collisions_agent.append(0)
                    total_duration_times_agent.append(0)
                    
                collision_mean_human = np.mean(collisions_human)
                collisions_std_human = np.std(collisions_human)
                total_duration_mean_human = np.mean(total_duration_times_human)
                total_duration_std_human = np.std(total_duration_times_human)

                collision_mean_agent = np.mean(collisions_agent)
                collisions_std_agent = np.std(collisions_agent)
                total_duration_mean_agent = np.mean(total_duration_times_agent)
                total_duration_std_agent = np.std(total_duration_times_agent)
                
                if total_duration_mean_human != 0:
                    average_collision_time_human = collision_mean_human/total_duration_mean_human
                else:
                    average_collision_time_human = 0

                if total_duration_mean_agent != 0:
                    average_collision_time_agent = collision_mean_agent/total_duration_mean_agent
                else:
                    average_collision_time_agent = 0

                # Create a row and insert at the end of the data frame
                list_row = [world, planner, mode, \
                            collision_mean_human, \
                            collision_mean_agent, \
                            collisions_std_human, \
                            collisions_std_agent, \
                            total_duration_mean_human, \
                            total_duration_mean_agent, \
                            total_duration_std_human, \
                            total_duration_std_agent, \
                            average_collision_time_human, \
                            average_collision_time_agent]

                collision_df.loc[len(collision_df)] = list_row
                    

    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #collision_df = collision_df.sort_values(by=[collision_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(collision_df, headers=collision_headers, tablefmt="fancy_grid")

    # Print table if needed, otherwise look in table.txt
    #print(table)
                
    with open(collision_path, 'w') as f:
        f.write(table)


#################################### Aggregated results ###########################################################

def visualize_agg_results(n,num_obst):
    agg_headers = ["Planner", "Coverage mean", "Coverage std", \
                    "Time mean", "Time std", "Path length mean", \
                    "Path length std","Planning time mean",\
                    "Planning time std", "Collision mean human", "Collision std human", \
                    "Collision duration mean human", "Collision duration std human", \
                    "Collision mean agent", "Collision std agent", \
                    "Collision duration mean agent", "Collision duration std agent", ]
    agg_df = pd.DataFrame(columns=agg_headers)
    for planner in planners:
        collisions_human = []
        collisions_agent = []
        total_duration_times_human = []
        total_duration_times_agent = []
        total_coverages = []
        total_times = []
        total_planning_times = []
        total_path_lengths = []
        
        for world in worlds:
            for mode in ["dynamic"]:
                for iteration in range(N):
                    # Coverage data
                    coverage_file = files[1]
                    coverage_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{coverage_file}"
                    if os.path.isfile(coverage_path): #Check if file exists  
                        df = pd.read_csv(coverage_path)
                        total_coverages.append(df[" Coverage (%)"].iloc[-1])
                        
                    # Logfile data
                    log_file = files[2]
                    log_file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{log_file}"
                    if os.path.isfile(log_file_path): #Check if file exists 
                        df = pd.read_csv(log_file_path)         
                        #Add the total planning and flyging time for each iteration
                        total_times.append(df[" Time"].iloc[-1])
                        total_path_lengths.append(df[" Path length"].iloc[-1])
                        total_planning_times.append(df[" Planning"].iloc[-1])

                    # Collision data
                    collision = [0,0]
                    duration = [0,0]
                    for p in planners_individual:
                        if p[0:-1] == planner[0:-1]:
                            collision_file = files[0]
                            collision_path = f"/home/visualization/experiment_data/{p}_{world}_{mode}_{n}_{iteration}_{num_obst}/{collision_file}"
                    
                            if os.path.isfile(collision_path): #Check if file exists     
                                df = pd.read_csv(collision_path)
                                for idx, row in df.iterrows():
                                    if row['type_of_collision'] == "Agent-Human":
                                        collision[0] += row['nr_of_collisions']
                                        duration[0] += row['Total duration']
                                    elif row['type_of_collision'] == "Agent-Agent":
                                        collision[1] += row['nr_of_collisions']
                                        duration[1] += row['Total duration']
                    # if collision:
                    collisions_human.append(collision[0])
                    collisions_agent.append(collision[1])
                    total_duration_times_human.append(duration[0])
                    total_duration_times_agent.append(duration[1])
                    # print(planner,world,mode,iteration, collision)


        if total_coverages:
            total_coverages_mean = np.round(np.mean(total_coverages), 2)
            total_coverages_std = np.round(np.std(total_coverages), 2)
        else:
            total_coverages_mean = 0
            total_coverages_std = 0

        ## human collisions and data
        if collisions_human:
            collisions_mean_human = np.mean(collisions_human)
            collisions_std_human = np.std(collisions_human)
        else:
            collisions_mean_human = 0
            collisions_std_human = 0

        if total_duration_times_human:
            total_duration_mean_human = np.mean(total_duration_times_human)
            total_duration_std_human = np.std(total_duration_times_human)
        else:
            total_duration_mean_human = 0
            total_duration_std_human = 0
 

        ## agent collisions and data
        if collisions_agent:
            collisions_mean_agent = np.mean(collisions_agent)
            collisions_std_agent = np.std(collisions_agent)
        else:
            collisions_mean_agent = 0
            collisions_std_agent = 0

        if total_duration_times_agent:
            total_duration_mean_agent = np.mean(total_duration_times_agent)
            total_duration_std_agent = np.std(total_duration_times_agent)
        else:
            total_duration_mean_agent = 0
            total_duration_std_agent = 0



        if total_times:
            total_time_mean = np.round(np.mean(total_times), 2)
            total_time_std = np.round(np.std(total_times), 2)
        else:
            total_time_mean = 0
            total_time_std = 0

        if total_path_lengths:
            total_path_lengths_mean = np.round(np.mean(total_path_lengths), 2)
            total_path_lengths_std = np.round(np.std(total_path_lengths), 2)
        else:
            total_path_lengths_mean = 0
            total_path_lengths_std = 0

        if total_planning_times:
            total_planning_times_mean = np.round(np.mean(total_planning_times), 2)
            total_planning_times_std = np.round(np.std(total_planning_times), 2)
        else:
            total_planning_times_mean = 0
            total_planning_times_std = 0

        
        # Create a row and insert at the end of the data frame
        list_row = (
        [
            planner,
            total_coverages_mean, total_coverages_std,
            total_time_mean, total_time_std,
            total_path_lengths_mean, total_path_lengths_std,
            total_planning_times_mean, total_planning_times_std,
            collisions_mean_human, collisions_std_human,
            total_duration_mean_human, total_duration_std_human,
            collisions_mean_agent, collisions_std_agent,
            total_duration_mean_agent, total_duration_std_agent,
        ]
        )
        agg_df.loc[len(agg_df)] = list_row
        
        table = tabulate(agg_df, headers=agg_headers, tablefmt="fancy_grid")
        # print(table)
        with open(agg_path, 'w') as f:
            f.write(table)

def plot_coverage_by_agents(world, mode, planner, agent_data_frames, max_time):
    """
    Plot coverage over time for different numbers of agents using the same planner.
    
    Args:
        world (str): The world being used
        mode (str): Static or dynamic mode
        planner (str): The planner being used
        agent_data_frames (dict): Dictionary with keys as agent counts and values as lists of dataframes
        max_time (float): Maximum time to plot
    """
    N_interpolation_points = int(np.ceil(max_time*2))
    
    plt.figure(figsize=(12, 8))
    colors = plt.cm.viridis(np.linspace(0, 1, len(agent_data_frames)))
    
    for i, agent_count in enumerate(sorted(agent_data_frames.keys())):
        # get all the simulations for certain world, planner and agent count
        dfs = agent_data_frames[agent_count]
        N_df = len(dfs)
        Y = np.zeros((N_df, N_interpolation_points))
        max_t = 0
        for j in range(len(dfs)):
            if dfs[j]['Time'].iloc[-1] > max_t:
                max_t = dfs[j]['Time'].iloc[-1]
        
        for j, df in enumerate(dfs):
            y = df[' Coverage (m3)']
            
            x = np.linspace(0, max_t, N_interpolation_points)
            interpolate_df = np.interp(x, np.linspace(0, max_t, len(y)), y)
            Y[j, :] = interpolate_df

        mean_Y = np.mean(Y, axis=0)
        std_Y = np.std(Y, axis=0)

        plt.plot(x, mean_Y, color=colors[i], label=f"{agent_count} Agents", linewidth=2)
        plt.plot(x, mean_Y + std_Y, color=colors[i], linestyle='--', linewidth=1, alpha=0.7)
        plt.plot(x, mean_Y - std_Y, color=colors[i], linestyle='--', linewidth=1, alpha=0.7)
        # Fill the area between the lines
        plt.fill_between(x, mean_Y - std_Y, mean_Y + std_Y, color=colors[i], alpha=0.2)

    plt.xlabel('Time [s]')
    plt.ylabel('Coverage [m3]')
    plt.title(f'World: {world}, Planner: {get_legend_name(planner)} - Agent Count Comparison')
    plt.legend()
    
    # create the subdirectory if it does not exist
    agents_path = data_path + "plots/agent_comparison"
    if not os.path.exists(agents_path):
        os.makedirs(agents_path)
        
    # save the plot in the subdirectory
    plt.savefig(agents_path + f"/{world}_{planner}_{mode}")


def visualize_coverage_by_agents(show=False):
    """
    Creates plots comparing coverage performance with different numbers of agents for each planner.
    """
    file = "coverage.csv"
    
    for world in worlds:
        for mode in ["dynamic"]:  # Using only dynamic mode
            for planner in planners:
                agent_dfs = {}  # Dictionary to store dataframes by agent count
                longest_exploration_time = None
                
                # Collect data for all agent counts
                for n in range(num_agents, num_agents+1):
                    agent_dfs[n] = []
                    
                    for iteration in range(N):
                        file_path = f"/home/visualization/experiment_data/{planner}_{world}_{mode}_{n}_{iteration}_{num_obst}/{file}"
                        if os.path.isfile(file_path):
                            df = pd.read_csv(file_path)
                            
                            if longest_exploration_time is None:
                                longest_exploration_time = df['Time'].iloc[-1]
                            elif df['Time'].iloc[-1] > longest_exploration_time:
                                longest_exploration_time = df['Time'].iloc[-1]
                            
                            agent_dfs[n].append(df)
                
                # Filter out empty agent counts
                agent_dfs = {k: v for k, v in agent_dfs.items() if v}
                
                # Plot if we have data
                if agent_dfs and longest_exploration_time is not None:
                    plot_coverage_by_agents(world, mode, planner, agent_dfs, longest_exploration_time)
    
    if show:
        plt.show()

if __name__ == '__main__':
    for num_obst in range(num_obstacles,num_obstacles+1,10):
        for n in range(num_agents,num_agents+1):
            data_path = "visualized_data/"+str(n)+"/" +str(num_obst) + "/"
            if not os.path.exists(data_path):
                os.makedirs(data_path)
            logfile_path = data_path + "logfile.txt"
            collision_path = data_path + "collision.txt"
            coverage_path = data_path + "coverage.txt"
            agg_path = data_path + "aggregated.txt"

            print("Using save location: "+data_path)

            # Store volume (%) plots to the following path
            volume_p_path = data_path + "plots/coverage_p"

            # Store volume (m3) plots to the following path
            volume_v_path = data_path + "plots/coverage_v"

            # Store average volume (m3) plots to the following path
            volume_avg_path = data_path + "plots/coverage_avg"
            visualize_coverage_avg(False,n,num_obst)
            visualize_coverage_p(False,n,num_obst)
            visualize_coverage_v(False,n,num_obst)
            visualize_collision(n,num_obst)
            visualize_logfile(n,num_obst)
            visualize_coverage(n,num_obst)
            visualize_agg_results(n,num_obst)
            visualize_coverage_by_agents(False)

