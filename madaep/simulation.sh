cd
source .bashrc
world=$1
mode=$2
human_avoidance=$3
drone_avoidance=$4
drone_num=$5
spawn_pos1=$6
spawn_pos2=$7
spawn_pos3=$8
spawn_pos4=$9
spawn_pos5=${10}
spawn_pos6=${11}
spawn_pos7=${12}
spawn_pos8=${13}
num_obstacles=${14}

roslaunch drone_gazebo simulation.launch \
    mode:=$mode \
    world:=$world \
    spawn_pos1:=$spawn_pos1 \
    spawn_pos2:=$spawn_pos2 \
    spawn_pos3:=$spawn_pos3 \
    spawn_pos4:=$spawn_pos4 \
    spawn_pos5:=$spawn_pos5 \
    spawn_pos6:=$spawn_pos6 \
    spawn_pos7:=$spawn_pos7 \
    spawn_pos8:=$spawn_pos8 \
    avoidance:=$human_avoidance \
    avoidance_mode:=$drone_avoidance \
    drone_num:=$drone_num \
    num_obstacles:=$num_obstacles
