#!/bin/bash

echo "========================================="
echo "ROS 2 - Start"
echo "========================================="

# Configurações
WORKSPACE_DIR="$HOME/ros2_ws"

# Executar limpeza
source ./cleanup.sh

# Build
echo "Build do workspace..."
cd "$WORKSPACE_DIR" || { 
    echo "ERRO: Pasta não encontrada em $WORKSPACE_DIR"
    exit 1
}

colcon build --symlink-install || { 
    echo "ERRO: Build falhou!"
    exit 1
}
echo "✓ Build OK"
echo ""

# Criar sessão Tmux
echo "Criando sessão Tmux '$SESSION_NAME'..."
echo ""

create_tmux_window() {
    local idx=$1
    local name=$2
    local delay=$3
    local cmd=$4
    
    # Full command with workspace sourcing
    local full_cmd="source $WORKSPACE_DIR/install/local_setup.bash && $cmd"
    
    if [ $idx -eq 0 ]; then
        $TMUX new-session -d -s $SESSION_NAME -n "$name" \
            "$full_cmd; exec bash"
    else
        $TMUX new-window -t $SESSION_NAME:$idx -n "$name" \
            "echo 'Iniciando $name em $delay segundos...'; sleep $delay; $full_cmd; exec bash"
    fi
    echo "  [Janela $idx] $name ✔"
}

# Criar janelas com delays progressivos
create_tmux_window 0 "Robot_Desc" 0 "ros2 launch robotics_subject robot_description.launch.py"
#create_tmux_window 1 "Teleop" 0 "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=jetauto/cmd_vel"
create_tmux_window 1 "Simulation" 3 "ros2 launch robotics_subject simulation_world.launch.py"
create_tmux_window 2 "EKF" 6 "ros2 launch robotics_subject ekf.launch.py"
create_tmux_window 3 "RViz" 15 "ros2 launch robotics_subject rviz.launch.py"
create_tmux_window 4 "SLAM" 18 "ros2 launch robotics_subject slam.launch.py"
create_tmux_window 5 "Nav2" 20  "ros2 launch robotics_subject nav2.launch.py"

create_tmux_window 6 "MazeSolver"    25  "ros2 run robotics_subject maze_solver_nav2_node use_sim_time:true"
create_tmux_window 7 "ColorDetector" 28  "ros2 run robotics_subject color_detector_node"

# Janela de monitoramento/controle
$TMUX new-window -t $SESSION_NAME:8 -n 'Control'

# Anexar à sessão
$TMUX attach-session -t $SESSION_NAME
