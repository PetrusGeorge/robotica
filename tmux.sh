#!/bin/bash

echo "========================================="
echo "ROS 2 - Start"
echo "========================================="

# Configurações
TMUX="tmux"

# # Executar limpeza
source ./cleanup.sh

# Build
echo "1. Build do workspace..."
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

# Source
echo "2. Configurando environment ROS..."
source "$WORKSPACE_DIR/install/local_setup.bash"

# Criar sessão Tmux
echo "3. Criando sessão Tmux '$SESSION_NAME'..."
echo ""

# Função auxiliar para criar janelas
create_tmux_window() {
    local idx=$1
    local name=$2
    local delay=$3
    local cmd=$4
    
    if [ $idx -eq 0 ]; then
        $TMUX new-session -d -s $SESSION_NAME -n "$name" \
            "$cmd; exec bash"
    else
        $TMUX new-window -t $SESSION_NAME:$idx -n "$name" \
            "echo 'Iniciando $name em $delay segundos...'; sleep $delay; $cmd; exec bash"
    fi
    echo "  [Janela $idx] $name ✔"
}

# Criar janelas com delays progressivos
create_tmux_window 0 "Robot_Desc" 0 "ros2 launch robotics_class robot_description.launch.py"
create_tmux_window 1 "Simulation" 2 "ros2 launch robotics_class simulation_world.launch.py"
create_tmux_window 2 "EKF" 15 "ros2 launch robotics_class ekf.launch.py"
create_tmux_window 3 "RViz" 2 "ros2 launch robotics_class rviz.launch.py"
create_tmux_window 4 "Teleop" 0 "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=jetauto/cmd_vel"
create_tmux_window 5 "Navigation" 15 "ros2 launch nav2_bringup navigation_launch.py"
# create_tmux_window 6 "SLAM" 18 "ros2 launch robotics_class slam.launch.py"
create_tmux_window 6 "Localization" 15 "ros2 launch robotics_class localization.launch.py"

# Janela de monitoramento/controle
$TMUX new-window -t $SESSION_NAME:7 -n 'Control'

# Anexar à sessão
$TMUX attach-session -t $SESSION_NAME
