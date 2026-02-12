#!/bin/bash
# Função para limpar processos ROS específicos

SESSION_NAME="ros2"
TMUX="tmux"

cleanup() {
    echo "Limpando processos ROS anteriores..."
    
    # Lista de comandos ROS que queremos matar
    local ros_commands=(
        "ros2 launch robotics_class"
        "ros2 launch nav2_bringup"
        "ros2 run teleop_twist_keyboard"
        "rviz2"
        "gzserver"
        "gzclient"
    )
    
    echo "Procurando processos ROS para terminar..."
    
    for cmd in "${ros_commands[@]}"; do
        # Encontrar PIDs dos processos específicos
        pids=$(ps aux | grep "$cmd" | grep -v grep | awk '{print $2}')
        
        if [ -n "$pids" ]; then
            echo "  Terminando: $cmd (PIDs: $pids)"
            # Matar gentilmente
            kill $pids 2>/dev/null
            sleep 0.5
            # Forçar se necessário
            kill -9 $pids 2>/dev/null 2>/dev/null
        fi
    done
    
    # Matar sessão Tmux
    echo "Verificando sessão Tmux antiga..."
    if $TMUX has-session -t $SESSION_NAME 2>/dev/null; then
        echo "  Matando sessão Tmux anterior: $SESSION_NAME"
        $TMUX kill-session -t $SESSION_NAME 2>/dev/null
    fi
    
    echo "✓ Limpeza concluída"
    echo ""
}

# Executar limpeza
cleanup
