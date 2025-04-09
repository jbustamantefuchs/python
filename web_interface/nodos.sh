#!/bin/bash

SCRIPT1="joystick_control.py"
SCRIPT2="nodered_control.py"
NODE1="direction_n_speed"
NODE2="nr_controller"

function stop_script {
    local script_name=$1
    local node_name=$2
    

    pid=$(pgrep -f "$script_name")
    
    if [ ! -z "$pid" ]; then
        echo "Killing process $script_name with PID $pid..."
        kill "$pid"
        sleep 1

        if pgrep -f "$script_name" > /dev/null; then
            echo "Forcing the Termination of the Process $script_name..."
            kill -9 "$pid"
        fi
    else
        echo "Process not found $script_name"
    fi


    if rosnode list | grep -q "$node_name"; then
        echo "Killing ROS node $node_name..."
        rosnode kill "$node_name"
    else
        echo "Node Not Found $node_name"
    fi
}

if [ "$1" == "joystick" ]; then
    stop_script "$SCRIPT2" "$NODE2"
    nohup python3 ~/path/to/$SCRIPT1 > /dev/null 2>&1 &
elif [ "$1" == "node-red" ]; then
    stop_script "$SCRIPT1" "$NODE1"
    nohup python3 ~/path/to/$SCRIPT2 > /dev/null 2>&1 &
elif [ "$1" == "stop" ]; then
    stop_script "$SCRIPT1" "$NODE1"
    stop_script "$SCRIPT2" "$NODE2"
else
    echo "Use: nodos.sh [script1 | script2 | stop_all]"
fi

