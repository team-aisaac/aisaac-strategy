#!/usr/bin/env bash

set -eu

for i in $(seq 0 7); do
    echo "Stopping robot ${i}"
    robot_id=${i}
    status_msg='{ status: "stop", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: 0.0 }'
    cmd="rostopic pub -1 /yellow/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
    echo -e "command:\n${cmd}\n"
    echo ${cmd} | bash &
    echo
done

wait
