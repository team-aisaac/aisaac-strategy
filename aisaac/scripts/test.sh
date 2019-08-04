#!/usr/bin/env bash

set -eu

sleep_time=7

echo '座標 [0.0, 2.0] に移動'
robot_id=0
status_msg='{ status: "move_linear", pid_goal_pos_x: 0.0, pid_goal_pos_y: 2.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: 0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

sleep ${sleep_time}

echo '座標 [0.0, -2.0] でパス受け'
robot_id=1
status_msg='{ status: "receive", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: -2.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

echo '座標 [0.0, -2.0] へパス'
robot_id=0
status_msg='{ status: "pass", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: -2.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

sleep ${sleep_time}

echo '座標 [-2.0, 0.0] へパス'
robot_id=0
status_msg='{ status: "pass", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: -2.0, pass_target_pos_y: 0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

echo '座標 [-2.0, 0.0] でパス受け'
robot_id=1
status_msg='{ status: "receive", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: -2.0, pass_target_pos_y: 0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

echo 'defence1'
robot_id=2
status_msg='{ status: "defence1", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: 0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

echo 'defence2'
robot_id=3
status_msg='{ status: "defence2", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: 0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

echo 'keeper'
robot_id=4
status_msg='{ status: "keeper", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: 0.0, pass_target_pos_y: 0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

sleep ${sleep_time}

echo 'シュート'
robot_id=5
status_msg='{ status: "pass", pid_goal_pos_x: 0.0, pid_goal_pos_y: 0.0, pid_goal_theta: 0.0, pid_circle_center_x: 0.0, pid_circle_center_y: 0.0, pass_target_pos_x: -6.0, pass_target_pos_y: -0.0 }'
cmd="rostopic pub -1 /blue/robot_${robot_id}/status aisaac/Status '"${status_msg}"'"
echo -e "command:\n${cmd}\n"
echo ${cmd} | bash &
echo

sleep ${sleep_time}
