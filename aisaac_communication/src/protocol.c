#include "aisaac_communication/protocol.h"
#include <stdint.h>
#include <arpa/inet.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#define RX_ASSERT(x) (assert(send.x == recv.x));

const uint8_t protocol_version = 0b00100010;    // Ver. 2.2
const uint8_t strategy_pc_command_data_type = 0b10100001;   // 5-1
const uint8_t dwa_result_data_type = 0b10100010;   // 5-2
const uint8_t vision_data_data_type = 0b10100011;   // 5-3
const uint8_t manual_controller_data_type = 0b10100100; // 5-4
const uint8_t robot_odometry_data_type = 0b10100101; // 5-5
const uint8_t robot_observed_ball_data_type = 0b10100110;    // 5-6

int encodeStrategyPcCommand(_strategy_pc_command *command, char *buffer) {
    uint8_t buffer_index = 0;
    uint16_t tmp_u16;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = strategy_pc_command_data_type;
    buffer_index += 1;

    buffer[buffer_index] =
        (char)command->halt_flag << 7
        | (char)command->stop_game_flag << 6
        | (char)command->ball_placement_flag << 5
        | (char)command->ball_placement_team << 4
        | (char)command->in_game << 3
        | (char)command->robot_position_init << 2
        | (char)command->dribble_state << 1
        | (char)command->dribble_advance;
    buffer_index += 1;
    // Dribble
    tmp_u16= htons(command->dribble_enabble_error);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16= htons(command->dribble_target_ball_x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16= htons(command->dribble_target_ball_y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    buffer[buffer_index] = (char)(command->dribble_type);
    buffer_index += 1;
    // Kick
    buffer[buffer_index] =
        (char)(command->ball_kick_state) << 3
        | (char)(command->free_kick_flag) << 2
        | (char)(command->ball_kick) << 1
        | (char)(command->kick_straight);
    buffer_index += 1;
    tmp_u16 = htons(command->ball_target_allowable_error);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(command->target_ball_x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(command->target_ball_y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    buffer[buffer_index] = (char)(command->kick_type);
    buffer_index += 1;
    // Target position
    tmp_u16 = htons(command->robot_position_target_x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(command->robot_position_target_y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(command->robot_position_target_theta);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;

    return buffer_index;
}

int decodeStrategyPcCommand(_strategy_pc_command *command, char *buffer, uint8_t buffer_length) {
    uint8_t buffer_index = 0;
    uint16_t tmp_u16;
    // Check buffer length
    assert(buffer_length > 70);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    command->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == strategy_pc_command_data_type);
    command->data_type = strategy_pc_command_data_type;
    buffer_index += 1;

    command->halt_flag = (buffer[buffer_index] & 0b10000000) == 0b10000000;
    command->stop_game_flag = (buffer[buffer_index] & 0b1000000) == 0b1000000;
    command->ball_placement_flag = (buffer[buffer_index] & 0b100000) == 0b100000;
    command->ball_placement_team = (buffer[buffer_index] & 0b10000) == 0b10000;
    command->in_game = (buffer[buffer_index] & 0b1000) == 0b1000;
    command->robot_position_init = (buffer[buffer_index] & 0b100) == 0b100;

    // Dribble
    command->dribble_state = (buffer[buffer_index] & 0b10) == 0b10;
    command->dribble_advance = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->dribble_enabble_error = (uint16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->dribble_target_ball_x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->dribble_target_ball_y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    command->dribble_type = (uint8_t)buffer[buffer_index];
    buffer_index += 1;

    // Kick
    command->ball_kick_state = (buffer[buffer_index] & 0b1000) == 0b1000;
    command->free_kick_flag = (buffer[buffer_index] & 0b100) == 0b100;
    command->ball_kick = (buffer[buffer_index] & 0b10) == 0b10;
    command->kick_straight = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->ball_target_allowable_error = (uint16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->target_ball_x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->target_ball_y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    command->kick_type = (uint8_t)buffer[buffer_index];
    buffer_index += 1;

    // Target position
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->robot_position_target_x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->robot_position_target_y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    command->robot_position_target_theta = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;

    return buffer_index;
}

int encodeVisionData(_vision_data *vision_data, char *buffer) {
    uint16_t buffer_index = 0;
    uint16_t tmp_u16;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = vision_data_data_type;
    buffer_index += 1;

    // Current Pose
    tmp_u16 = htons(vision_data->current_pose.x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->current_pose.y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->current_pose.theta);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->current_pose.vx);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->current_pose.vy);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->current_pose.omega);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    buffer[buffer_index] =
        (char) vision_data->current_pose.camera_valid << 1
        | (char) vision_data->current_pose.data_valid;
    buffer_index += 1;

    // Ball Position
    tmp_u16 = htons(vision_data->ball_position.x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->ball_position.y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->ball_position.vx);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(vision_data->ball_position.vy);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    buffer[buffer_index] =
        (char) vision_data->ball_position.camera_valid << 1
        | (char) vision_data->ball_position.data_valid;
    buffer_index += 1;

    // Obstacles
    assert(vision_data->number_of_obstacles < 32);
    buffer[buffer_index] = vision_data->number_of_obstacles;
    buffer_index += 1;
    for (int obstacle_index = 0; obstacle_index < vision_data->number_of_obstacles; obstacle_index++) {
        tmp_u16 = htons(vision_data->obstacles[obstacle_index].x);
        memcpy(&buffer[buffer_index], &tmp_u16, 2);
        buffer_index += 2;
        tmp_u16 = htons(vision_data->obstacles[obstacle_index].y);
        memcpy(&buffer[buffer_index], &tmp_u16, 2);
        buffer_index += 2;
        tmp_u16 = htons(vision_data->obstacles[obstacle_index].theta);
        memcpy(&buffer[buffer_index], &tmp_u16, 2);
        buffer_index += 2;
        tmp_u16 = htons(vision_data->obstacles[obstacle_index].vx);
        memcpy(&buffer[buffer_index], &tmp_u16, 2);
        buffer_index += 2;
        tmp_u16 = htons(vision_data->obstacles[obstacle_index].vy);
        memcpy(&buffer[buffer_index], &tmp_u16, 2);
        buffer_index += 2;
        tmp_u16 = htons(vision_data->obstacles[obstacle_index].omega);
        memcpy(&buffer[buffer_index], &tmp_u16, 2);
        buffer_index += 2;
        buffer[buffer_index] =
            (char) vision_data->obstacles[obstacle_index].camera_valid << 1
            | (char) vision_data->obstacles[obstacle_index].data_valid;
        buffer_index += 1;
    }

    return buffer_index;
}

int decodeVisionData(_vision_data *vision_data, char *buffer, uint16_t buffer_length) {
    uint16_t buffer_index = 0;
    uint16_t tmp_u16;
    uint32_t tmp_u32;
    // Check buffer length
    assert(buffer_length > 647);    // Need update max length

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    vision_data->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == vision_data_data_type);
    vision_data->data_type = vision_data_data_type;
    buffer_index += 1;

    // Current Pose
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->current_pose.x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->current_pose.y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->current_pose.theta = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->current_pose.vx = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->current_pose.vy = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->current_pose.omega = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    vision_data->current_pose.camera_valid = (buffer[buffer_index] & 0b10) == 0b10;
    vision_data->current_pose.data_valid = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;

    // Ball Position
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->ball_position.x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->ball_position.y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->ball_position.vx = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    vision_data->ball_position.vy = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    vision_data->ball_position.camera_valid = (buffer[buffer_index] & 0b10) == 0b10;
    vision_data->ball_position.data_valid = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;

    // Obstacles
    vision_data->number_of_obstacles = buffer[buffer_index];
    buffer_index += 1;
    for (int obstacle_index = 0; obstacle_index < vision_data->number_of_obstacles; obstacle_index++) {
        memcpy(&tmp_u16, &buffer[buffer_index], 2);
        vision_data->obstacles[obstacle_index].x = (int16_t)ntohs(tmp_u16);
        buffer_index += 2;
        memcpy(&tmp_u16, &buffer[buffer_index], 2);
        vision_data->obstacles[obstacle_index].y = (int16_t)ntohs(tmp_u16);
        buffer_index += 2;
        memcpy(&tmp_u16, &buffer[buffer_index], 2);
        vision_data->obstacles[obstacle_index].theta = (int16_t)ntohs(tmp_u16);
        buffer_index += 2;
        memcpy(&tmp_u16, &buffer[buffer_index], 2);
        vision_data->obstacles[obstacle_index].vx = (int16_t)ntohs(tmp_u16);
        buffer_index += 2;
        memcpy(&tmp_u16, &buffer[buffer_index], 2);
        vision_data->obstacles[obstacle_index].vy = (int16_t)ntohs(tmp_u16);
        buffer_index += 2;
        memcpy(&tmp_u16, &buffer[buffer_index], 2);
        vision_data->obstacles[obstacle_index].omega = (int16_t)ntohs(tmp_u16);
        buffer_index += 2;
        vision_data->obstacles[obstacle_index].camera_valid = (buffer[buffer_index] & 0b10) == 0b10;
        vision_data->obstacles[obstacle_index].data_valid = (buffer[buffer_index] & 0b1) == 0b1;
        buffer_index += 1;
    }

    return buffer_index;
}

int encodeManualContollerData(_manual_controller_data *controller_data, char *buffer) {
    uint8_t buffer_index = 0;
    uint32_t tmp_u32;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = manual_controller_data_type;
    buffer_index += 1;

    buffer[buffer_index] =
        (char)controller_data->controller_start << 4
        | (char)controller_data->dribbler_on << 3
        | (char)controller_data->kick_straight << 2
        | (char)controller_data->kick_tip << 1
        | (char)controller_data->emergency_stop;
    buffer_index += 1;

    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    controller_data->robot_vx = (int32_t)ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    controller_data->robot_vy = (int32_t)ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    controller_data->robot_vw = (int32_t)ntohl(tmp_u32);
    buffer_index += 4;

    return buffer_index;
}

int decodeManualContollerData(_manual_controller_data *controller_data, char *buffer, uint8_t buffer_length) {
    uint8_t buffer_index = 0;
    uint32_t tmp_u32;
    // Check buffer length
    assert(buffer_length > 14);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    controller_data->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == manual_controller_data_type);
    controller_data->data_type = manual_controller_data_type;
    buffer_index += 1;

    controller_data->controller_start = (buffer[buffer_index] & 0b10000) == 0b10000;
    controller_data->dribbler_on = (buffer[buffer_index] & 0b1000) == 0b1000;
    controller_data->kick_straight = (buffer[buffer_index] & 0b100) == 0b100;
    controller_data->kick_tip = (buffer[buffer_index] & 0b10) == 0b10;
    controller_data->emergency_stop = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;

    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    controller_data->robot_vx = (int32_t)ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    controller_data->robot_vy = (int32_t)ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    controller_data->robot_vw = (int32_t)ntohl(tmp_u32);
    buffer_index += 4;

    return buffer_index;
}

int encodeRobotOdometryData(_robot_odometry_data *odometry_data, char *buffer) {
    uint8_t buffer_index = 0;
    uint16_t tmp_u16;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = robot_odometry_data_type;
    buffer_index += 1;

    tmp_u16 = htons(odometry_data->robot_position_x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(odometry_data->robot_position_y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(odometry_data->robot_position_theta);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;

    return buffer_index;
}

int decodeRobotOdometryData(_robot_odometry_data *odometry_data, char *buffer, uint8_t buffer_length) {
    uint8_t buffer_index = 0;
    uint16_t tmp_u16;
    // Check buffer length
    assert(buffer_length > 7);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    odometry_data->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == robot_odometry_data_type);
    odometry_data->data_type = robot_odometry_data_type;
    buffer_index += 1;

    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    odometry_data->robot_position_x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    odometry_data->robot_position_y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    odometry_data->robot_position_theta = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;

    return buffer_index;
}

int encodeRobotObservedBallData(_robot_observed_ball_data *ball_data, char *buffer) {
    uint8_t buffer_index = 0;
    uint16_t tmp_u16;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = robot_observed_ball_data_type;
    buffer_index += 1;

    tmp_u16 = htons(ball_data->ball_position_x);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;
    tmp_u16 = htons(ball_data->ball_position_y);
    memcpy(&buffer[buffer_index], &tmp_u16, 2);
    buffer_index += 2;

    return buffer_index;
}

int decodeRobotObservedBallData(_robot_observed_ball_data *ball_data, char *buffer, uint8_t buffer_length) {
    uint8_t buffer_index = 0;
    uint16_t tmp_u16;
    // Check buffer length
    assert(buffer_length > 5);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    ball_data->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == robot_observed_ball_data_type);
    ball_data->data_type = robot_observed_ball_data_type;
    buffer_index += 1;

    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    ball_data->ball_position_x = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;
    memcpy(&tmp_u16, &buffer[buffer_index], 2);
    ball_data->ball_position_y = (int16_t)ntohs(tmp_u16);
    buffer_index += 2;

    return buffer_index;
}
