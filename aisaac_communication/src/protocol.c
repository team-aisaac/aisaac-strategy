#include "aisaac_communication/protocol.h"
#include <stdint.h>
#include <arpa/inet.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#define RX_ASSERT(x) (assert(send.x == recv.x));

const uint8_t protocol_version = 0b00100001;    // Ver. 2.1
const uint8_t strategy_pc_command_data_type = 0b10100001;   // 5-1
const uint8_t dwa_result_data_type = 0b10100010;   // 5-2
const uint8_t vision_data_data_type = 0b10100011;   // 5-3

int encodeStrategyPcCommand(_strategy_pc_command *command, unsigned char *buffer) {
    uint8_t buffer_index = 0;
    uint32_t tmp_u32;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = strategy_pc_command_data_type;
    buffer_index += 1;

    tmp_u32 = htonl(command->goal_pose.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->goal_pose.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->goal_pose.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->middle_goal_pose.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->middle_goal_pose.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->middle_goal_pose.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    buffer[buffer_index] = (char)command->prohibited_zone_ignore << 2
        | (char)command->middle_target_flag << 1
        | (char)command->halt_flag;
    buffer_index += 1;
    // Kick
    tmp_u32 = htonl(command->kick_power);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->ball_goal.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->ball_goal.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->ball_goal.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->ball_target_allowable_error);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    buffer[buffer_index] = (char)(command->kick_type);
    buffer_index += 1;
    buffer[buffer_index] = (char)(command->ball_kick_state) << 3
        | (char)(command->ball_kick) << 2
        | (char)(command->ball_kick_active) << 1
        | (char)(command->free_kick_flag);
    buffer_index += 1;
    // Dribble
    tmp_u32= htonl(command->dribble_power);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->dribble_goal.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->dribble_goal.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->dribble_goal.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(command->dribble_complete_distance);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    buffer[buffer_index] = (char)(command->dribble_state) << 1
        | (char)(command->dribbler_active);
    buffer_index += 1;

    return buffer_index;
}

int decodeStrategyPcCommand(_strategy_pc_command *command, unsigned char *buffer, uint8_t buffer_length) {
    uint8_t buffer_index = 0;
    uint32_t tmp_u32;
    // Check buffer length
    assert(buffer_length > 70);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    command->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == strategy_pc_command_data_type);
    command->data_type = strategy_pc_command_data_type;
    buffer_index += 1;

    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->goal_pose.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->goal_pose.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->goal_pose.theta = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->middle_goal_pose.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->middle_goal_pose.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->middle_goal_pose.theta = ntohl(tmp_u32);
    buffer_index += 4;
    command->prohibited_zone_ignore = (buffer[buffer_index] & 0b100) == 0b100;
    command->middle_target_flag = (buffer[buffer_index] & 0b10) == 0b10;
    command->halt_flag = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;
    // Kick
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->kick_power = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->ball_goal.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->ball_goal.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->ball_goal.theta = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->ball_target_allowable_error = ntohl(tmp_u32);
    buffer_index += 4;
    command->kick_type = buffer[buffer_index];
    buffer_index += 1;
    command->ball_kick_state = (buffer[buffer_index] & 0b1000) == 0b1000;
    command->ball_kick = (buffer[buffer_index] & 0b100) == 0b100;
    command->ball_kick_active = (buffer[buffer_index] & 0b10) == 0b10;
    command->free_kick_flag = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;
    // Dribble
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->dribble_power = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->dribble_goal.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->dribble_goal.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->dribble_goal.theta = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    command->dribble_complete_distance = ntohl(tmp_u32);
    buffer_index += 4;
    command->dribble_state = (buffer[buffer_index] & 0b10) == 0b10;
    command->dribbler_active = (buffer[buffer_index] & 0b1) == 0b1;
    buffer_index += 1;

    return buffer_index;
}

int encodeDwaResult(_dwa_result *dwa_result, unsigned char *buffer) {
    uint8_t buffer_index = 0;
    uint32_t tmp_u32;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = dwa_result_data_type;
    buffer_index += 1;

    tmp_u32 = htonl(dwa_result->dwa_position.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(dwa_result->dwa_position.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(dwa_result->dwa_position.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;

    return buffer_index;
}

int decodeDwaResult(_dwa_result *dwa_result, unsigned char *buffer, uint8_t buffer_length) {
    uint8_t buffer_index = 0;
    uint32_t tmp_u32;
    // Check buffer length
    assert(buffer_length > 14);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    dwa_result->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == dwa_result_data_type);
    dwa_result->data_type = dwa_result_data_type;
    buffer_index += 1;

    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    dwa_result->dwa_position.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    dwa_result->dwa_position.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    dwa_result->dwa_position.theta = ntohl(tmp_u32);
    buffer_index += 4;

    return buffer_index;
}

int encodeVisionData(_vision_data *vision_data, unsigned char *buffer) {
    uint16_t buffer_index = 0;
    uint32_t tmp_u32;

    buffer[buffer_index] = protocol_version;
    buffer_index += 1;
    buffer[buffer_index] = vision_data_data_type;
    buffer_index += 1;

    // Current Pose
    tmp_u32 = htonl(vision_data->current_pose.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(vision_data->current_pose.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(vision_data->current_pose.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    // Ball Position
    tmp_u32 = htonl(vision_data->ball_position.x);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(vision_data->ball_position.y);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    tmp_u32 = htonl(vision_data->ball_position.theta);
    memcpy(&buffer[buffer_index], &tmp_u32, 4);
    buffer_index += 4;
    // Obstacles
    assert(vision_data->number_of_obstacles < 32);
    buffer[buffer_index] = vision_data->number_of_obstacles;
    buffer_index += 1;
    for (int obstacle_index = 0; obstacle_index < vision_data->number_of_obstacles; obstacle_index++) {
        tmp_u32 = htonl(vision_data->obstacles[obstacle_index].x);
        memcpy(&buffer[buffer_index], &tmp_u32, 4);
        buffer_index += 4;
        tmp_u32 = htonl(vision_data->obstacles[obstacle_index].y);
        memcpy(&buffer[buffer_index], &tmp_u32, 4);
        buffer_index += 4;
        tmp_u32 = htonl(vision_data->obstacles[obstacle_index].theta);
        memcpy(&buffer[buffer_index], &tmp_u32, 4);
        buffer_index += 4;
        tmp_u32 = htonl(vision_data->obstacles[obstacle_index].vx);
        memcpy(&buffer[buffer_index], &tmp_u32, 4);
        buffer_index += 4;
        tmp_u32 = htonl(vision_data->obstacles[obstacle_index].vy);
        memcpy(&buffer[buffer_index], &tmp_u32, 4);
        buffer_index += 4;
    }

    return buffer_index;
}

int decodeVisionData(_vision_data *vision_data, unsigned char *buffer, uint16_t buffer_length) {
    uint16_t buffer_index = 0;
    uint32_t tmp_u32;
    // Check buffer length
    assert(buffer_length > 647);

    assert((uint8_t)buffer[buffer_index] == protocol_version);
    vision_data->protocol_version = protocol_version;
    buffer_index += 1;
    assert((uint8_t)buffer[buffer_index] == vision_data_data_type);
    vision_data->data_type = vision_data_data_type;
    buffer_index += 1;

    // Current Pose
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    vision_data->current_pose.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    vision_data->current_pose.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    vision_data->current_pose.theta = ntohl(tmp_u32);
    buffer_index += 4;
    // Ball Position
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    vision_data->ball_position.x = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    vision_data->ball_position.y = ntohl(tmp_u32);
    buffer_index += 4;
    memcpy(&tmp_u32, &buffer[buffer_index], 4);
    vision_data->ball_position.theta = ntohl(tmp_u32);
    buffer_index += 4;
    // Obstacles
    vision_data->number_of_obstacles = buffer[buffer_index];
    buffer_index += 1;
    for (int obstacle_index = 0; obstacle_index < vision_data->number_of_obstacles; obstacle_index++) {
        memcpy(&tmp_u32, &buffer[buffer_index], 4);
        vision_data->obstacles[obstacle_index].x = ntohl(tmp_u32);
        buffer_index += 4;
        memcpy(&tmp_u32, &buffer[buffer_index], 4);
        vision_data->obstacles[obstacle_index].y = ntohl(tmp_u32);
        buffer_index += 4;
        memcpy(&tmp_u32, &buffer[buffer_index], 4);
        vision_data->obstacles[obstacle_index].theta = ntohl(tmp_u32);
        buffer_index += 4;
        memcpy(&tmp_u32, &buffer[buffer_index], 4);
        vision_data->obstacles[obstacle_index].vx = ntohl(tmp_u32);
        buffer_index += 4;
        memcpy(&tmp_u32, &buffer[buffer_index], 4);
        vision_data->obstacles[obstacle_index].vy = ntohl(tmp_u32);
        buffer_index += 4;
    }

    return buffer_index;
}

int main() {
    // test code
    {
        _strategy_pc_command send, recv;
        send.goal_pose.x = 1000;
        send.goal_pose.y = -1000;
        send.goal_pose.theta = 10000;
        send.middle_goal_pose.x = 2000;
        send.middle_goal_pose.y = -2000;
        send.middle_goal_pose.theta = 20000;
        send.prohibited_zone_ignore = false;
        send.middle_target_flag = true;
        send.halt_flag = false;
        send.kick_power = 3000;
        send.ball_goal.x = 4000;
        send.ball_goal.y = -4000;
        send.ball_goal.theta = 40000;
        send.ball_target_allowable_error = 5000;
        send.kick_type = 3;
        send.ball_kick_state = true;
        send.ball_kick = false;
        send.ball_kick_active = true;
        send.free_kick_flag = false;
        send.dribble_power = 6000;
        send.dribble_goal.x = 7000;
        send.dribble_goal.y = -7000;
        send.dribble_goal.theta = 70000;
        send.dribble_complete_distance = 8000;
        send.dribble_state = true;
        send.dribbler_active = false;

        char buffer[200];

        int length = encodeStrategyPcCommand(&send, &buffer[0]);
        decodeStrategyPcCommand(&recv, &buffer[0], 200);

        printf("len: %d\r\n", length);
        printf("[ ");
        for (int i = 0; i < length; i++) {
            printf("%d, ", (uint8_t)buffer[i]);
        }
        printf(" ];\r\n");

        RX_ASSERT(goal_pose.x)
        RX_ASSERT(goal_pose.y)
        RX_ASSERT(goal_pose.theta)
        RX_ASSERT(middle_goal_pose.x)
        RX_ASSERT(middle_goal_pose.y)
        RX_ASSERT(middle_goal_pose.theta)
        RX_ASSERT(prohibited_zone_ignore)
        RX_ASSERT(middle_target_flag)
        RX_ASSERT(halt_flag)
        // Kick
        RX_ASSERT(kick_power)
        RX_ASSERT(ball_goal.x)
        RX_ASSERT(ball_goal.y)
        RX_ASSERT(ball_goal.theta)
        RX_ASSERT(ball_target_allowable_error)
        RX_ASSERT(kick_type)
        // RX_ASSERT(ball_kick_state)
        RX_ASSERT(ball_kick)
        // RX_ASSERT(ball_kick_active)
        RX_ASSERT(free_kick_flag)
        // Dribble
        RX_ASSERT(dribble_power)
        RX_ASSERT(dribble_goal.x)
        RX_ASSERT(dribble_goal.y)
        RX_ASSERT(dribble_goal.theta)
        RX_ASSERT(dribble_complete_distance)
        RX_ASSERT(dribble_state)
        RX_ASSERT(dribbler_active)
    }

    {
        _dwa_result send, recv;
        send.dwa_position.x = 100;
        send.dwa_position.y = -10000;
        send.dwa_position.theta = 200;

        char buffer[15];

        int length = encodeDwaResult(&send, &buffer[0]);
        decodeDwaResult(&recv, &buffer[0], 15);

        printf("len: %d\r\n", length);
        printf("[ ");
        for (int i = 0; i < length; i++) {
            printf("%d, ", (uint8_t)buffer[i]);
        }
        printf(" ];\r\n");

        RX_ASSERT(dwa_position.x)
        RX_ASSERT(dwa_position.y)
        RX_ASSERT(dwa_position.theta)
    }

    {
        _vision_data send, recv;
        send.current_pose.x = 100;
        send.current_pose.y = -100;
        send.current_pose.theta = 1000;
        send.ball_position.x = 200;
        send.ball_position.y = -200;
        send.number_of_obstacles = 31;
        for (int i = 0; i < send.number_of_obstacles; i++) {
            send.obstacles[i].x = 2000 + i;
            send.obstacles[i].y = -2000 - i;
            send.obstacles[i].theta = 20000 + i;
            send.obstacles[i].vx = 3000 + i;
            send.obstacles[i].vy = -3000 - i;
        }

        char buffer[680];

        int length = encodeVisionData(&send, &buffer[0]);
        decodeVisionData(&recv, &buffer[0], 680);

        printf("len: %d\r\n", length);
        printf("[ ");
        for (int i = 0; i < length; i++) {
            printf("%d, ", (uint8_t)buffer[i]);
        }
        printf(" ];\r\n");

        RX_ASSERT(current_pose.x)
        RX_ASSERT(current_pose.y)
        RX_ASSERT(current_pose.theta)
        RX_ASSERT(ball_position.x)
        RX_ASSERT(ball_position.y)
        for (int i = 0; i < send.number_of_obstacles; i++) {
            RX_ASSERT(obstacles[i].x)
            RX_ASSERT(obstacles[i].y)
            RX_ASSERT(obstacles[i].theta)
        }
    }

    return 0;
}
