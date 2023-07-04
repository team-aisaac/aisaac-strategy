#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>

#define MAX_OBSTACLE_NUM 31

typedef struct {
    int32_t x;
    int32_t y;
    int32_t theta;
} _Position;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t theta;
    int32_t vx;
    int32_t vy;
} _Obstacle;

typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;

    _Position goal_pose;
    _Position middle_goal_pose;
    bool prohibited_zone_ignore;
    bool middle_target_flag;
    bool halt_flag;
    // Kick
    uint32_t kick_power;
    _Position ball_goal;
    int32_t ball_target_allowable_error;
    uint8_t kick_type;
    bool ball_kick_state;
    bool ball_kick;
    bool ball_kick_active;
    bool free_kick_flag;
    // Dribble
    uint32_t dribble_power;
    _Position dribble_goal;
    int32_t dribble_complete_distance;
    bool dribble_state;
    bool dribbler_active;
} _strategy_pc_command;

typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;
    _Position dwa_position;
} _dwa_result;

typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;
    _Position current_pose;
    _Position ball_position;
    uint8_t number_of_obstacles;
    _Obstacle obstacles[MAX_OBSTACLE_NUM];
} _vision_data;

int encodeStrategyPcCommand(_strategy_pc_command *command, unsigned char *buffer);
int decodeStrategyPcCommand(_strategy_pc_command *command, unsigned char *buffer, uint8_t buffer_length);
int encodeDwaResult(_dwa_result *dwa_result, unsigned char *buffer);
int decodeDwaResult(_dwa_result *dwa_result, unsigned char *buffer, uint8_t buffer_length);
int encodeVisionData(_vision_data *vision_data, unsigned char *buffer);
int decodeVisionData(_vision_data *vision_data, unsigned char *buffer, uint16_t buffer_length);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _PROTOCOL_H_ */
