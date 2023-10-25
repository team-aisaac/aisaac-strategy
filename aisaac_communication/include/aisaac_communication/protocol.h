#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>

#define MAX_OBSTACLE_NUM 31
#ifndef htons
#define htons(x) __builtin_bswap16(x)
#define ntohs(x) __builtin_bswap16(x)
#define htonl(x) __builtin_bswap32(x)
#define ntohl(x) __builtin_bswap32(x)
#endif


typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;

    bool halt_flag;
    bool stop_game_flag;
    bool ball_placement_flag;
    bool ball_placement_team;
    bool in_game;
    bool robot_position_init;
    // Dribble
    bool dribble_state;
    bool dribble_advance;
    uint16_t dribble_enabble_error;
    int16_t dribble_target_ball_x;
    int16_t dribble_target_ball_y;
    uint8_t dribble_type;
    // Kick
    bool ball_kick_state;
    bool free_kick_flag;
    bool ball_kick;
    bool kick_straight;
    uint16_t ball_target_allowable_error;
    int16_t target_ball_x;
    int16_t target_ball_y;
    uint8_t kick_type;
    // Target position
    int16_t robot_position_target_x;
    int16_t robot_position_target_y;
    int16_t robot_position_target_theta;
} _strategy_pc_command;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t theta;
    int16_t vx;
    int16_t vy;
    int16_t omega;
    bool camera_valid;
    bool data_valid;
} _ssl_vision_robot_data;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t vx;
    int16_t vy;
    bool camera_valid;
    bool data_valid;
} _ssl_vision_ball_data;

typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;

    _ssl_vision_robot_data current_pose;
    _ssl_vision_ball_data ball_position;
    uint8_t number_of_obstacles;
    _ssl_vision_robot_data obstacles[MAX_OBSTACLE_NUM];
} _vision_data;

// Add Ver. 1.2
typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;

    bool controller_start;
    int32_t robot_vx;
    int32_t robot_vy;
    int32_t robot_vw;
    bool dribbler_on;
    bool kick_straight;
    bool kick_tip;
    bool emergency_stop;
} _manual_controller_data;

// Add Ver. 1.1
typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;

    int16_t robot_position_x;
    int16_t robot_position_y;
    int16_t robot_position_theta;
} _robot_odometry_data;

typedef struct {
    uint8_t protocol_version;
    uint8_t data_type;
    
    int16_t ball_position_x;
    int16_t ball_position_y;
} _robot_observed_ball_data;

int encodeStrategyPcCommand(_strategy_pc_command *command, char *buffer);
int decodeStrategyPcCommand(_strategy_pc_command *command, char *buffer, uint8_t buffer_length);
int encodeVisionData(_vision_data *vision_data, char *buffer);
int decodeVisionData(_vision_data *vision_data, char *buffer, uint16_t buffer_length);
int encodeManualContollerData(_manual_controller_data *controller_data, char *buffer);
int decodeManualContollerData(_manual_controller_data *controller_data, char *buffer, uint8_t buffer_length);
int encodeRobotOdometryData(_robot_odometry_data *odometry_data, char *buffer);
int decodeRobotOdometryData(_robot_odometry_data *odometry_data, char *buffer, uint8_t buffer_length);
int encodeRobotObservedBallData(_robot_observed_ball_data *ball_data, char *buffer);
int decodeRobotObservedBallData(_robot_observed_ball_data *ball_data, char *buffer, uint8_t buffer_length);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _PROTOCOL_H_ */
