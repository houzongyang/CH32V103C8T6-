#ifndef __CONE_DETECT_H
#define __CONE_DETECT_H

#include <stdint.h>
#include "rplidar_360.h"

typedef struct
{
    uint8_t found;
    uint8_t gate_found;

    float cone1_x;
    float cone1_y;
    float cone2_x;
    float cone2_y;

    float center_x;
    float center_y;
    float center_dist;
    float center_angle_deg;

    float gate_width;

    float normal_x;
    float normal_y;
    float normal_angle_deg;
    float lateral_error_mm;
} ConeDetectResult_t;

typedef enum
{
    CONE_DRIVE_STATE_SEARCH = 0,
    CONE_DRIVE_STATE_APPROACH,
    CONE_DRIVE_STATE_PASS,
    CONE_DRIVE_STATE_CLEAR_GATE,
    CONE_DRIVE_STATE_DONE
} ConeDriveState_t;

typedef enum
{
    CONE_DRIVE_MODE_UNKNOWN = 0,
    CONE_DRIVE_MODE_LEFT_FRONT,
    CONE_DRIVE_MODE_RIGHT_FRONT
} ConeDriveMode_t;

typedef struct
{
    uint16_t center_start_distance_mm;
    uint16_t pass_start_distance_mm;
    uint16_t pass_time_ms;
    uint16_t clear_gate_time_ms;
    uint16_t search_reacquire_time_ms;
    uint16_t emergency_stop_distance_mm;

    uint8_t speed_search;
    uint8_t speed_turn;
    uint8_t speed_approach;
    uint8_t speed_centering;
    uint8_t speed_pass;

    int16_t scan_steer_angle;
    uint8_t lost_max_count;
    uint16_t lost_timeout_ms;

    int16_t kp_gate_angle_q8;
    int16_t kd_gate_angle_q8;
    int16_t kp_center_angle_q8;
    int16_t kd_center_angle_q8;
    int16_t kp_yaw_q8;
    int16_t kd_yaw_q8;

    int16_t center_ok_deg;
    int16_t yaw_ok_deg;
    int16_t yaw_sign;
    int16_t target_yaw_deg;
    int16_t max_steer_angle;
    int16_t steer_trim_angle;

    uint8_t target_gate_count;
} ConeDriveConfig_t;

typedef struct
{
    ConeDriveState_t state;

    int16_t last_gate_error;
    int16_t last_center_error;
    int16_t last_yaw_error;

    uint32_t state_start_ms;
    uint32_t last_seen_ms;
    uint32_t last_lidar_round_seq;
    uint8_t lost_count;
    uint8_t passed_gate_count;

    ConeDriveMode_t gate_mode;
    int16_t last_steer_angle;
    ConeDetectResult_t gate;
} ConeDriveContext_t;

void ConeDetect_Init(void);
uint8_t ConeDetect_Update(const RPLIDAR360_Frame_t *frame,
                          ConeDetectResult_t *result);
void ConeDetect_ShowOLED(const ConeDetectResult_t *result);

void ConeDrive_ConfigDefault(ConeDriveConfig_t *cfg);
void ConeDrive_Init(ConeDriveContext_t *ctx);
void ConeDrive_Update(ConeDriveContext_t *ctx,
                      const ConeDriveConfig_t *cfg,
                      uint32_t now_ms);

#endif
