#include "cone_detect.h"
#include "Motor.h"
#include "Servo.h"
#include "JY901.h"
#include "Oled.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define PI_F                         3.1415926f



#define PASS_CENTER_OK_DEG            6
#define PASS_SIDE_SAFE_X_MM         240.0f
#define PASS_SIDE_SAFE_Y_MM         900.0f
#define SIDE_ESCAPE_STEER_DEG        18


#define SCAN_LEFT_DEG               -105
#define SCAN_RIGHT_DEG               105
#define MIN_POINT_DIST_MM            100u
#define MAX_POINT_DIST_MM            3300u

#define MAX_POINTS                   220u
#define MAX_CLUSTERS                  32u

#define CLUSTER_GAP_MM               330.0f
#define CLUSTER_MAX_WIDTH_MM        1100.0f
#define CLUSTER_MIN_Y_MM              80.0f

#define GATE_WIDTH_MAX_MM           1600.0f

#define APPROACH_BACK_MM             950.0f
#define LINE_LOOKAHEAD_MM           1300.0f
#define GATE_TARGET_OFFSET_MM        300.0f
#define GATE_TARGET_ARRIVE_MM        200.0f
#define SINGLE_TARGET_GUARD_MM       900.0f
#define LINE_OK_MM                   120.0f
#define LINE_OK_DEG                   10
#define LINE_STEER_MAX_DEG            36

#define FRONT_EMERGENCY_LEFT_DEG      -18
#define FRONT_EMERGENCY_RIGHT_DEG      18
#define LIDAR_FRONT_OFFSET_DEG        0

typedef struct
{
    float x;
    float y;
    float angle_deg;
    uint16_t dist_mm;
} ConePoint_t;

typedef struct
{
    uint8_t valid;
    uint16_t count;

    float x_sum;
    float y_sum;
    float x;
    float y;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float width;
    float dist;
    float angle_deg;
} ConeCluster_t;

static void GetGateLeftRightCones(const ConeDetectResult_t *gate,
                                  float *left_x,
                                  float *left_y,
                                  float *right_x,
                                  float *right_y);

static int16_t CalcLineSteer(const ConeDetectResult_t *gate,
                             ConeDriveContext_t *ctx,
                             const ConeDriveConfig_t *cfg);

static ConePoint_t detect_points[MAX_POINTS];
static ConeCluster_t detect_clusters[MAX_CLUSTERS];
static RPLIDAR360_Frame_t drive_frame;

static int16_t AbsS16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

static int16_t CalcCenteringSteerSafe(const ConeDetectResult_t *gate,
                                      ConeDriveContext_t *ctx,
                                      const ConeDriveConfig_t *cfg)
{
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    int16_t steer;

    steer = CalcLineSteer(gate, ctx, cfg);

    if((gate == 0) || (gate->gate_found == 0u))
    {
        return steer;
    }

    GetGateLeftRightCones(gate, &left_x, &left_y, &right_x, &right_y);

    /*
     * 左锥桶太近：强制向右修正
     * 右锥桶太近：强制向左修正
     *
     * 注意：这里默认 steer > 0 是右打，steer < 0 是左打。
     * 如果你实车方向相反，把 +SIDE_ESCAPE_STEER_DEG 和 -SIDE_ESCAPE_STEER_DEG 对调。
     */
    if((left_y > 0.0f) &&
       (left_y < PASS_SIDE_SAFE_Y_MM) &&
       (left_x > -PASS_SIDE_SAFE_X_MM))
    {
        if(steer < SIDE_ESCAPE_STEER_DEG)
        {
            steer = SIDE_ESCAPE_STEER_DEG;
        }
    }

    if((right_y > 0.0f) &&
       (right_y < PASS_SIDE_SAFE_Y_MM) &&
       (right_x < PASS_SIDE_SAFE_X_MM))
    {
        if(steer > -SIDE_ESCAPE_STEER_DEG)
        {
            steer = -SIDE_ESCAPE_STEER_DEG;
        }
    }

    return steer;
}

static void GetGateLeftRightCones(const ConeDetectResult_t *gate,
                                  float *left_x,
                                  float *left_y,
                                  float *right_x,
                                  float *right_y)
{
    if(gate->cone1_x <= gate->cone2_x)
    {
        *left_x = gate->cone1_x;
        *left_y = gate->cone1_y;
        *right_x = gate->cone2_x;
        *right_y = gate->cone2_y;
    }
    else
    {
        *left_x = gate->cone2_x;
        *left_y = gate->cone2_y;
        *right_x = gate->cone1_x;
        *right_y = gate->cone1_y;
    }
}

static uint8_t IsGateSideUnsafe(const ConeDetectResult_t *gate)
{
    float left_x;
    float left_y;
    float right_x;
    float right_y;

    if((gate == 0) || (gate->gate_found == 0u))
    {
        return 0u;
    }

    GetGateLeftRightCones(gate, &left_x, &left_y, &right_x, &right_y);

    /*
     * 左锥桶在前方较近，并且离车体中心线太近：危险
     */
    if((left_y > 0.0f) &&
       (left_y < PASS_SIDE_SAFE_Y_MM) &&
       (left_x > -PASS_SIDE_SAFE_X_MM))
    {
        return 1u;
    }

    /*
     * 右锥桶在前方较近，并且离车体中心线太近：危险
     */
    if((right_y > 0.0f) &&
       (right_y < PASS_SIDE_SAFE_Y_MM) &&
       (right_x < PASS_SIDE_SAFE_X_MM))
    {
        return 1u;
    }

    return 0u;
}









static int16_t ClampS16(int16_t v, int16_t min_v, int16_t max_v)
{
    if(v > max_v)
    {
        return max_v;
    }

    if(v < min_v)
    {
        return min_v;
    }

    return v;
}



static int16_t FloatToS16(float v)
{
    if(v >= 0.0f)
    {
        return (int16_t)(v + 0.5f);
    }

    return (int16_t)(v - 0.5f);
}

static int16_t DivQ8(int32_t value)
{
    if(value >= 0)
    {
        return (int16_t)((value + 128) >> 8);
    }

    return (int16_t)(-(((-value) + 128) >> 8));
}

static int16_t PD(int16_t error,
                  int16_t *last_error,
                  int16_t kp_q8,
                  int16_t kd_q8,
                  int16_t max_output)
{
    int16_t diff;
    int32_t out;

    diff = (int16_t)(error - *last_error);
    *last_error = error;

    out = (int32_t)kp_q8 * (int32_t)error +
          (int32_t)kd_q8 * (int32_t)diff;

    return ClampS16(DivQ8(out), (int16_t)(-max_output), max_output);
}

static int16_t DirectSteerFromAngle(int16_t angle_error,
                                    int16_t max_steer)
{
    int32_t steer;

    steer = (int32_t)angle_error * 3;

    return ClampS16((int16_t)steer,
                    (int16_t)(-max_steer),
                    max_steer);
}

static int16_t DirectSteerFromAngleGain(int16_t angle_error,
                                        int16_t max_steer,
                                        int16_t gain)
{
    int32_t steer;

    if(AbsS16(angle_error) <= 2)
    {
        return 0;
    }

    steer = (int32_t)angle_error * (int32_t)gain;

    return ClampS16((int16_t)steer,
                    (int16_t)(-max_steer),
                    max_steer);
}

static int16_t NormalizeAngle180(int16_t angle)
{
    while(angle > 180)
    {
        angle -= 360;
    }

    while(angle < -180)
    {
        angle += 360;
    }

    return angle;
}

static int16_t GetYawErrorDeg(const ConeDriveConfig_t *cfg)
{
    int16_t yaw_now;
    int16_t yaw_error;

    yaw_now = FloatToS16(JY901_GetYawRelative());
    yaw_now = (int16_t)(yaw_now * cfg->yaw_sign);
    yaw_error = (int16_t)(yaw_now - cfg->target_yaw_deg);

    return NormalizeAngle180(yaw_error);
}

static int16_t YawHoldPD(int16_t yaw_error,
                         int16_t *last_yaw_error,
                         const ConeDriveConfig_t *cfg)
{
    int16_t out;

    out = PD(yaw_error,
             last_yaw_error,
             cfg->kp_yaw_q8,
             cfg->kd_yaw_q8,
             cfg->max_steer_angle);

    return (int16_t)(-out);
}

static uint16_t SignedDegToIndex(int16_t signed_deg)
{
    int16_t lidar_deg = (int16_t)(signed_deg + LIDAR_FRONT_OFFSET_DEG);

    while(lidar_deg >= 360)
    {
        lidar_deg -= 360;
    }

    while(lidar_deg < 0)
    {
        lidar_deg += 360;
    }

    return (uint16_t)lidar_deg;
}

static void SetSpeed(uint8_t speed)
{
    if(speed > 100u)
    {
        speed = 100u;
    }

    if((speed > 0u) && (speed < 38u))
    {
        speed = 38u;
    }

    Car_SetRearSpeed(speed);
}

static void SetSteerRaw(int16_t angle, const ConeDriveConfig_t *cfg)
{
    static int16_t last_angle = 0;
    int16_t delta;

    if(cfg != 0)
    {
        angle = (int16_t)(angle + cfg->steer_trim_angle);
    }

    angle = ClampS16(angle, -45, 45);

    delta = (int16_t)(angle - last_angle);
    if(delta > 8)
    {
        delta = 8;
    }
    else if(delta < -8)
    {
        delta = -8;
    }

    last_angle = (int16_t)(last_angle + delta);
    turnAngle(last_angle);
}
static int16_t CalcLineSteer(const ConeDetectResult_t *gate,
                             ConeDriveContext_t *ctx,
                             const ConeDriveConfig_t *cfg)
{
    int16_t center_angle;
    int16_t normal_angle;
    int16_t aim_angle;
    int16_t steer;

    (void)ctx;
    (void)cfg;

    center_angle = FloatToS16(gate->center_angle_deg);
    normal_angle = FloatToS16(gate->normal_angle_deg);

    aim_angle = (int16_t)(center_angle + normal_angle / 5);

    if(AbsS16(aim_angle) <= 4)
    {
        return 0;
    }

    steer = DirectSteerFromAngleGain(aim_angle,
                                     LINE_STEER_MAX_DEG,
                                     2);

    return steer;
}
static uint8_t IsFrontEmergencyStop(const RPLIDAR360_Frame_t *frame,
                                    const ConeDriveConfig_t *cfg)
{
    int16_t deg;

    if((frame == 0) || (cfg == 0) ||
       (cfg->emergency_stop_distance_mm == 0u))
    {
        return 0u;
    }

    for(deg = FRONT_EMERGENCY_LEFT_DEG; deg <= FRONT_EMERGENCY_RIGHT_DEG; deg++)
    {
        uint16_t idx = SignedDegToIndex(deg);
        const RPLIDAR360_Bin_t *bin = &frame->bin[idx];

        if((idx < RPLIDAR360_BIN_COUNT) &&
           (bin->valid != 0u) &&
           (bin->distance_mm > 0u) &&
           (bin->distance_mm <= cfg->emergency_stop_distance_mm))
        {
            return 1u;
        }
    }

    return 0u;
}

static void ClearResult(ConeDetectResult_t *result)
{
    if(result != 0)
    {
        memset(result, 0, sizeof(ConeDetectResult_t));
    }
}

static uint8_t ConeDetect_UpdateWithSide(const RPLIDAR360_Frame_t *frame,
                                         ConeDetectResult_t *result,
                                         int8_t preferred_side);

static uint16_t CollectFrontPoints(const RPLIDAR360_Frame_t *frame,
                                   ConePoint_t points[MAX_POINTS])
{
    uint16_t count = 0u;
    int16_t deg;

    for(deg = SCAN_LEFT_DEG; deg <= SCAN_RIGHT_DEG; deg++)
    {
        uint16_t idx = SignedDegToIndex(deg);
        const RPLIDAR360_Bin_t *bin = &frame->bin[idx];
        uint16_t d;
        float rad;

        if(bin->valid == 0u)
        {
            continue;
        }

        d = bin->distance_mm;
        if((d < MIN_POINT_DIST_MM) || (d > MAX_POINT_DIST_MM))
        {
            continue;
        }

        if(count >= MAX_POINTS)
        {
            break;
        }

        rad = (float)deg * PI_F / 180.0f;
        points[count].angle_deg = (float)deg;
        points[count].dist_mm = d;
        points[count].x = (float)d * sinf(rad);
        points[count].y = (float)d * cosf(rad);
        count++;
    }

    return count;
}

static uint8_t BuildClusters(const ConePoint_t points[MAX_POINTS],
                             uint16_t point_count,
                             ConeCluster_t clusters[MAX_CLUSTERS],
                             uint8_t *cluster_count)
{
    uint16_t i = 0u;
    uint8_t cnum = 0u;

    memset(clusters, 0, sizeof(ConeCluster_t) * MAX_CLUSTERS);

    while((i < point_count) && (cnum < MAX_CLUSTERS))
    {
        ConeCluster_t *c = &clusters[cnum];
        uint16_t j = i + 1u;

        c->count = 1u;
        c->x_sum = points[i].x;
        c->y_sum = points[i].y;
        c->min_x = points[i].x;
        c->max_x = points[i].x;
        c->min_y = points[i].y;
        c->max_y = points[i].y;

        while(j < point_count)
        {
            float dx = points[j].x - points[j - 1u].x;
            float dy = points[j].y - points[j - 1u].y;
            float gap = sqrtf(dx * dx + dy * dy);

            if(gap > CLUSTER_GAP_MM)
            {
                break;
            }

            c->count++;
            c->x_sum += points[j].x;
            c->y_sum += points[j].y;

            if(points[j].x < c->min_x) c->min_x = points[j].x;
            if(points[j].x > c->max_x) c->max_x = points[j].x;
            if(points[j].y < c->min_y) c->min_y = points[j].y;
            if(points[j].y > c->max_y) c->max_y = points[j].y;

            j++;
        }

        c->x = c->x_sum / (float)c->count;
        c->y = c->y_sum / (float)c->count;
        c->dist = sqrtf(c->x * c->x + c->y * c->y);
        c->angle_deg = atan2f(c->x, c->y) * 180.0f / PI_F;

        {
            float wx = c->max_x - c->min_x;
            float wy = c->max_y - c->min_y;
            c->width = sqrtf(wx * wx + wy * wy);
        }

        if((c->y > CLUSTER_MIN_Y_MM) &&
           (c->width <= CLUSTER_MAX_WIDTH_MM))
        {
            c->valid = 1u;
        }

        cnum++;
        i = j;
    }

    *cluster_count = cnum;
    return (cnum > 0u) ? 1u : 0u;
}

static uint8_t MakeGate(const ConeCluster_t *a,
                        const ConeCluster_t *b,
                        ConeDetectResult_t *result)
{
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    float width = sqrtf(dx * dx + dy * dy);
    float cx = (a->x + b->x) * 0.5f;
    float cy = (a->y + b->y) * 0.5f;
    float center_dist = sqrtf(cx * cx + cy * cy);
    float tx;
    float ty;
    float nx;
    float ny;

    if((width < 1.0f) || (width > GATE_WIDTH_MAX_MM))
    {
        return 0u;
    }

    tx = dx / width;
    ty = dy / width;

    nx = -ty;
    ny = tx;
    if(ny < 0.0f)
    {
        nx = -nx;
        ny = -ny;
    }

    memset(result, 0, sizeof(ConeDetectResult_t));
    result->found = 1u;
    result->gate_found = 1u;
    result->cone1_x = a->x;
    result->cone1_y = a->y;
    result->cone2_x = b->x;
    result->cone2_y = b->y;
    result->center_x = cx;
    result->center_y = cy;
    result->center_dist = center_dist;
    result->center_angle_deg = atan2f(cx, cy) * 180.0f / PI_F;
    result->gate_width = width;
    result->normal_x = nx;
    result->normal_y = ny;
    result->normal_angle_deg = atan2f(nx, ny) * 180.0f / PI_F;
    result->lateral_error_mm = -(cx * tx + cy * ty);

    return 1u;
}

static void GetGateCarSideTarget(const ConeDetectResult_t *gate,
                                 float *target_x,
                                 float *target_y)
{
    float x1;
    float y1;
    float x2;
    float y2;
    float d1;
    float d2;

    if((gate == 0) || (target_x == 0) || (target_y == 0))
    {
        return;
    }

    if(gate->gate_found == 0u)
    {
        *target_x = gate->center_x;
        *target_y = gate->center_y;
        return;
    }

    x1 = gate->center_x + gate->normal_x * GATE_TARGET_OFFSET_MM;
    y1 = gate->center_y + gate->normal_y * GATE_TARGET_OFFSET_MM;
    x2 = gate->center_x - gate->normal_x * GATE_TARGET_OFFSET_MM;
    y2 = gate->center_y - gate->normal_y * GATE_TARGET_OFFSET_MM;

    d1 = x1 * x1 + y1 * y1;
    d2 = x2 * x2 + y2 * y2;

    if(d1 < d2)
    {
        *target_x = x1;
        *target_y = y1;
    }
    else
    {
        *target_x = x2;
        *target_y = y2;
    }
}

static float GetGateCarSideTargetDist(const ConeDetectResult_t *gate)
{
    float target_x = 0.0f;
    float target_y = 0.0f;

    GetGateCarSideTarget(gate, &target_x, &target_y);
    return sqrtf(target_x * target_x + target_y * target_y);
}

static uint8_t IsGateReadyToPass(const ConeDetectResult_t *gate)
{
 if((gate == 0) || (gate->gate_found == 0u))
    {
        return 0u;
    }

    if(GetGateCarSideTargetDist(gate) > GATE_TARGET_ARRIVE_MM)
    {
        return 0u;
    }

    if(fabsf(gate->lateral_error_mm) > LINE_OK_MM)
    {
        return 0u;
    }

    if(AbsS16(FloatToS16(gate->normal_angle_deg)) > LINE_OK_DEG)
    {
        return 0u;
    }

    /*
     * 新增：门中心角也要基本在正前方。
     * 否则车可能是斜插进门。
     */
    if(AbsS16(FloatToS16(gate->center_angle_deg)) > PASS_CENTER_OK_DEG)
    {
        return 0u;
    }

    /*
     * 新增：如果左右锥桶任意一侧离车体太近，不允许进 PASS。
     */
    if(IsGateSideUnsafe(gate) != 0u)
    {
        return 0u;
    }

    return 1u;
}

static uint8_t MakeSingleTarget(const ConeCluster_t *target,
                                ConeDetectResult_t *result)
{
    if((target == 0) || (result == 0))
    {
        return 0u;
    }

    memset(result, 0, sizeof(ConeDetectResult_t));
    result->found = 1u;
    result->gate_found = 0u;
    result->cone1_x = target->x;
    result->cone1_y = target->y;
    result->cone2_x = target->x;
    result->cone2_y = target->y;
    result->center_x = target->x;
    result->center_y = target->y;
    result->center_dist = target->dist;
    result->center_angle_deg = target->angle_deg;
    result->gate_width = 0.0f;
    result->normal_x = 0.0f;
    result->normal_y = 1.0f;
    result->normal_angle_deg = 0.0f;
    result->lateral_error_mm = target->x;

    return 1u;
}

static uint8_t FindNearestObstacleGate(const ConeCluster_t clusters[MAX_CLUSTERS],
                                       uint8_t cluster_count,
                                       int8_t preferred_side,
                                       ConeDetectResult_t *result)
{
    uint8_t i;
    int nearest = -1;
    int pair = -1;
    float nearest_dist = 99999999.0f;
    float pair_dist = 99999999.0f;

    for(i = 0u; i < cluster_count; i++)
    {
        if(clusters[i].valid == 0u)
        {
            continue;
        }

        if((preferred_side > 0) && (clusters[i].x < -80.0f))
        {
            continue;
        }

        if((preferred_side < 0) && (clusters[i].x > 80.0f))
        {
            continue;
        }

        if(clusters[i].dist < nearest_dist)
        {
            nearest_dist = clusters[i].dist;
            nearest = (int)i;
        }
    }

    if((nearest < 0) && (preferred_side != 0))
    {
        preferred_side = 0;
        nearest_dist = 99999999.0f;

        for(i = 0u; i < cluster_count; i++)
        {
            if(clusters[i].valid == 0u)
            {
                continue;
            }

            if(clusters[i].dist < nearest_dist)
            {
                nearest_dist = clusters[i].dist;
                nearest = (int)i;
            }
        }
    }

    if(nearest < 0)
    {
        return 0u;
    }

    MakeSingleTarget(&clusters[nearest], result);

    for(i = 0u; i < cluster_count; i++)
    {
        float dx;
        float dy;
        float d;

        if((int)i == nearest)
        {
            continue;
        }

        if(clusters[i].valid == 0u)
        {
            continue;
        }

        dx = clusters[i].x - clusters[nearest].x;
        dy = clusters[i].y - clusters[nearest].y;
        d = sqrtf(dx * dx + dy * dy);

        if(d < pair_dist)
        {
            pair_dist = d;
            pair = (int)i;
        }
    }

    if((pair >= 0) && (pair_dist <= GATE_WIDTH_MAX_MM))
    {
        MakeGate(&clusters[nearest], &clusters[pair], result);
    }

    return 1u;
}

void ConeDetect_Init(void)
{
    memset(detect_points, 0, sizeof(detect_points));
    memset(detect_clusters, 0, sizeof(detect_clusters));
    memset(&drive_frame, 0, sizeof(drive_frame));
}

uint8_t ConeDetect_Update(const RPLIDAR360_Frame_t *frame,
                          ConeDetectResult_t *result)
{
    return ConeDetect_UpdateWithSide(frame, result, 0);
}

static uint8_t ConeDetect_UpdateWithSide(const RPLIDAR360_Frame_t *frame,
                                         ConeDetectResult_t *result,
                                         int8_t preferred_side)
{
    uint16_t point_count;
    uint8_t cluster_count = 0u;

    if((frame == 0) || (result == 0))
    {
        return 0u;
    }

    ClearResult(result);

    point_count = CollectFrontPoints(frame, detect_points);
    if(point_count < 2u)
    {
        return 0u;
    }

    if(BuildClusters(detect_points, point_count, detect_clusters, &cluster_count) == 0u)
    {
        return 0u;
    }

    if(FindNearestObstacleGate(detect_clusters, cluster_count, preferred_side, result) == 0u)
    {
        return 0u;
    }

    return 1u;
}

void ConeDetect_ShowOLED(const ConeDetectResult_t *result)
{
    char line[24];

    OLED_Clear();
    if((result == 0) || (result->found == 0u))
    {
        OLED_ShowString(0, 0, (const u8 *)"Gate: NO");
        OLED_Refresh_Gram();
        return;
    }

    if(result->gate_found != 0u)
    {
        OLED_ShowString(0, 0, (const u8 *)"Gate: YES");
    }
    else
    {
        OLED_ShowString(0, 0, (const u8 *)"Target:YES");
    }
    sprintf(line, "A:%4d D:%4d",
            (int)result->center_angle_deg,
            (int)result->center_dist);
    OLED_ShowString(0, 16, (const u8 *)line);
    sprintf(line, "L:%4d N:%4d",
            (int)result->lateral_error_mm,
            (int)result->normal_angle_deg);
    OLED_ShowString(0, 32, (const u8 *)line);
    sprintf(line, "W:%4d", (int)result->gate_width);
    OLED_ShowString(0, 48, (const u8 *)line);
    OLED_Refresh_Gram();
}

void ConeDrive_ConfigDefault(ConeDriveConfig_t *cfg)
{
    if(cfg == 0)
    {
        return;
    }

    cfg->center_start_distance_mm = 2400u;
    cfg->pass_start_distance_mm = 1600u;
    cfg->pass_time_ms = 3000u;
    cfg->clear_gate_time_ms = 500u;
    cfg->search_reacquire_time_ms = 700u;
    cfg->emergency_stop_distance_mm = 0u;

    cfg->speed_search = 38u;
    cfg->speed_turn = 38u;
    cfg->speed_approach = 38u;
    cfg->speed_centering = 38u;
    cfg->speed_pass = 45u;

    cfg->scan_steer_angle = 0;
    cfg->lost_max_count = 8u;
    cfg->lost_timeout_ms = 700u;

    cfg->kp_gate_angle_q8 = 80;
    cfg->kd_gate_angle_q8 = 35;
    cfg->kp_center_angle_q8 = 70;
    cfg->kd_center_angle_q8 = 30;
    cfg->kp_yaw_q8 = 35;
    cfg->kd_yaw_q8 = 18;

    cfg->center_ok_deg = 5;
    cfg->yaw_ok_deg = 4;
    cfg->yaw_sign = 1;
    cfg->target_yaw_deg = 0;
    cfg->max_steer_angle = 42;
    cfg->steer_trim_angle = 0;

    cfg->target_gate_count = 0u;
}

void ConeDrive_Init(ConeDriveContext_t *ctx)
{
    if(ctx == 0)
    {
        return;
    }

    memset(ctx, 0, sizeof(ConeDriveContext_t));
    ctx->state = CONE_DRIVE_STATE_SEARCH;
    ctx->last_lidar_round_seq = 0u;
}

static void SetState(ConeDriveContext_t *ctx,
                     ConeDriveState_t state,
                     uint32_t now_ms)
{
    int16_t entry_steer;

    entry_steer = ctx->last_steer_angle;

    ctx->state = state;
    ctx->state_start_ms = now_ms;
    ctx->lost_count = 0u;
    ctx->last_gate_error = 0;
    ctx->last_center_error = 0;
    ctx->last_yaw_error = 0;

    if(state == CONE_DRIVE_STATE_PASS)
    {
        ctx->last_yaw_error = entry_steer;
    }
}
static int16_t ShapeGateSteer(int16_t steer);
static int16_t CalcApproachSteer(const ConeDetectResult_t *gate,
                                 ConeDriveContext_t *ctx,
                                 const ConeDriveConfig_t *cfg)
{
    int16_t target_angle;
    int16_t normal_angle;
    int16_t aim_angle;
    int16_t steer;
    float target_x;
    float target_y;

    (void)ctx;

    GetGateCarSideTarget(gate, &target_x, &target_y);

    target_angle = FloatToS16(atan2f(target_x, target_y) * 180.0f / PI_F);
    normal_angle = FloatToS16(gate->normal_angle_deg);

    aim_angle = (int16_t)(target_angle + normal_angle / 2);

steer = DirectSteerFromAngle(aim_angle, cfg->max_steer_angle);

return steer;
}

static int16_t CalcTargetSteer(const ConeDetectResult_t *target,
                               ConeDriveContext_t *ctx,
                               const ConeDriveConfig_t *cfg)
{
   int16_t target_angle;
    int16_t steer;
    float target_x;
    float target_y;

    (void)ctx;

    GetGateCarSideTarget(target, &target_x, &target_y);
    target_angle = FloatToS16(atan2f(target_x, target_y) * 180.0f / PI_F);

   steer = DirectSteerFromAngle(target_angle, cfg->max_steer_angle);

return steer;
}

static int16_t CalcSingleTargetGuardSteer(const ConeDetectResult_t *target,
                                          const ConeDriveConfig_t *cfg)
{
    if(target->center_x >= 0.0f)
    {
        return (int16_t)(-cfg->max_steer_angle);
    }

    return cfg->max_steer_angle;
}

static int8_t GetExpectedSearchSide(const ConeDriveContext_t *ctx)
{
    (void)ctx;
    return 0;
}

static int16_t GetSearchScanSteer(const ConeDriveContext_t *ctx)
{
    (void)ctx;
    return 0;
}




static int16_t ShapeGateSteer(int16_t steer)
{
    /*
     * 当前现象：小车整体偏向左前方。
     * 处理策略：
     * 1. 左转削弱、限幅；
     * 2. 右转增强一点；
     * 3. 右转允许比左转更大。
     *
     * 假设：steer > 0 为右转，steer < 0 为左转。
     */

    if(steer > 0)
    {
        /* 右转增强 */
        steer = (int16_t)(steer + 6);

        /* 右转最大允许 34 度 */
        steer = ClampS16(steer, 0, 34);
    }
    else if(steer < 0)
    {
        /* 左转削弱 */
        steer = (int16_t)(steer / 2);

        /* 左转最大只允许 -16 度 */
        steer = ClampS16(steer, -16, 0);
    }

    return steer;
}

static int16_t SuppressLeftSteerInApproach(int16_t steer)
{
    /*
     * APPROACH 阶段左拐抑制。
     *
     * 假设 steer < 0 是左拐，steer > 0 是右拐。
     * 如果实车方向相反，把下面的 steer < 0 改成 steer > 0。
     */

    if(steer < 0)
    {
        /*
         * 先不要直接压成 0，先保留一点左转能力，避免完全不能修正。
         * 原左转减半，并限制最大左转幅度。
         */
        steer = (int16_t)(steer / 3);
        steer = ClampS16(steer, -10, 0);
    }

    return steer;
}


static int16_t NoLeftSteerInGate(int16_t steer)
{
    /*
     * 临时测试：
     * 在 APPROACH / PASS 阶段禁止左拐。
     *
     * 假设 steer < 0 是左拐。
     * 如果实车方向相反，改成 if(steer > 0) steer = 0;
     */
    if(steer < 0)
    {
        steer = 0;
    }

    return steer;
}
void ConeDrive_Update(ConeDriveContext_t *ctx,
                      const ConeDriveConfig_t *cfg,
                      uint32_t now_ms)
{
    ConeDetectResult_t gate;
    uint8_t found = 0u;
    uint8_t lidar_usable = 0u;
    uint8_t lidar_current = 0u;
    int16_t steer = 0;
    int8_t expected_side;

    if((ctx == 0) || (cfg == 0))
    {
        return;
    }

    memset(&gate, 0, sizeof(gate));
    expected_side = GetExpectedSearchSide(ctx);

    lidar_current = APP_RPLIDAR_CopyCurrentFrame(&drive_frame);
    if((lidar_current != 0u) && (IsFrontEmergencyStop(&drive_frame, cfg) != 0u))
    {
        SetSpeed(cfg->speed_search);
        steer = GetSearchScanSteer(ctx);
        SetSteerRaw(steer, cfg);
        ctx->last_steer_angle = steer;
        if(ctx->state != CONE_DRIVE_STATE_DONE)
        {
            SetState(ctx, CONE_DRIVE_STATE_SEARCH, now_ms);
        }
        return;
    }

    if(APP_RPLIDAR_CopyUsableFrame(&drive_frame) != 0u)
    {
        lidar_usable = 1u;
        found = ConeDetect_UpdateWithSide(&drive_frame, &gate, expected_side);
    }
    else if(lidar_current != 0u)
    {
        found = ConeDetect_UpdateWithSide(&drive_frame, &gate, expected_side);
    }


    if(found != 0u)
    {
        ctx->gate = gate;
        ctx->lost_count = 0u;
        ctx->last_seen_ms = now_ms;
        ctx->last_lidar_round_seq = drive_frame.round_seq;
    }
    else if((lidar_current != 0u) &&
            (drive_frame.round_seq != ctx->last_lidar_round_seq))
    {
        ctx->last_lidar_round_seq = drive_frame.round_seq;
        if(ctx->lost_count < 255u)
        {
            ctx->lost_count++;
        }
    }

    switch(ctx->state)
    {
    case CONE_DRIVE_STATE_SEARCH:
        if(lidar_usable != 0u)
        {
            SetSpeed(cfg->speed_search);
        }
        else if((ctx->passed_gate_count > 0u) &&
                ((uint32_t)(now_ms - ctx->state_start_ms) < cfg->search_reacquire_time_ms))
        {
            SetSpeed(cfg->speed_search);
        }
        else
        {
            SetSpeed(cfg->speed_search);
        }

        if(found != 0u)
        {
            if((gate.gate_found == 0u) &&
               (gate.center_dist < SINGLE_TARGET_GUARD_MM))
            {
                steer = CalcSingleTargetGuardSteer(&gate, cfg);
            }
            else
            {
                steer = CalcTargetSteer(&gate, ctx, cfg);
            }

            SetSteerRaw(steer, cfg);
            ctx->last_steer_angle = steer;

            if(gate.gate_found != 0u)
            {
                SetState(ctx, CONE_DRIVE_STATE_APPROACH, now_ms);
            }
        }
        else
        {
            SetSteerRaw(GetSearchScanSteer(ctx), cfg);
        }
        break;

case CONE_DRIVE_STATE_APPROACH:
{
    /*
     * 情况 1：当前没有完整门
     * found == 0：完全丢目标
     * gate_found == 0：只看到单锥桶/非完整门
     */
    if((found == 0u) || (gate.gate_found == 0u))
    {
        if(found != 0u)
        {
            /*
             * 当前只看到单目标。
             * 如果上一帧保存的门已经满足通过条件，则进入 PASS。
             */
            if(IsGateReadyToPass(&ctx->gate) != 0u)
            {
                steer = 0;
                SetState(ctx, CONE_DRIVE_STATE_PASS, now_ms);
            }
            else if(gate.center_dist < SINGLE_TARGET_GUARD_MM)
            {
                steer = CalcSingleTargetGuardSteer(&gate, cfg);
            }
            else
            {
                steer = CalcTargetSteer(&gate, ctx, cfg);
            }

            SetSpeed(cfg->speed_search);

            /*
             * APPROACH 阶段统一抑制左拐。
             */
            steer = SuppressLeftSteerInApproach(steer);

            SetSteerRaw(steer, cfg);
            ctx->last_steer_angle = steer;
        }
        else if((uint32_t)(now_ms - ctx->last_seen_ms) >= cfg->lost_timeout_ms)
        {
            /*
             * 丢门超过超时时间。
             */
            if(IsGateReadyToPass(&ctx->gate) != 0u)
            {
                SetState(ctx, CONE_DRIVE_STATE_PASS, now_ms);
            }
            else
            {
                steer = GetSearchScanSteer(ctx);
                SetSpeed(cfg->speed_search);

                steer = SuppressLeftSteerInApproach(steer);

                SetSteerRaw(steer, cfg);
                ctx->last_steer_angle = steer;

                SetState(ctx, CONE_DRIVE_STATE_SEARCH, now_ms);
            }
        }
        else
        {
            /*
             * 短时间丢门，保持上一帧舵角。
             */
            steer = ctx->last_steer_angle;
            SetSpeed(cfg->speed_approach);

            steer = SuppressLeftSteerInApproach(steer);

            SetSteerRaw(steer, cfg);
            ctx->last_steer_angle = steer;
        }

        break;
    }

    /*
     * 情况 2：当前识别到完整门
     */
    if(IsGateSideUnsafe(&gate) != 0u)
    {
        /*
         * 近距离锥桶危险，优先使用安全居中修正。
         */
        steer = CalcCenteringSteerSafe(&gate, ctx, cfg);
        SetSpeed(cfg->speed_centering);
    }
    else if(GetGateCarSideTargetDist(&gate) > GATE_TARGET_ARRIVE_MM)
    {
        /*
         * 还没到门前目标点，继续 APPROACH。
         */
        steer = CalcApproachSteer(&gate, ctx, cfg);
        SetSpeed(cfg->speed_approach);
    }
    else if(IsGateReadyToPass(&gate) != 0u)
    {
        /*
         * 满足通过条件，进入 PASS。
         */
        steer = 0;
        SetSpeed(cfg->speed_centering);
        SetState(ctx, CONE_DRIVE_STATE_PASS, now_ms);
    }
    else
    {
        /*
         * 已经接近门，但还没满足 PASS 条件，继续微调居中。
         */
        steer = CalcCenteringSteerSafe(&gate, ctx, cfg);
        SetSpeed(cfg->speed_centering);
    }

    /*
     * APPROACH 阶段统一抑制左拐。
     */
    steer = SuppressLeftSteerInApproach(steer);

    SetSteerRaw(steer, cfg);
    ctx->last_steer_angle = steer;
}
break;

   

    case CONE_DRIVE_STATE_PASS:
{
    uint32_t pass_elapsed;

    pass_elapsed = now_ms - ctx->state_start_ms;

    SetSpeed(cfg->speed_pass);

    /*
     * PASS 前半段：允许根据当前门修正姿态
     * PASS 后半段：不再追旧门，防止过门后继续右打/左打
     */
    if((pass_elapsed < 700u) &&
       (found != 0u) &&
       (gate.gate_found != 0u))
    {
        steer = CalcLineSteer(&gate, ctx, cfg);
    }
    else
    {
        steer = 0;
    }

steer = NoLeftSteerInGate(steer);

SetSteerRaw(steer, cfg);
ctx->last_steer_angle = steer;

    if(pass_elapsed >= cfg->pass_time_ms)
    {
        if(ctx->passed_gate_count < 255u)
        {
            ctx->passed_gate_count++;
        }

        if((cfg->target_gate_count > 0u) &&
           (ctx->passed_gate_count >= cfg->target_gate_count))
        {
            SetState(ctx, CONE_DRIVE_STATE_DONE, now_ms);
        }
        else
        {
            if(cfg->clear_gate_time_ms > 0u)
            {
                SetState(ctx, CONE_DRIVE_STATE_CLEAR_GATE, now_ms);
            }
            else
            {
                SetState(ctx, CONE_DRIVE_STATE_SEARCH, now_ms);
            }
        }
    }
}
        break;

    case CONE_DRIVE_STATE_CLEAR_GATE:
        if((uint32_t)(now_ms - ctx->state_start_ms) >= cfg->clear_gate_time_ms)
        {
            SetSpeed(cfg->speed_search);
            SetState(ctx, CONE_DRIVE_STATE_SEARCH, now_ms);
        }
        else
        {
            steer = 0;
            SetSpeed(cfg->speed_search);
            SetSteerRaw(steer, cfg);
            ctx->last_steer_angle = steer;
        }
        break;

    case CONE_DRIVE_STATE_DONE:
        SetSpeed(cfg->speed_search);
        steer = GetSearchScanSteer(ctx);
        SetSteerRaw(steer, cfg);
        SetState(ctx, CONE_DRIVE_STATE_SEARCH, now_ms);
        break;

    default:
        SetState(ctx, CONE_DRIVE_STATE_SEARCH, now_ms);
        break;
    }
}
