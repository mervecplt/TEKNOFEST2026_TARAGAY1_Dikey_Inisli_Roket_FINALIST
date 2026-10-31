#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Modules/Control/TaragayFlightLogic/taragay_control_tuning.h"
#include "Services/SolenoidOutput/solenoid_output.h"

#include "Common/app_config.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Services/NeedleValveIntegrationTest/needle_valve_integration_test.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "Services/Timebase/timebase.h"

#include <math.h>
#include <string.h>

#define TFL_TS_S                 TGY_CTRL_TS_S
#define TFL_G                    TGY_CTRL_G
#define TFL_RAD2DEG              57.29577951308232f
#define TFL_DEG2RAD              0.017453292519943295f
#define TFL_Z_RISE_DELTA_M       3.00f
#define TFL_Z_CEILING_DELTA_M    4.00f
#define TFL_Z_CEILING_HYST_M     0.20f
#define TFL_Z_TOUCH_M            0.4013f

/* R14/P110 fast physical needle feedback bridge.
 * Read-only, lock-free: both sources are volatile 16-bit diagnostics already
 * maintained by the active P83/P110 actuator path.  No telemetry/status getter
 * and no IRQ masking are used here, so actuator timing is untouched. */
static float tfl_needle_turns_actual_fast(void)
{
    uint16_t zero_adc = p111_baseline_adc;
    uint16_t current_adc = p83_filtered_adc;
    uint32_t travel_adc;

    if (zero_adc < NEEDLE_VALVE_SAFE_ZERO_MIN_ADC)
    {
        return 0.0f;
    }

    travel_adc = (current_adc < zero_adc) ?
        (uint32_t)(zero_adc - current_adc) : 0UL;

    if (travel_adc > (uint32_t)APP_R8R32_NEEDLE_MAX_TRAVEL_ADC)
    {
        travel_adc = (uint32_t)APP_R8R32_NEEDLE_MAX_TRAVEL_ADC;
    }

    return (float)travel_adc / (float)APP_R8R32_NEEDLE_ADC_PER_TURN;
}

/* -------------------------------------------------------------------------- */
/* R14p3b/R14p4 adaptive vertical state                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t initialized;
    uint8_t state;
    uint8_t release_latch;
    uint8_t safe_latch;
    uint8_t authority_latch;
    uint8_t identify_active;
    uint8_t identify_complete;
    uint8_t id_airborne_latch;
    uint8_t handover_active;
    uint8_t v_initialized;
    uint32_t init_count;
    uint32_t health_count;
    uint32_t preposition_count;
    uint32_t hover_count;
    uint32_t hover_best;
    uint32_t authority_count;
    uint32_t id_timeout_count;
    uint32_t id_step_wait_count;
    uint32_t id_airborne_count;
    uint32_t id_settle_count;
    uint32_t id_sample_count;
    float z0_ref;
    float z_target;
    float v_prev;
    float a_filt;
    float iz;
    float b_hat;
    float b_auth;
    float id_probe_u;
    float id_b_sum;
    float b_id_est;
    float handover_u;
} HoverRuntime_t;

typedef struct
{
    float valve_cmd;          /* normalized flow/effectiveness command 0..1 */
    float actuator_cmd;       /* physical normalized 0..1 travel command */
    float turns_target;
    float u_actual;
    float z_reference_m;
    uint8_t state;
    float hover_best_s;
    float target_force_n;     /* legacy diagnostic only; controller is mass independent */
    uint8_t thrust_shortage;
    float a_filt;
    float a_cmd;
    float b_hat;
    float b_auth;
    float authority_ratio;
    float u_hover_est;
    float d_stop_pred;
    float v_ref;
} HoverOutput_t;

/* -------------------------------------------------------------------------- */
/* RCS V7.13.4 state                                                          */
/* -------------------------------------------------------------------------- */

typedef struct
{
    float prev_angle;
    float filtered_rate;
    float filtered_trend;
    uint8_t rate_initialized;
    uint8_t trend_initialized;

    uint8_t mode;
    uint8_t phase;
    int8_t desired;
    int8_t applied;
    int8_t pending;

    uint8_t event_active;
    uint8_t event_armed;
    uint8_t hard_latched;
    uint8_t brake_used;
    int8_t hazard_sign;

    uint8_t dead_active;
    uint16_t dead_age;
    uint16_t off_age;
    uint16_t on_age;
    uint16_t event_age;
    uint16_t event_end_age;
    uint16_t safe_age;
    uint16_t brake_age;
    uint16_t window_age;
    uint8_t reversal_count;
    uint16_t event_window_age;
    uint8_t event_count;
    uint8_t pred_count;

    float target;
    float stopping;
    float braking_distance;
    float switch_error;

    uint8_t track_active;
    int8_t track_sign;
    uint16_t track_age;
    uint16_t track_pulse_n;
    uint16_t track_cooldown;
    float track_filtered_rate;
    uint8_t track_rate_initialized;
    uint8_t track_confirm_count;
    int8_t track_confirm_sign;
} RCSAxis_t;

typedef struct
{
    uint8_t initialized;
    RCSAxis_t pitch;
    RCSAxis_t yaw;
    uint32_t sample_count;
    uint8_t hard_fault;
    uint8_t fault_code;
    uint8_t landed_latch;
} RCSRuntime_t;

typedef struct
{
    uint8_t v1;
    uint8_t v3;
    uint8_t v5;
    uint8_t v7;
    int8_t upitch;
    int8_t uyaw;
    uint8_t mode_pitch;
    uint8_t mode_yaw;
    uint8_t fault;
    float pred_pitch_deg;
    float pred_yaw_deg;
    float time_to_ground_s;
} RCSOutput_t;

static HoverRuntime_t s_hover;
static RCSRuntime_t s_rcs;
static TaragayFlightLogicStatus_t s_status;
static uint8_t s_divider_200hz;
static uint32_t s_synthetic_step;
static uint8_t s_prev_rcs_request_mask;

/* R4R2 relative-altitude mission reference. The first valid 100 Hz control
 * sample after PE9 flight-active reset becomes the separation height. */
static uint8_t s_separation_z_valid;
static float s_separation_z_cg_m;
static uint8_t s_main_ceiling_active;

/* -------------------------------------------------------------------------- */
/* R8R30 fixed IMU -> rocket transform; R8R29 solver retained dormant       */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t phase;       /* 0 wait source, 1 upright capture,
                          * 2 wait +X, 3 hold +X,
                          * 4 wait -X, 5 hold -X,
                          * 6 wait +Y, 7 hold +Y,
                          * 8 wait -Y, 9 hold -Y,
                          * 10 calibrated, 11 fault */
    uint8_t valid;
    uint8_t fault;
    uint16_t upright_count;
    uint16_t tilt_count;
    uint16_t neg_x_count;
    uint16_t y_tilt_count;
    uint16_t neg_y_count;
    float upright_sum[3];
    float tilt_sum[3];
    float neg_x_sum[3];
    float y_tilt_sum[3];
    float neg_y_sum[3];
    float z_axis_i[3];
    float x_plus_axis_i[3];
    float x_minus_axis_i[3];
    float x_axis_meas_i[3];
    float y_plus_axis_i[3];
    float y_minus_axis_i[3];
    float r_rocket_from_imu[3][3];
    float tilt_deg;
    float neg_x_tilt_deg;
    float y_tilt_deg;
    float neg_y_tilt_deg;
    float x_opposition;
    float y_opposition;
    float xy_angle_deg;
    float axis_agreement;
    float z_agreement;
    float ortho_error;
    float det;
} MountCalRuntime_t;

static MountCalRuntime_t s_mount_cal;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static float tfl_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float tfl_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static int8_t tfl_sign8(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

static uint16_t tfl_sat_inc16(uint16_t x)
{
    return (x < 65535U) ? (uint16_t)(x + 1U) : x;
}

static uint8_t tfl_finite(float x)
{
    return (isfinite(x) != 0) ? 1U : 0U;
}

static uint16_t tfl_ceil_samples(float seconds)
{
    float n = ceilf(seconds / TFL_TS_S);
    if (n < 0.0f) n = 0.0f;
    if (n > 65535.0f) n = 65535.0f;
    return (uint16_t)n;
}

static float tfl_vec3_dot(const float a[3], const float b[3])
{
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static float tfl_vec3_norm(const float a[3])
{
    return sqrtf(tfl_vec3_dot(a, a));
}

static uint8_t tfl_vec3_normalize(float a[3])
{
    float n = tfl_vec3_norm(a);
    if ((tfl_finite(n) == 0U) || (n < 1.0e-6f)) return 0U;
    a[0] /= n; a[1] /= n; a[2] /= n;
    return 1U;
}

static void tfl_vec3_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static void mount_cal_publish(void)
{
    s_status.mount_cal_phase = s_mount_cal.phase;
    s_status.mount_cal_valid = s_mount_cal.valid;
    s_status.mount_cal_fault = s_mount_cal.fault;
    s_status.mount_cal_upright_samples = s_mount_cal.upright_count;
    s_status.mount_cal_tilt_samples = s_mount_cal.tilt_count;
    s_status.mount_cal_y_tilt_samples = s_mount_cal.y_tilt_count;
    s_status.mount_cal_neg_x_samples = s_mount_cal.neg_x_count;
    s_status.mount_cal_neg_y_samples = s_mount_cal.neg_y_count;
    s_status.mount_cal_tilt_deg = s_mount_cal.tilt_deg;
    s_status.mount_cal_y_tilt_deg = s_mount_cal.y_tilt_deg;
    s_status.mount_cal_neg_x_tilt_deg = s_mount_cal.neg_x_tilt_deg;
    s_status.mount_cal_neg_y_tilt_deg = s_mount_cal.neg_y_tilt_deg;
    s_status.mount_cal_x_opposition = s_mount_cal.x_opposition;
    s_status.mount_cal_y_opposition = s_mount_cal.y_opposition;
    s_status.mount_cal_xy_angle_deg = s_mount_cal.xy_angle_deg;
    s_status.mount_cal_axis_agreement = s_mount_cal.axis_agreement;
    s_status.mount_cal_z_agreement = s_mount_cal.z_agreement;
    s_status.mount_cal_ortho_error = s_mount_cal.ortho_error;
    s_status.mount_cal_det = s_mount_cal.det;
    s_status.mount_r00 = s_mount_cal.r_rocket_from_imu[0][0];
    s_status.mount_r01 = s_mount_cal.r_rocket_from_imu[0][1];
    s_status.mount_r02 = s_mount_cal.r_rocket_from_imu[0][2];
    s_status.mount_r10 = s_mount_cal.r_rocket_from_imu[1][0];
    s_status.mount_r11 = s_mount_cal.r_rocket_from_imu[1][1];
    s_status.mount_r12 = s_mount_cal.r_rocket_from_imu[1][2];
    s_status.mount_r20 = s_mount_cal.r_rocket_from_imu[2][0];
    s_status.mount_r21 = s_mount_cal.r_rocket_from_imu[2][1];
    s_status.mount_r22 = s_mount_cal.r_rocket_from_imu[2][2];
}

static void mount_cal_reset(void)
{
    memset(&s_mount_cal, 0, sizeof(s_mount_cal));
#if (APP_P112R12R8R30_FIXED_IMU_ROCKET_FRAME_REV != 0U)
    /* R8R30 production-path integration: load the physically calibrated
     * constant transform immediately. No pose capture is performed at boot.
     * Legacy quality fields are frozen provenance from the accepted R8R29 log,
     * not live measurements. */
    s_mount_cal.phase = APP_R8R30_FIXED_CAL_PHASE;
    s_mount_cal.valid = 1U;
    s_mount_cal.fault = 0U;
    s_mount_cal.tilt_deg = APP_R8R30_PROV_POS_X_TILT_DEG;
    s_mount_cal.neg_x_tilt_deg = APP_R8R30_PROV_NEG_X_TILT_DEG;
    s_mount_cal.y_tilt_deg = APP_R8R30_PROV_POS_Y_TILT_DEG;
    s_mount_cal.neg_y_tilt_deg = APP_R8R30_PROV_NEG_Y_TILT_DEG;
    s_mount_cal.x_opposition = APP_R8R30_PROV_X_OPPOSITION;
    s_mount_cal.y_opposition = APP_R8R30_PROV_Y_OPPOSITION;
    s_mount_cal.xy_angle_deg = APP_R8R30_PROV_XY_ANGLE_DEG;
    s_mount_cal.axis_agreement = APP_R8R30_PROV_AXIS_AGREEMENT;
    s_mount_cal.z_agreement = APP_R8R30_PROV_Z_AGREEMENT;
    s_mount_cal.ortho_error = APP_R8R30_PROV_ORTHO_ERROR;
    s_mount_cal.det = APP_R8R30_PROV_DET;
    s_mount_cal.r_rocket_from_imu[0][0] = APP_R8R30_R00;
    s_mount_cal.r_rocket_from_imu[0][1] = APP_R8R30_R01;
    s_mount_cal.r_rocket_from_imu[0][2] = APP_R8R30_R02;
    s_mount_cal.r_rocket_from_imu[1][0] = APP_R8R30_R10;
    s_mount_cal.r_rocket_from_imu[1][1] = APP_R8R30_R11;
    s_mount_cal.r_rocket_from_imu[1][2] = APP_R8R30_R12;
    s_mount_cal.r_rocket_from_imu[2][0] = APP_R8R30_R20;
    s_mount_cal.r_rocket_from_imu[2][1] = APP_R8R30_R21;
    s_mount_cal.r_rocket_from_imu[2][2] = APP_R8R30_R22;
#endif
    mount_cal_publish();
}

static uint8_t mount_cal_is_still(const SensorData_t *sensor,
                                  const FullStateESKFData_t *eskf)
{
    float gx, gy, gz;
    if ((sensor == 0) || (eskf == 0)) return 0U;
    if ((sensor->accel_filtered_norm_g < APP_R8R26_CAL_ACCEL_NORM_MIN_G) ||
        (sensor->accel_filtered_norm_g > APP_R8R26_CAL_ACCEL_NORM_MAX_G)) return 0U;
    gx = sensor->gyro_x_filtered_dps - eskf->gyro_bias_x_dps;
    gy = sensor->gyro_y_filtered_dps - eskf->gyro_bias_y_dps;
    gz = sensor->gyro_z_filtered_dps - eskf->gyro_bias_z_dps;
    if ((tfl_absf(gx) > APP_R8R26_CAL_GYRO_STILL_DPS) ||
        (tfl_absf(gy) > APP_R8R26_CAL_GYRO_STILL_DPS) ||
        (tfl_absf(gz) > APP_R8R26_CAL_GYRO_STILL_DPS)) return 0U;
    return 1U;
}

static void mount_cal_clear_sum(float sum[3])
{
    sum[0] = 0.0f; sum[1] = 0.0f; sum[2] = 0.0f;
}

static void mount_cal_accumulate(float sum[3], const SensorData_t *sensor)
{
    sum[0] += sensor->accel_x_filtered_g;
    sum[1] += sensor->accel_y_filtered_g;
    sum[2] += sensor->accel_z_filtered_g;
}

static uint8_t mount_cal_mean_unit(const float sum[3], uint16_t count, float out[3])
{
    if ((sum == 0) || (out == 0) || (count == 0U)) return 0U;
    out[0] = sum[0] / (float)count;
    out[1] = sum[1] / (float)count;
    out[2] = sum[2] / (float)count;
    return tfl_vec3_normalize(out);
}

/* Convert a static accelerometer UP vector from a tilted pose into the
 * physical horizontal axis toward which the rocket TOP was tilted. */
static uint8_t mount_cal_axis_from_tilt(const float up_i[3],
                                       const float z_i[3],
                                       float axis_i[3],
                                       float *tilt_deg)
{
    float a[3], proj[3], d, pn;
    if ((up_i == 0) || (z_i == 0) || (axis_i == 0) || (tilt_deg == 0)) return 0U;
    a[0] = up_i[0]; a[1] = up_i[1]; a[2] = up_i[2];
    if (tfl_vec3_normalize(a) == 0U) return 0U;
    d = tfl_clampf(tfl_vec3_dot(z_i, a), -1.0f, 1.0f);
    *tilt_deg = acosf(d) * TFL_RAD2DEG;
    proj[0] = a[0] - (d * z_i[0]);
    proj[1] = a[1] - (d * z_i[1]);
    proj[2] = a[2] - (d * z_i[2]);
    pn = tfl_vec3_norm(proj);
    if ((tfl_finite(pn) == 0U) || (pn < 0.10f)) return 0U;
    axis_i[0] = -proj[0] / pn;
    axis_i[1] = -proj[1] / pn;
    axis_i[2] = -proj[2] / pn;
    return tfl_vec3_normalize(axis_i);
}

static float mount_cal_det3(const float r[3][3])
{
    return
        r[0][0] * ((r[1][1] * r[2][2]) - (r[1][2] * r[2][1])) -
        r[0][1] * ((r[1][0] * r[2][2]) - (r[1][2] * r[2][0])) +
        r[0][2] * ((r[1][0] * r[2][1]) - (r[1][1] * r[2][0]));
}

static float mount_cal_ortho_error(const float r[3][3])
{
    float e = 0.0f, v;
    uint32_t i, j;
    for (i = 0U; i < 3U; i++)
    {
        v = tfl_absf(tfl_vec3_dot(r[i], r[i]) - 1.0f);
        if (v > e) e = v;
        for (j = i + 1U; j < 3U; j++)
        {
            v = tfl_absf(tfl_vec3_dot(r[i], r[j]));
            if (v > e) e = v;
        }
    }
    return e;
}

static uint8_t mount_cal_opposite_pose_acceptable(const float up_i[3],
                                                   const float reference_axis[3],
                                                   float *tilt_deg,
                                                   float *opposition)
{
    float a[3], up_plus[3], sep_deg, denom, d;
    (void)reference_axis; /* R8R29 no longer uses upright-projected axis sign. */
    if ((up_i == 0) || (tilt_deg == 0) || (opposition == 0)) return 0U;
    a[0] = up_i[0]; a[1] = up_i[1]; a[2] = up_i[2];
    if ((tfl_vec3_normalize(a) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.tilt_sum, s_mount_cal.tilt_count, up_plus) == 0U)) return 0U;

    d = tfl_clampf(tfl_vec3_dot(s_mount_cal.z_axis_i, a), -1.0f, 1.0f);
    *tilt_deg = acosf(d) * TFL_RAD2DEG;
    if ((*tilt_deg < APP_R8R26_CAL_TILT_MIN_DEG) ||
        (*tilt_deg > APP_R8R26_CAL_TILT_MAX_DEG)) return 0U;

    d = tfl_clampf(tfl_vec3_dot(up_plus, a), -1.0f, 1.0f);
    sep_deg = acosf(d) * TFL_RAD2DEG;
    denom = s_mount_cal.tilt_deg + *tilt_deg;
    if ((sep_deg < APP_R8R29_CAL_OPPOSITE_SEPARATION_MIN_DEG) || (denom < 1.0f)) return 0U;
    *opposition = tfl_clampf(sep_deg / denom, 0.0f, 1.0f);
    return (*opposition >= APP_R8R28_CAL_PAIR_OPPOSITION_MIN) ? 1U : 0U;
}

/* +X/-X pair gives physical +Y directly from its plane normal. Use that
 * exact pair-derived axis to identify +Y without depending on manual upright. */
static uint8_t mount_cal_plus_y_pose_acceptable(const float up_i[3],
                                                float *tilt_deg,
                                                float *xy_angle_deg,
                                                float *axis_agreement)
{
    float a[3], up_px[3], up_mx[3], y_from_x_pair[3], sy;
    if ((up_i == 0) || (tilt_deg == 0) || (xy_angle_deg == 0) || (axis_agreement == 0)) return 0U;
    a[0] = up_i[0]; a[1] = up_i[1]; a[2] = up_i[2];
    if ((tfl_vec3_normalize(a) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.tilt_sum, s_mount_cal.tilt_count, up_px) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.neg_x_sum, s_mount_cal.neg_x_count, up_mx) == 0U)) return 0U;
    tfl_vec3_cross(up_px, up_mx, y_from_x_pair);
    if (tfl_vec3_normalize(y_from_x_pair) == 0U) return 0U;

    sy = tfl_clampf(-tfl_vec3_dot(a, y_from_x_pair), -1.0f, 1.0f);
    if (sy <= 0.0f) return 0U;
    *tilt_deg = asinf(sy) * TFL_RAD2DEG;
    if ((*tilt_deg < APP_R8R26_CAL_TILT_MIN_DEG) ||
        (*tilt_deg > APP_R8R26_CAL_TILT_MAX_DEG)) return 0U;

    *xy_angle_deg = 90.0f;
    *axis_agreement = 1.0f;
    return 1U;
}

static uint8_t mount_cal_minus_y_pose_acceptable(const float up_i[3],
                                                 float *tilt_deg,
                                                 float *opposition)
{
    float a[3], up_px[3], up_mx[3], up_py[3], y_from_x_pair[3];
    float sy, sep_deg, denom, d;
    if ((up_i == 0) || (tilt_deg == 0) || (opposition == 0)) return 0U;
    a[0] = up_i[0]; a[1] = up_i[1]; a[2] = up_i[2];
    if ((tfl_vec3_normalize(a) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.tilt_sum, s_mount_cal.tilt_count, up_px) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.neg_x_sum, s_mount_cal.neg_x_count, up_mx) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.y_tilt_sum, s_mount_cal.y_tilt_count, up_py) == 0U)) return 0U;
    tfl_vec3_cross(up_px, up_mx, y_from_x_pair);
    if (tfl_vec3_normalize(y_from_x_pair) == 0U) return 0U;

    sy = tfl_clampf(tfl_vec3_dot(a, y_from_x_pair), -1.0f, 1.0f);
    if (sy <= 0.0f) return 0U;
    *tilt_deg = asinf(sy) * TFL_RAD2DEG;
    if ((*tilt_deg < APP_R8R26_CAL_TILT_MIN_DEG) ||
        (*tilt_deg > APP_R8R26_CAL_TILT_MAX_DEG)) return 0U;

    d = tfl_clampf(tfl_vec3_dot(up_py, a), -1.0f, 1.0f);
    sep_deg = acosf(d) * TFL_RAD2DEG;
    denom = s_mount_cal.y_tilt_deg + *tilt_deg;
    if ((sep_deg < APP_R8R29_CAL_OPPOSITE_SEPARATION_MIN_DEG) || (denom < 1.0f)) return 0U;
    *opposition = tfl_clampf(sep_deg / denom, 0.0f, 1.0f);
    return (*opposition >= APP_R8R28_CAL_PAIR_OPPOSITION_MIN) ? 1U : 0U;
}

/* Diagnostic +X/-X difference axis. R8R29 final matrix does NOT use this
 * upright projection; final +X/+Y/+Z come from pair plane normals. */
static uint8_t mount_cal_pair_axis(const float plus_sum[3], uint16_t plus_count,
                                   const float minus_sum[3], uint16_t minus_count,
                                   float pair_axis[3], float *opposition)
{
    float up_plus[3], up_minus[3], sep_deg, plus_deg, minus_deg, denom, d;
    if ((pair_axis == 0) || (opposition == 0)) return 0U;
    if ((mount_cal_mean_unit(plus_sum, plus_count, up_plus) == 0U) ||
        (mount_cal_mean_unit(minus_sum, minus_count, up_minus) == 0U)) return 0U;

    pair_axis[0] = up_minus[0] - up_plus[0];
    pair_axis[1] = up_minus[1] - up_plus[1];
    pair_axis[2] = up_minus[2] - up_plus[2];
    if (tfl_vec3_normalize(pair_axis) == 0U) return 0U;

    d = tfl_clampf(tfl_vec3_dot(up_plus, up_minus), -1.0f, 1.0f);
    sep_deg = acosf(d) * TFL_RAD2DEG;
    d = tfl_clampf(tfl_vec3_dot(s_mount_cal.z_axis_i, up_plus), -1.0f, 1.0f);
    plus_deg = acosf(d) * TFL_RAD2DEG;
    d = tfl_clampf(tfl_vec3_dot(s_mount_cal.z_axis_i, up_minus), -1.0f, 1.0f);
    minus_deg = acosf(d) * TFL_RAD2DEG;
    denom = plus_deg + minus_deg;
    if ((sep_deg < APP_R8R29_CAL_OPPOSITE_SEPARATION_MIN_DEG) || (denom < 1.0f)) return 0U;
    *opposition = tfl_clampf(sep_deg / denom, 0.0f, 1.0f);
    return (*opposition >= APP_R8R28_CAL_PAIR_OPPOSITION_MIN) ? 1U : 0U;
}

static uint8_t mount_cal_build_matrix_five_pose(void)
{
    float up_px[3], up_mx[3], up_py[3], up_my[3];
    float x_diff[3], y_diff[3];
    float x_from_y_pair[3], y_from_x_pair[3], z_from_pairs[3];
    float y_expected[3], x_from_y[3], x_blend[3], x_final[3], y_final[3];
    float dxy, dz;

    if ((s_mount_cal.upright_count == 0U) ||
        (s_mount_cal.tilt_count == 0U) ||
        (s_mount_cal.neg_x_count == 0U) ||
        (s_mount_cal.y_tilt_count == 0U) ||
        (s_mount_cal.neg_y_count == 0U)) return 0U;

    if ((mount_cal_mean_unit(s_mount_cal.tilt_sum, s_mount_cal.tilt_count, up_px) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.neg_x_sum, s_mount_cal.neg_x_count, up_mx) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.y_tilt_sum, s_mount_cal.y_tilt_count, up_py) == 0U) ||
        (mount_cal_mean_unit(s_mount_cal.neg_y_sum, s_mount_cal.neg_y_count, up_my) == 0U)) return 0U;

    /* R8R29 core idea:
     *   +X/-X static UP vectors span the physical X-Z plane. Their cross
     *   product therefore gives +Y independent of the exact +/- tilt angles.
     *   +Y/-Y static UP vectors span the physical Y-Z plane. Their cross
     *   product gives -X; reversing it gives +X.
     *
     * This removes the R8R28 dependence on the manually captured upright Z
     * from the solved matrix. Upright is retained only as a sign/quality check.
     */
    x_diff[0] = up_mx[0] - up_px[0];
    x_diff[1] = up_mx[1] - up_px[1];
    x_diff[2] = up_mx[2] - up_px[2];
    y_diff[0] = up_my[0] - up_py[0];
    y_diff[1] = up_my[1] - up_py[1];
    y_diff[2] = up_my[2] - up_py[2];
    if ((tfl_vec3_normalize(x_diff) == 0U) || (tfl_vec3_normalize(y_diff) == 0U)) return 0U;

    tfl_vec3_cross(up_py, up_my, x_from_y_pair);
    if (tfl_vec3_normalize(x_from_y_pair) == 0U) return 0U;
    x_from_y_pair[0] = -x_from_y_pair[0];
    x_from_y_pair[1] = -x_from_y_pair[1];
    x_from_y_pair[2] = -x_from_y_pair[2];
    if (tfl_vec3_dot(x_from_y_pair, x_diff) < 0.0f)
    {
        x_from_y_pair[0] = -x_from_y_pair[0];
        x_from_y_pair[1] = -x_from_y_pair[1];
        x_from_y_pair[2] = -x_from_y_pair[2];
    }

    tfl_vec3_cross(up_px, up_mx, y_from_x_pair);
    if (tfl_vec3_normalize(y_from_x_pair) == 0U) return 0U;
    if (tfl_vec3_dot(y_from_x_pair, y_diff) < 0.0f)
    {
        y_from_x_pair[0] = -y_from_x_pair[0];
        y_from_x_pair[1] = -y_from_x_pair[1];
        y_from_x_pair[2] = -y_from_x_pair[2];
    }

    dxy = tfl_clampf(tfl_vec3_dot(x_from_y_pair, y_from_x_pair), -1.0f, 1.0f);
    s_mount_cal.xy_angle_deg = acosf(dxy) * TFL_RAD2DEG;
    if ((s_mount_cal.xy_angle_deg < APP_R8R28_CAL_XY_ANGLE_MIN_DEG) ||
        (s_mount_cal.xy_angle_deg > APP_R8R28_CAL_XY_ANGLE_MAX_DEG)) return 0U;

    tfl_vec3_cross(x_from_y_pair, y_from_x_pair, z_from_pairs);
    if (tfl_vec3_normalize(z_from_pairs) == 0U) return 0U;

    /* Upright is validation only. Never blend it into the solved Z. */
    dz = tfl_vec3_dot(z_from_pairs, s_mount_cal.z_axis_i);
    s_mount_cal.z_agreement = dz;
    if (s_mount_cal.z_agreement < APP_R8R29_CAL_UPRIGHT_Z_AGREE_MIN) return 0U;

    tfl_vec3_cross(z_from_pairs, x_from_y_pair, y_expected);
    if (tfl_vec3_normalize(y_expected) == 0U) return 0U;
    s_mount_cal.axis_agreement = tfl_vec3_dot(y_expected, y_from_x_pair);
    if (s_mount_cal.axis_agreement < APP_R8R28_CAL_AXIS_AGREE_MIN) return 0U;

    /* Symmetric orthonormalization: fuse direct +X from the Y-pair plane with
     * +X reconstructed from the independent X-pair-derived +Y. */
    tfl_vec3_cross(y_from_x_pair, z_from_pairs, x_from_y);
    if (tfl_vec3_normalize(x_from_y) == 0U) return 0U;
    x_blend[0] = x_from_y_pair[0] + x_from_y[0];
    x_blend[1] = x_from_y_pair[1] + x_from_y[1];
    x_blend[2] = x_from_y_pair[2] + x_from_y[2];
    if (tfl_vec3_normalize(x_blend) == 0U) return 0U;

    tfl_vec3_cross(z_from_pairs, x_blend, y_final);
    if (tfl_vec3_normalize(y_final) == 0U) return 0U;
    tfl_vec3_cross(y_final, z_from_pairs, x_final);
    if (tfl_vec3_normalize(x_final) == 0U) return 0U;

    s_mount_cal.r_rocket_from_imu[0][0] = x_final[0];
    s_mount_cal.r_rocket_from_imu[0][1] = x_final[1];
    s_mount_cal.r_rocket_from_imu[0][2] = x_final[2];
    s_mount_cal.r_rocket_from_imu[1][0] = y_final[0];
    s_mount_cal.r_rocket_from_imu[1][1] = y_final[1];
    s_mount_cal.r_rocket_from_imu[1][2] = y_final[2];
    s_mount_cal.r_rocket_from_imu[2][0] = z_from_pairs[0];
    s_mount_cal.r_rocket_from_imu[2][1] = z_from_pairs[1];
    s_mount_cal.r_rocket_from_imu[2][2] = z_from_pairs[2];

    s_mount_cal.det = mount_cal_det3(s_mount_cal.r_rocket_from_imu);
    s_mount_cal.ortho_error = mount_cal_ortho_error(s_mount_cal.r_rocket_from_imu);
    if ((s_mount_cal.det < APP_R8R27_CAL_DET_MIN) ||
        (s_mount_cal.det > APP_R8R27_CAL_DET_MAX) ||
        (s_mount_cal.ortho_error > APP_R8R27_CAL_ORTHO_ERR_MAX)) return 0U;
    return 1U;
}

static uint8_t mount_cal_step(const SensorData_t *sensor,
                              const FullStateESKFData_t *eskf)
{
    float a[3], axis[3], angle_deg = 0.0f, metric = 0.0f, xy = 0.0f;
    uint8_t still;

    if ((sensor == 0) || (eskf == 0)) return 0U;
    still = mount_cal_is_still(sensor, eskf);

    if (s_mount_cal.phase == 0U)
    {
        s_mount_cal.phase = 1U;
        mount_cal_clear_sum(s_mount_cal.upright_sum);
        s_mount_cal.upright_count = 0U;
    }

    if (s_mount_cal.phase == 1U)
    {
        if (still == 0U)
        {
            mount_cal_clear_sum(s_mount_cal.upright_sum);
            s_mount_cal.upright_count = 0U;
        }
        else
        {
            mount_cal_accumulate(s_mount_cal.upright_sum, sensor);
            if (s_mount_cal.upright_count < 65535U) s_mount_cal.upright_count++;
            if (s_mount_cal.upright_count >= APP_R8R26_CAL_UPRIGHT_SAMPLES)
            {
                if (mount_cal_mean_unit(s_mount_cal.upright_sum, s_mount_cal.upright_count,
                                        s_mount_cal.z_axis_i) == 0U)
                {
                    s_mount_cal.phase = 11U; s_mount_cal.fault = 1U;
                }
                else
                {
                    s_mount_cal.phase = 2U;
                    mount_cal_clear_sum(s_mount_cal.tilt_sum);
                    s_mount_cal.tilt_count = 0U;
                }
            }
        }
        mount_cal_publish();
        return 0U;
    }

    a[0] = sensor->accel_x_filtered_g;
    a[1] = sensor->accel_y_filtered_g;
    a[2] = sensor->accel_z_filtered_g;
    if (tfl_vec3_normalize(a) == 0U)
    {
        s_mount_cal.phase = 11U; s_mount_cal.fault = 2U;
        mount_cal_publish();
        return 0U;
    }

    if (s_mount_cal.phase == 2U || s_mount_cal.phase == 3U)
    {
        if (mount_cal_axis_from_tilt(a, s_mount_cal.z_axis_i, axis, &angle_deg) == 0U) angle_deg = 0.0f;
        s_mount_cal.tilt_deg = angle_deg;
    }
    if (s_mount_cal.phase == 2U)
    {
        if ((still != 0U) && (angle_deg >= APP_R8R26_CAL_TILT_MIN_DEG) && (angle_deg <= APP_R8R26_CAL_TILT_MAX_DEG))
        {
            s_mount_cal.phase = 3U; mount_cal_clear_sum(s_mount_cal.tilt_sum); s_mount_cal.tilt_count = 0U;
        }
        mount_cal_publish(); return 0U;
    }
    if (s_mount_cal.phase == 3U)
    {
        if ((still == 0U) || (angle_deg < (APP_R8R26_CAL_TILT_MIN_DEG - 2.0f)) || (angle_deg > (APP_R8R26_CAL_TILT_MAX_DEG + 4.0f)))
        {
            s_mount_cal.phase = 2U; mount_cal_clear_sum(s_mount_cal.tilt_sum); s_mount_cal.tilt_count = 0U;
        }
        else
        {
            mount_cal_accumulate(s_mount_cal.tilt_sum, sensor); s_mount_cal.tilt_count = tfl_sat_inc16(s_mount_cal.tilt_count);
            if (s_mount_cal.tilt_count >= APP_R8R26_CAL_TILT_SAMPLES)
            {
                float mean[3];
                if ((mount_cal_mean_unit(s_mount_cal.tilt_sum, s_mount_cal.tilt_count, mean) == 0U) ||
                    (mount_cal_axis_from_tilt(mean, s_mount_cal.z_axis_i, s_mount_cal.x_plus_axis_i, &s_mount_cal.tilt_deg) == 0U))
                { s_mount_cal.phase = 11U; s_mount_cal.fault = 3U; }
                else
                { s_mount_cal.phase = 4U; mount_cal_clear_sum(s_mount_cal.neg_x_sum); s_mount_cal.neg_x_count = 0U; }
            }
        }
        mount_cal_publish(); return 0U;
    }

    if (s_mount_cal.phase == 4U || s_mount_cal.phase == 5U)
    {
        if (mount_cal_opposite_pose_acceptable(a, s_mount_cal.x_plus_axis_i, &angle_deg, &metric) != 0U)
        { s_mount_cal.neg_x_tilt_deg = angle_deg; s_mount_cal.x_opposition = metric; }
        else if (mount_cal_axis_from_tilt(a, s_mount_cal.z_axis_i, axis, &angle_deg) != 0U)
        { s_mount_cal.neg_x_tilt_deg = angle_deg; s_mount_cal.x_opposition = -tfl_vec3_dot(s_mount_cal.x_plus_axis_i, axis); }
    }
    if (s_mount_cal.phase == 4U)
    {
        if ((still != 0U) && (mount_cal_opposite_pose_acceptable(a, s_mount_cal.x_plus_axis_i, &angle_deg, &metric) != 0U))
        { s_mount_cal.phase = 5U; mount_cal_clear_sum(s_mount_cal.neg_x_sum); s_mount_cal.neg_x_count = 0U; }
        mount_cal_publish(); return 0U;
    }
    if (s_mount_cal.phase == 5U)
    {
        if ((still == 0U) || (mount_cal_opposite_pose_acceptable(a, s_mount_cal.x_plus_axis_i, &angle_deg, &metric) == 0U))
        { s_mount_cal.phase = 4U; mount_cal_clear_sum(s_mount_cal.neg_x_sum); s_mount_cal.neg_x_count = 0U; }
        else
        {
            mount_cal_accumulate(s_mount_cal.neg_x_sum, sensor); s_mount_cal.neg_x_count = tfl_sat_inc16(s_mount_cal.neg_x_count);
            if (s_mount_cal.neg_x_count >= APP_R8R28_CAL_NEG_X_TILT_SAMPLES)
            {
                float mean[3];
                if ((mount_cal_mean_unit(s_mount_cal.neg_x_sum, s_mount_cal.neg_x_count, mean) == 0U) ||
                    (mount_cal_axis_from_tilt(mean, s_mount_cal.z_axis_i, s_mount_cal.x_minus_axis_i, &s_mount_cal.neg_x_tilt_deg) == 0U) ||
                    (mount_cal_pair_axis(s_mount_cal.tilt_sum, s_mount_cal.tilt_count,
                                         s_mount_cal.neg_x_sum, s_mount_cal.neg_x_count,
                                         s_mount_cal.x_axis_meas_i, &s_mount_cal.x_opposition) == 0U))
                { s_mount_cal.phase = 11U; s_mount_cal.fault = 4U; }
                else
                { s_mount_cal.phase = 6U; mount_cal_clear_sum(s_mount_cal.y_tilt_sum); s_mount_cal.y_tilt_count = 0U; }
            }
        }
        mount_cal_publish(); return 0U;
    }

    if (s_mount_cal.phase == 6U || s_mount_cal.phase == 7U)
    {
        if (mount_cal_plus_y_pose_acceptable(a, &angle_deg, &xy, &metric) != 0U)
        { s_mount_cal.y_tilt_deg = angle_deg; s_mount_cal.xy_angle_deg = xy; s_mount_cal.axis_agreement = metric; }
        else
        {
            s_mount_cal.y_tilt_deg = 0.0f;
        }
    }
    if (s_mount_cal.phase == 6U)
    {
        if ((still != 0U) && (mount_cal_plus_y_pose_acceptable(a, &angle_deg, &xy, &metric) != 0U))
        { s_mount_cal.phase = 7U; mount_cal_clear_sum(s_mount_cal.y_tilt_sum); s_mount_cal.y_tilt_count = 0U; }
        mount_cal_publish(); return 0U;
    }
    if (s_mount_cal.phase == 7U)
    {
        if ((still == 0U) || (mount_cal_plus_y_pose_acceptable(a, &angle_deg, &xy, &metric) == 0U))
        { s_mount_cal.phase = 6U; mount_cal_clear_sum(s_mount_cal.y_tilt_sum); s_mount_cal.y_tilt_count = 0U; }
        else
        {
            mount_cal_accumulate(s_mount_cal.y_tilt_sum, sensor); s_mount_cal.y_tilt_count = tfl_sat_inc16(s_mount_cal.y_tilt_count);
            if (s_mount_cal.y_tilt_count >= APP_R8R27_CAL_Y_TILT_SAMPLES)
            {
                float mean[3];
                if (mount_cal_mean_unit(s_mount_cal.y_tilt_sum, s_mount_cal.y_tilt_count, mean) == 0U)
                { s_mount_cal.phase = 11U; s_mount_cal.fault = 5U; }
                else
                {
                    s_mount_cal.y_plus_axis_i[0] = mean[0];
                    s_mount_cal.y_plus_axis_i[1] = mean[1];
                    s_mount_cal.y_plus_axis_i[2] = mean[2];
                    s_mount_cal.phase = 8U;
                    mount_cal_clear_sum(s_mount_cal.neg_y_sum);
                    s_mount_cal.neg_y_count = 0U;
                }
            }
        }
        mount_cal_publish(); return 0U;
    }

    if (s_mount_cal.phase == 8U || s_mount_cal.phase == 9U)
    {
        if (mount_cal_minus_y_pose_acceptable(a, &angle_deg, &metric) != 0U)
        { s_mount_cal.neg_y_tilt_deg = angle_deg; s_mount_cal.y_opposition = metric; }
        else
        { s_mount_cal.neg_y_tilt_deg = 0.0f; }
    }
    if (s_mount_cal.phase == 8U)
    {
        if ((still != 0U) && (mount_cal_minus_y_pose_acceptable(a, &angle_deg, &metric) != 0U))
        { s_mount_cal.phase = 9U; mount_cal_clear_sum(s_mount_cal.neg_y_sum); s_mount_cal.neg_y_count = 0U; }
        mount_cal_publish(); return 0U;
    }
    if (s_mount_cal.phase == 9U)
    {
        if ((still == 0U) || (mount_cal_minus_y_pose_acceptable(a, &angle_deg, &metric) == 0U))
        { s_mount_cal.phase = 8U; mount_cal_clear_sum(s_mount_cal.neg_y_sum); s_mount_cal.neg_y_count = 0U; }
        else
        {
            mount_cal_accumulate(s_mount_cal.neg_y_sum, sensor); s_mount_cal.neg_y_count = tfl_sat_inc16(s_mount_cal.neg_y_count);
            if (s_mount_cal.neg_y_count >= APP_R8R28_CAL_NEG_Y_TILT_SAMPLES)
            {
                float mean[3];
                if ((mount_cal_mean_unit(s_mount_cal.neg_y_sum, s_mount_cal.neg_y_count, mean) == 0U) ||
                    (mount_cal_build_matrix_five_pose() == 0U))
                { s_mount_cal.valid = 0U; s_mount_cal.phase = 11U; s_mount_cal.fault = 6U; }
                else
                { s_mount_cal.valid = 1U; s_mount_cal.phase = 10U; s_mount_cal.fault = 0U; }
            }
        }
        mount_cal_publish(); return s_mount_cal.valid;
    }

    mount_cal_publish();
    return s_mount_cal.valid;
}

static void mount_cal_transform_vec(float x_i, float y_i, float z_i, float out_r[3])
{
    out_r[0] = (s_mount_cal.r_rocket_from_imu[0][0] * x_i) +
               (s_mount_cal.r_rocket_from_imu[0][1] * y_i) +
               (s_mount_cal.r_rocket_from_imu[0][2] * z_i);
    out_r[1] = (s_mount_cal.r_rocket_from_imu[1][0] * x_i) +
               (s_mount_cal.r_rocket_from_imu[1][1] * y_i) +
               (s_mount_cal.r_rocket_from_imu[1][2] * z_i);
    out_r[2] = (s_mount_cal.r_rocket_from_imu[2][0] * x_i) +
               (s_mount_cal.r_rocket_from_imu[2][1] * y_i) +
               (s_mount_cal.r_rocket_from_imu[2][2] * z_i);
}

static uint8_t mount_cal_correct_attitude_rates(const SensorData_t *sensor,
                                                const FullStateESKFData_t *eskf,
                                                float *rocket_roll_rad,
                                                float *rocket_pitch_rad,
                                                float *rocket_p_rad_s,
                                                float *rocket_q_rad_s)
{
    float up_i[3], up_r[3], gyro_r[3];
    float qw, qx, qy, qz;
    if ((sensor == 0) || (eskf == 0) || (s_mount_cal.valid == 0U)) return 0U;
    qw = eskf->q_w; qx = eskf->q_x; qy = eskf->q_y; qz = eskf->q_z;
    up_i[0] = 2.0f * ((qx * qz) - (qw * qy));
    up_i[1] = 2.0f * ((qy * qz) + (qw * qx));
    up_i[2] = 1.0f - (2.0f * ((qx * qx) + (qy * qy)));
    mount_cal_transform_vec(up_i[0], up_i[1], up_i[2], up_r);
    if (tfl_vec3_normalize(up_r) == 0U) return 0U;
    *rocket_roll_rad = atan2f(up_r[1], up_r[2]);
    *rocket_pitch_rad = -asinf(tfl_clampf(up_r[0], -1.0f, 1.0f));

    mount_cal_transform_vec(
        sensor->gyro_x_filtered_dps - eskf->gyro_bias_x_dps,
        sensor->gyro_y_filtered_dps - eskf->gyro_bias_y_dps,
        sensor->gyro_z_filtered_dps - eskf->gyro_bias_z_dps,
        gyro_r);
    *rocket_p_rad_s = gyro_r[0] * TFL_DEG2RAD;
    *rocket_q_rad_s = gyro_r[1] * TFL_DEG2RAD;
    return 1U;
}

/* -------------------------------------------------------------------------- */
/* R14p3b/R14p4 adaptive vertical controller                                 */
/* -------------------------------------------------------------------------- */

static const float s_turn_bp_hw[11] =
    {0.00f, 0.10f, 0.25f, 0.50f, 0.75f, 1.00f, 1.25f, 1.50f, 2.00f, 2.50f, 3.00f};
static const float s_cv_bp_hw[11] =
    {0.00f, 0.00f, 0.30f, 0.55f, 0.92f, 1.16f, 1.45f, 1.66f, 1.78f, 1.81f, 1.82f};

static float tfl_interp(const float *x, const float *y, uint32_t n, float q)
{
    uint32_t k;
    if (q <= x[0]) return y[0];
    if (q >= x[n - 1U]) return y[n - 1U];
    for (k = 0U; k + 1U < n; k++)
    {
        if ((q >= x[k]) && (q <= x[k + 1U]))
        {
            float den = x[k + 1U] - x[k];
            float a = (den > 1.0e-9f) ? ((q - x[k]) / den) : 0.0f;
            return y[k] + a * (y[k + 1U] - y[k]);
        }
    }
    return y[n - 1U];
}

static float tfl_turns_to_u(float turns)
{
    float cv;
    turns = tfl_clampf(turns, 0.0f, TGY_CTRL_VALVE_MAX_TURNS);
    cv = tfl_interp(s_turn_bp_hw, s_cv_bp_hw, 11U, turns);
    return tfl_clampf(cv / TGY_CTRL_CV_MAX_HW, 0.0f, 1.0f);
}

static float tfl_u_to_turns(float u)
{
    float cv = tfl_clampf(u, 0.0f, 1.0f) * TGY_CTRL_CV_MAX_HW;
    return tfl_clampf(tfl_interp(s_cv_bp_hw, s_turn_bp_hw, 11U, cv),
                      0.0f, TGY_CTRL_VALVE_MAX_TURNS);
}

static float tfl_u_to_actuator(float u)
{
    return tfl_clampf(tfl_u_to_turns(u) / TGY_CTRL_VALVE_MAX_TURNS, 0.0f, 1.0f);
}

static uint32_t tfl_samples(float seconds)
{
    float n = ceilf(seconds / TGY_CTRL_TS_S);
    if (n < 1.0f) n = 1.0f;
    if (n > 4294967040.0f) n = 4294967040.0f;
    return (uint32_t)n;
}

static void hover_reset(void)
{
    memset(&s_hover, 0, sizeof(s_hover));
    s_hover.initialized = 1U;
    s_hover.state = 0U;
    s_hover.b_hat = TGY_CTRL_B_INIT;
    s_hover.b_auth = TGY_CTRL_B_INIT;
    s_hover.b_id_est = TGY_CTRL_B_INIT;
    s_hover.id_probe_u = TGY_CTRL_ID_PROBE_START;
    s_separation_z_valid = 0U;
    s_separation_z_cg_m = 0.0f;
    s_main_ceiling_active = 0U;
}

static HoverOutput_t hover_step(float z, float v, float turns_actual,
                                uint8_t health_ok, uint8_t release_event)
{
    HoverOutput_t out;
    float turns_safe;
    float u_actual;
    float a_raw = 0.0f;
    float b_inst = TGY_CTRL_B_INIT;
    float b_safe;
    float ez = 0.0f;
    float a_cmd = 0.0f;
    float v_ref = 0.0f;
    float u_target = 0.0f;
    float d_stop_pred = 0.0f;

    if (s_hover.initialized == 0U) hover_reset();
    memset(&out, 0, sizeof(out));

    if ((tfl_finite(z) == 0U) || (tfl_finite(v) == 0U) ||
        (tfl_finite(turns_actual) == 0U))
    {
        s_hover.safe_latch = 1U;
    }

    turns_safe = tfl_clampf(turns_actual, 0.0f, TGY_CTRL_VALVE_MAX_TURNS);
    u_actual = tfl_turns_to_u(turns_safe);

    if (s_hover.v_initialized == 0U)
    {
        s_hover.v_prev = v;
        s_hover.v_initialized = 1U;
    }
    else
    {
        a_raw = (v - s_hover.v_prev) / TGY_CTRL_TS_S;
        s_hover.v_prev = v;
        a_raw = tfl_clampf(a_raw, -20.0f, 20.0f);
        s_hover.a_filt += TGY_CTRL_A_FILTER_ALPHA * (a_raw - s_hover.a_filt);
    }

    if ((release_event != 0U) && (s_hover.release_latch == 0U))
    {
        s_hover.release_latch = 1U;
        s_hover.z0_ref = z;
        s_hover.z_target = z + TGY_CTRL_Z_TARGET_DELTA_M;
        s_separation_z_valid = 1U;
        s_separation_z_cg_m = z;
    }

    if ((s_hover.release_latch != 0U) &&
        (tfl_absf(z - s_hover.z0_ref) >= TGY_CTRL_Z_HARD_LIMIT_M))
        s_hover.safe_latch = 1U;

    /* Online thrust-effectiveness estimators. */
    if ((s_hover.state >= 4U) && (s_hover.state <= 7U) &&
        (u_actual >= TGY_CTRL_EST_MIN_U))
    {
        float auth_alpha;
        b_inst = (s_hover.a_filt + TGY_CTRL_G) / u_actual;
        b_inst = tfl_clampf(b_inst, TGY_CTRL_B_MIN, TGY_CTRL_B_MAX);
        s_hover.b_hat += TGY_CTRL_B_FILTER_ALPHA * (b_inst - s_hover.b_hat);
        s_hover.b_hat = tfl_clampf(s_hover.b_hat, TGY_CTRL_B_MIN, TGY_CTRL_B_MAX);
        auth_alpha = (b_inst < s_hover.b_auth) ? TGY_CTRL_AUTH_ALPHA_DOWN : TGY_CTRL_AUTH_ALPHA_UP;
        s_hover.b_auth += auth_alpha * (b_inst - s_hover.b_auth);
        s_hover.b_auth = tfl_clampf(s_hover.b_auth, TGY_CTRL_B_MIN, TGY_CTRL_B_MAX);
    }

    b_safe = fmaxf(s_hover.b_hat, TGY_CTRL_B_MIN);

    if (s_hover.safe_latch != 0U)
        s_hover.state = 8U;

    /* STATE 0 - INIT */
    if (s_hover.state == 0U)
    {
        s_hover.init_count++;
        if (s_hover.init_count >= tfl_samples(TGY_CTRL_INIT_HOLD_S))
        {
            s_hover.state = 1U;
            s_hover.health_count = 0U;
        }
    }

    /* STATE 1 - SELF_CHECK */
    if (s_hover.state == 1U)
    {
        if ((health_ok != 0U) && (s_hover.safe_latch == 0U))
            s_hover.health_count++;
        else
            s_hover.health_count = 0U;
        if (s_hover.health_count >= tfl_samples(TGY_CTRL_HEALTH_CONFIRM_S))
        {
            s_hover.state = 2U;
            s_hover.preposition_count = 0U;
        }
    }

    /* STATE 2 - PREPOSITION. Main valve stays physically CLOSED before PE9. */
    if (s_hover.state == 2U)
    {
        if ((health_ok != 0U) && (turns_safe <= TGY_CTRL_PREPOSITION_TURN_TOL))
            s_hover.preposition_count++;
        else
            s_hover.preposition_count = 0U;
        if (s_hover.preposition_count >= tfl_samples(TGY_CTRL_PREPOSITION_CONFIRM_S))
            s_hover.state = 3U;
    }

    /* STATE 3 - READY + release-time thrust ID. */
    if (s_hover.state == 3U)
    {
        if (health_ok == 0U)
        {
            s_hover.state = 1U;
            s_hover.health_count = 0U;
            s_hover.identify_active = 0U;
        }
        else if (s_hover.release_latch != 0U)
        {
            if ((s_hover.identify_active == 0U) && (s_hover.identify_complete == 0U))
            {
                s_hover.identify_active = 1U;
                s_hover.id_probe_u = TGY_CTRL_ID_PROBE_START;
                s_hover.id_timeout_count = 0U;
                s_hover.id_step_wait_count = 0U;
                s_hover.id_airborne_count = 0U;
                s_hover.id_airborne_latch = 0U;
                s_hover.id_settle_count = 0U;
                s_hover.id_sample_count = 0U;
                s_hover.id_b_sum = 0.0f;
            }

            if (s_hover.identify_active != 0U)
            {
                uint8_t airborne_now;
                u_target = s_hover.id_probe_u;
                s_hover.id_timeout_count++;
                airborne_now = ((v >= TGY_CTRL_ID_AIRBORNE_V) ||
                                ((z - s_hover.z0_ref) >= TGY_CTRL_ID_AIRBORNE_DZ)) ? 1U : 0U;

                if (s_hover.id_airborne_latch == 0U)
                {
                    if (airborne_now != 0U) s_hover.id_airborne_count++;
                    else s_hover.id_airborne_count = 0U;
                    if (s_hover.id_airborne_count >= tfl_samples(TGY_CTRL_ID_AIRBORNE_CONFIRM_S))
                    {
                        s_hover.id_airborne_latch = 1U;
                        s_hover.id_settle_count = 0U;
                    }
                    else
                    {
                        if (tfl_absf(u_actual - s_hover.id_probe_u) <= TGY_CTRL_ID_U_MATCH_TOL)
                            s_hover.id_step_wait_count++;
                        else
                            s_hover.id_step_wait_count = 0U;
                        if ((s_hover.id_step_wait_count >= tfl_samples(TGY_CTRL_ID_STEP_WAIT_S)) &&
                            (s_hover.id_probe_u < TGY_CTRL_ID_PROBE_U_MAX))
                        {
                            s_hover.id_probe_u = fminf(s_hover.id_probe_u + TGY_CTRL_ID_PROBE_STEP,
                                                      TGY_CTRL_ID_PROBE_U_MAX);
                            s_hover.id_step_wait_count = 0U;
                            u_target = s_hover.id_probe_u;
                        }
                    }
                }
                else
                {
                    if (s_hover.id_settle_count < tfl_samples(TGY_CTRL_ID_SETTLE_S))
                        s_hover.id_settle_count++;
                    else if (u_actual >= TGY_CTRL_EST_MIN_U)
                    {
                        float b_id_inst = (s_hover.a_filt + TGY_CTRL_G) / u_actual;
                        b_id_inst = tfl_clampf(b_id_inst, TGY_CTRL_B_MIN, TGY_CTRL_B_MAX);
                        s_hover.id_b_sum += b_id_inst;
                        s_hover.id_sample_count++;
                        if (s_hover.id_sample_count >= tfl_samples(TGY_CTRL_ID_SAMPLE_S))
                        {
                            s_hover.b_id_est = s_hover.id_b_sum / (float)s_hover.id_sample_count;
                            s_hover.identify_active = 0U;
                            s_hover.identify_complete = 1U;
                            s_hover.b_hat = s_hover.b_id_est;
                            s_hover.b_auth = s_hover.b_id_est;
                            if ((s_hover.b_id_est / TGY_CTRL_G) >= TGY_CTRL_ID_AUTHORITY_MIN)
                            {
                                s_hover.state = 4U;
                                s_hover.handover_active = 1U;
                                s_hover.handover_u = s_hover.id_probe_u;
                                s_hover.iz = 0.0f;
                            }
                            else
                            {
                                s_hover.authority_latch = 1U;
                                s_hover.state = 7U;
                            }
                        }
                    }
                }

                if ((s_hover.state == 3U) &&
                    (s_hover.id_timeout_count >= tfl_samples(TGY_CTRL_ID_TIMEOUT_S)))
                {
                    s_hover.identify_active = 0U;
                    s_hover.authority_latch = 1U;
                    s_hover.state = 7U;
                }
            }
        }
    }

    if ((s_hover.state >= 4U) && (s_hover.state <= 7U) && (health_ok == 0U))
    {
        s_hover.safe_latch = 1U;
        s_hover.state = 8U;
    }

    if ((s_hover.state >= 4U) && (s_hover.state <= 6U))
    {
        float u_brake;
        float turns_brake;
        float turn_difference;
        float t_act;
        float a_delay;
        float v_up;
        float d_delay;
        float v_after;
        float d_brake;
        float d_usable;
        float v_safe;
        float vz_target_pred;

        ez = s_hover.z_target - z;
        u_brake = tfl_clampf((TGY_CTRL_G - TGY_CTRL_A_PRED_BRAKE) / b_safe, 0.0f, 1.0f);
        turns_brake = tfl_u_to_turns(u_brake);
        turn_difference = fmaxf(turns_safe - turns_brake, 0.0f);
        t_act = turn_difference / TGY_CTRL_TURN_RATE_CLOSE_TPS;
        a_delay = fmaxf(s_hover.a_filt, 0.0f);
        v_up = fmaxf(v, 0.0f);
        d_delay = v_up * t_act + 0.5f * a_delay * t_act * t_act;
        v_after = fmaxf(v_up + a_delay * t_act, 0.0f);
        d_brake = (v_after * v_after) / (2.0f * TGY_CTRL_A_PRED_BRAKE);
        d_stop_pred = d_delay + d_brake + TGY_CTRL_PRED_MARGIN;
        d_usable = fmaxf(ez - d_delay - TGY_CTRL_PRED_MARGIN, 0.0f);
        v_safe = sqrtf(fmaxf(2.0f * TGY_CTRL_A_PRED_BRAKE * d_usable, 0.0f));
        vz_target_pred = fminf(TGY_CTRL_V_UP_MAX, v_safe);

        if (s_hover.state == 4U)
        {
            v_ref = vz_target_pred;
            a_cmd = TGY_CTRL_KZ_ASC * ez + TGY_CTRL_KV_ASC * (v_ref - v);
            a_cmd = tfl_clampf(a_cmd, TGY_CTRL_A_ASC_DN, TGY_CTRL_A_ASC_UP);
            if ((ez <= d_stop_pred) || (vz_target_pred <= TGY_CTRL_V_TARGET_MAX))
            {
                s_hover.state = 5U;
                s_hover.iz = 0.0f;
                s_hover.hover_count = 0U;
            }
        }
        else if (s_hover.state == 5U)
        {
            if (ez >= 0.0f)
            {
                float v_energy = sqrtf(fmaxf(2.0f * TGY_CTRL_A_CAPTURE_PROFILE * ez, 0.0f));
                v_ref = fminf(TGY_CTRL_V_CAPTURE_MAX, v_energy);
            }
            else
            {
                v_ref = fmaxf(TGY_CTRL_KZ_CAPTURE * ez, -TGY_CTRL_V_CAPTURE_DOWN_MAX);
            }
            if ((u_actual < 0.98f) || (ez < 0.0f)) s_hover.iz += ez * TGY_CTRL_TS_S;
            s_hover.iz = tfl_clampf(s_hover.iz, -TGY_CTRL_IZ_MAX, TGY_CTRL_IZ_MAX);
            a_cmd = TGY_CTRL_KV_CAPTURE * (v_ref - v) + TGY_CTRL_KI_CAPTURE * s_hover.iz;
            a_cmd = tfl_clampf(a_cmd, TGY_CTRL_A_CAPTURE_DN, TGY_CTRL_A_CAPTURE_UP);
            if ((tfl_absf(ez) <= TGY_CTRL_Z_HOVER_BAND) &&
                (tfl_absf(v) <= TGY_CTRL_V_HOVER_BAND))
            {
                s_hover.hover_count++;
                if (s_hover.hover_count > s_hover.hover_best) s_hover.hover_best = s_hover.hover_count;
                if (s_hover.hover_count >= tfl_samples(TGY_CTRL_HOVER_CONFIRM_S))
                    s_hover.state = 6U;
            }
            else s_hover.hover_count = 0U;
        }
        else if (s_hover.state == 6U)
        {
            uint8_t allow_integrator;
            v_ref = tfl_clampf(TGY_CTRL_KZ_HOVER * ez, -TGY_CTRL_V_HOVER_MAX, TGY_CTRL_V_HOVER_MAX);
            allow_integrator = (((u_actual > 0.02f) && (u_actual < 0.98f)) ||
                                ((u_actual >= 0.98f) && (ez < 0.0f)) ||
                                ((u_actual <= 0.02f) && (ez > 0.0f))) ? 1U : 0U;
            if (allow_integrator != 0U) s_hover.iz += ez * TGY_CTRL_TS_S;
            s_hover.iz = tfl_clampf(s_hover.iz, -TGY_CTRL_IZ_MAX, TGY_CTRL_IZ_MAX);
            a_cmd = TGY_CTRL_KV_HOVER * (v_ref - v) + TGY_CTRL_KI_HOVER * s_hover.iz;
            a_cmd = tfl_clampf(a_cmd, TGY_CTRL_A_HOVER_DN, TGY_CTRL_A_HOVER_UP);
            if ((tfl_absf(ez) <= TGY_CTRL_Z_HOVER_BAND) &&
                (tfl_absf(v) <= TGY_CTRL_V_HOVER_BAND))
            {
                s_hover.hover_count++;
                if (s_hover.hover_count > s_hover.hover_best) s_hover.hover_best = s_hover.hover_count;
            }
            else s_hover.hover_count = 0U;
        }

        if ((s_hover.state >= 4U) && (s_hover.state <= 6U))
        {
            float raw_target = tfl_clampf((TGY_CTRL_G + a_cmd) / b_safe, 0.0f, 1.0f);
            u_target = raw_target;
            if ((s_hover.handover_active != 0U) && (s_hover.state == 4U))
            {
                float du_max = TGY_CTRL_HANDOVER_RATE_UP * TGY_CTRL_TS_S;
                if (u_target > s_hover.handover_u)
                    s_hover.handover_u = fminf(s_hover.handover_u + du_max, u_target);
                else
                    s_hover.handover_u = u_target;
                u_target = s_hover.handover_u;
                if (tfl_absf(raw_target - s_hover.handover_u) <= TGY_CTRL_HANDOVER_DONE_TOL)
                    s_hover.handover_active = 0U;
            }
            else if (s_hover.state != 4U)
                s_hover.handover_active = 0U;

            /* R14p3b demand-qualified authority monitor. */
            {
                float authority_ratio = fmaxf(s_hover.b_auth, TGY_CTRL_B_MIN) / TGY_CTRL_G;
                uint8_t ratio_bad = ((authority_ratio < TGY_CTRL_AUTH_RATIO_TRIP) &&
                                     (raw_target >= TGY_CTRL_AUTH_DEMAND_U) &&
                                     (a_cmd >= TGY_CTRL_AUTH_A_NEG)) ? 1U : 0U;
                uint8_t fullopen_bad = ((u_actual >= TGY_CTRL_AUTH_U_SAT) &&
                                        (s_hover.a_filt < TGY_CTRL_AUTH_A_NEG)) ? 1U : 0U;
                if ((ratio_bad != 0U) || (fullopen_bad != 0U))
                {
                    s_hover.authority_count++;
                    if (s_hover.authority_count >= tfl_samples(TGY_CTRL_AUTH_CONFIRM_S))
                    {
                        s_hover.authority_latch = 1U;
                        s_hover.state = 7U;
                        u_target = 1.0f;
                    }
                }
                else s_hover.authority_count = 0U;
            }
        }
    }

    if (s_hover.state == 7U)
    {
        s_hover.authority_latch = 1U;
        u_target = 1.0f;
    }
    if (s_hover.state == 8U)
        u_target = 0.0f;
    if ((s_hover.state <= 2U) || ((s_hover.state == 3U) && (s_hover.release_latch == 0U)))
        u_target = 0.0f;

    out.valve_cmd = tfl_clampf(u_target, 0.0f, 1.0f);
    out.turns_target = tfl_u_to_turns(out.valve_cmd);
    out.actuator_cmd = tfl_u_to_actuator(out.valve_cmd);
    out.u_actual = u_actual;
    out.z_reference_m = (s_hover.release_latch != 0U) ? s_hover.z_target : z;
    out.state = s_hover.state;
    out.hover_best_s = (float)s_hover.hover_best * TGY_CTRL_TS_S;
    out.thrust_shortage = s_hover.authority_latch;
    out.a_filt = s_hover.a_filt;
    out.a_cmd = a_cmd;
    out.b_hat = s_hover.b_hat;
    out.b_auth = s_hover.b_auth;
    out.authority_ratio = fmaxf(s_hover.b_auth, TGY_CTRL_B_MIN) / TGY_CTRL_G;
    out.u_hover_est = TGY_CTRL_G / fmaxf(s_hover.b_hat, TGY_CTRL_B_MIN);
    out.d_stop_pred = d_stop_pred;
    out.v_ref = v_ref;
    return out;
}

/* -------------------------------------------------------------------------- */
/* Horizontal EnvelopeUyumlu port                                             */
/* -------------------------------------------------------------------------- */

static void horizontal_step(float x, float y, float vx, float vy, float z,
                            uint8_t hover_state,
                            float *target_pitch_rad, float *target_yaw_rad)
{
    const float normal_tilt = 4.0f * TFL_DEG2RAD;
    const float max_tilt = 4.8f * TFL_DEG2RAD;
    const float combined_tilt = 5.5f * TFL_DEG2RAD;
    const float combined_trigger = TFL_G * tanf(1.5f * TFL_DEG2RAD);
    float ex, ey, vx_use, vy_use;
    float ax_cmd, ay_cmd;
    float active_max_tilt;

    *target_pitch_rad = 0.0f;
    *target_yaw_rad = 0.0f;

    if ((tfl_finite(x) == 0U) || (tfl_finite(y) == 0U) ||
        (tfl_finite(vx) == 0U) || (tfl_finite(vy) == 0U) ||
        (tfl_finite(z) == 0U))
        return;

    if (hover_state >= 4U) return;

    ex = x;
    ey = y;

    if ((tfl_absf(ex) < 0.03f) && (tfl_absf(vx) < 0.03f))
    {
        ex = 0.0f;
        vx_use = 0.0f;
    }
    else vx_use = vx;

    if ((tfl_absf(ey) < 0.03f) && (tfl_absf(vy) < 0.03f))
    {
        ey = 0.0f;
        vy_use = 0.0f;
    }
    else vy_use = vy;

    if (hover_state >= 3U)
    {
        float landing_tilt_limit;
        float a_landing_max;

        ax_cmd = -0.10f * ex - 1.40f * vx_use;
        ay_cmd = -0.10f * ey - 1.40f * vy_use;

        if (z > 1.50f)
            landing_tilt_limit = 0.80f * TFL_DEG2RAD;
        else if (z > 1.00f)
        {
            float ratio = (z - 1.00f) / 0.50f;
            landing_tilt_limit = (0.50f + ratio * 0.30f) * TFL_DEG2RAD;
        }
        else if (z > 0.70f)
        {
            float ratio = (z - 0.70f) / 0.30f;
            landing_tilt_limit = (0.25f + ratio * 0.25f) * TFL_DEG2RAD;
        }
        else if (z > 0.60f)
        {
            float ratio = (z - 0.60f) / 0.10f;
            landing_tilt_limit = ratio * 0.25f * TFL_DEG2RAD;
        }
        else
        {
            *target_pitch_rad = 0.0f;
            *target_yaw_rad = 0.0f;
            return;
        }

        a_landing_max = TFL_G * tanf(landing_tilt_limit);
        ax_cmd = tfl_clampf(ax_cmd, -a_landing_max, a_landing_max);
        ay_cmd = tfl_clampf(ay_cmd, -a_landing_max, a_landing_max);
        *target_pitch_rad = -atan2f(ay_cmd, TFL_G);
        *target_yaw_rad = atan2f(ax_cmd, TFL_G);
        *target_pitch_rad = tfl_clampf(*target_pitch_rad,
                                      -landing_tilt_limit, landing_tilt_limit);
        *target_yaw_rad = tfl_clampf(*target_yaw_rad,
                                    -landing_tilt_limit, landing_tilt_limit);
        return;
    }
    else
    {
        float x_pred = x + 1.0f * vx_use;
        float y_pred = y + 1.0f * vy_use;
        float edge_x = tfl_clampf((tfl_absf(x_pred) - 1.0f) / 2.0f, 0.0f, 1.0f);
        float edge_y = tfl_clampf((tfl_absf(y_pred) - 1.0f) / 2.0f, 0.0f, 1.0f);
        float kp_x = 0.40f + edge_x * (0.90f - 0.40f);
        float kd_x = 0.90f + edge_x * (1.60f - 0.90f);
        float kp_y = 0.25f + edge_y * (0.75f - 0.25f);
        float kd_y = 1.00f + edge_y * (2.40f - 1.00f);
        uint8_t combined_control;
        float a_edge_max;
        float stop_x, stop_y;
        float edge_level;
        float tilt_limit;
        float a_axis_max;

        ax_cmd = -kp_x * ex - kd_x * vx_use;
        ay_cmd = -kp_y * ey - kd_y * vy_use;

        if ((y * vy_use < 0.0f) && (tfl_absf(y) > 0.03f))
            ay_cmd -= 0.65f * vy_use;

        combined_control =
            ((tfl_absf(ax_cmd) >= combined_trigger) &&
             (tfl_absf(ay_cmd) >= combined_trigger) &&
             (((x * vx_use) > 0.0f) || ((y * vy_use) > 0.0f))) ? 1U : 0U;

        active_max_tilt = (combined_control != 0U) ? combined_tilt : max_tilt;
        a_edge_max = TFL_G * tanf(active_max_tilt);
        stop_x = (vx_use * vx_use) / (2.0f * 0.40f);
        stop_y = (vy_use * vy_use) / (2.0f * 0.40f);

        if (tfl_absf(x) >= 3.0f)
        {
            if (x > 0.0f) ax_cmd = -a_edge_max;
            else if (x < 0.0f) ax_cmd = a_edge_max;
        }
        else if ((x * vx_use > 0.0f) &&
                 ((tfl_absf(x) + stop_x + 0.40f) >= 3.0f))
        {
            if (vx_use > 0.0f) ax_cmd = -a_edge_max;
            else if (vx_use < 0.0f) ax_cmd = a_edge_max;
        }

        if (tfl_absf(y) >= 3.0f)
        {
            if (y * vy_use >= 0.0f)
            {
                if (y > 0.0f) ay_cmd = -a_edge_max;
                else if (y < 0.0f) ay_cmd = a_edge_max;
            }
        }
        else if ((y * vy_use > 0.0f) &&
                 ((tfl_absf(y) + stop_y + 0.40f) >= 3.0f))
        {
            if (vy_use > 0.0f) ay_cmd = -a_edge_max;
            else if (vy_use < 0.0f) ay_cmd = a_edge_max;
        }

        edge_level = fmaxf(edge_x, edge_y);
        tilt_limit = normal_tilt + edge_level * (active_max_tilt - normal_tilt);
        if (combined_control != 0U) tilt_limit = combined_tilt;
        a_axis_max = TFL_G * tanf(tilt_limit);
        ax_cmd = tfl_clampf(ax_cmd, -a_axis_max, a_axis_max);
        ay_cmd = tfl_clampf(ay_cmd, -a_axis_max, a_axis_max);
        *target_pitch_rad = -atan2f(ay_cmd, TFL_G);
        *target_yaw_rad = atan2f(ax_cmd, TFL_G);
        *target_pitch_rad = tfl_clampf(*target_pitch_rad,
                                      -active_max_tilt, active_max_tilt);
        *target_yaw_rad = tfl_clampf(*target_yaw_rad,
                                    -active_max_tilt, active_max_tilt);
    }
}

/* -------------------------------------------------------------------------- */
/* RCS V7.13.4 port                                                           */
/* -------------------------------------------------------------------------- */

static void rcs_axis_init(RCSAxis_t *a, uint16_t min_off_n)
{
    memset(a, 0, sizeof(*a));
    a->event_armed = 1U;
    a->off_age = min_off_n;
    a->event_end_age = 65535U;
}

static void rcs_reset(void)
{
    uint16_t min_off_n = tfl_ceil_samples(0.120f);
    memset(&s_rcs, 0, sizeof(s_rcs));
    rcs_axis_init(&s_rcs.pitch, min_off_n);
    rcs_axis_init(&s_rcs.yaw, min_off_n);
    s_rcs.initialized = 1U;
}

static void rcs_force_safe_axis(RCSAxis_t *a)
{
    a->desired = 0;
    a->applied = 0;
    a->pending = 0;
    a->dead_active = 0U;
    a->on_age = 0U;
    a->mode = 0U;
    a->track_active = 0U;
    a->track_sign = 0;
    a->track_age = 0U;
    a->track_pulse_n = 0U;
    a->track_cooldown = 0U;
    a->track_filtered_rate = 0.0f;
    a->track_rate_initialized = 0U;
    a->track_confirm_count = 0U;
    a->track_confirm_sign = 0;
}

static void rcs_update_rates(RCSAxis_t *a, float angle_deg,
                             float gyro_rate_dps,
                             float *rate_dps, float *trend_dps)
{
    if (a->rate_initialized == 0U)
    {
        a->filtered_rate = gyro_rate_dps;
        a->rate_initialized = 1U;
    }
    else
    {
        a->filtered_rate = gyro_rate_dps; /* RATE_ALPHA = 1.0 */
    }

    if (a->trend_initialized == 0U)
    {
        a->prev_angle = angle_deg;
        a->filtered_trend = 0.0f;
        a->trend_initialized = 1U;
    }
    else
    {
        float delta = angle_deg - a->prev_angle;
        float raw_trend;
        if (delta > 180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        raw_trend = delta / TFL_TS_S;
        a->filtered_trend += 0.25f * (raw_trend - a->filtered_trend);
        a->prev_angle = angle_deg;
    }

    *rate_dps = a->filtered_rate;
    *trend_dps = a->filtered_trend;
}

static void rcs_age_counters(RCSAxis_t *a, uint16_t window_n)
{
    a->event_end_age = tfl_sat_inc16(a->event_end_age);
    if (a->event_active != 0U) a->event_age = tfl_sat_inc16(a->event_age);
    if (a->phase == 2U) a->brake_age = tfl_sat_inc16(a->brake_age);
    if (a->phase == 4U) a->safe_age = tfl_sat_inc16(a->safe_age);

    if ((a->applied == 0) && (a->dead_active == 0U))
        a->off_age = tfl_sat_inc16(a->off_age);
    else
        a->off_age = 0U;

    a->window_age = tfl_sat_inc16(a->window_age);
    if (a->window_age >= window_n)
    {
        a->window_age = 0U;
        a->reversal_count = 0U;
    }

    a->event_window_age = tfl_sat_inc16(a->event_window_age);
    if (a->event_window_age >= window_n)
    {
        a->event_window_age = 0U;
        a->event_count = 0U;
    }
}

static int8_t rcs_correction_torque(int8_t hazard_sign)
{
    if (hazard_sign > 0) return -1;
    if (hazard_sign < 0) return 1;
    return 0;
}

static int8_t rcs_braking_torque(int8_t hazard_sign)
{
    if (hazard_sign > 0) return 1;
    if (hazard_sign < 0) return -1;
    return 0;
}

static void rcs_finish_event(RCSAxis_t *a, uint8_t immediate_rearm)
{
    a->event_active = 0U;
    a->hard_latched = 0U;
    a->brake_used = 0U;
    a->hazard_sign = 0;
    a->desired = 0;
    a->pred_count = 0U;
    a->event_end_age = 0U;
    a->safe_age = 0U;
    a->event_age = 0U;
    a->brake_age = 0U;
    a->target = 0.0f;
    a->switch_error = 0.0f;
    a->track_active = 0U;
    a->track_sign = 0;
    a->track_age = 0U;
    a->track_pulse_n = 0U;
    a->track_cooldown = 0U;
    a->track_filtered_rate = 0.0f;
    a->track_rate_initialized = 0U;
    a->track_confirm_count = 0U;
    a->track_confirm_sign = 0;
    if (immediate_rearm != 0U)
    {
        a->phase = 0U;
        a->event_armed = 1U;
        a->mode = 0U;
    }
    else
    {
        a->phase = 4U;
        a->event_armed = 0U;
        a->mode = 4U;
    }
    if (a->applied == 0)
    {
        a->off_age = 0U;
        a->on_age = 0U;
    }
}

static uint8_t rcs_start_event(RCSAxis_t *a, int8_t hazard_sign,
                               uint8_t hard_limit, uint8_t max_events)
{
    if (hazard_sign == 0) return 0U;
    a->event_count++;
    if (a->event_count > max_events) return 2U;
    a->event_active = 1U;
    a->event_armed = 0U;
    a->hard_latched = hard_limit;
    a->brake_used = 0U;
    a->hazard_sign = hazard_sign;
    a->phase = 1U;
    a->mode = 1U;
    a->event_age = 0U;
    a->pred_count = 0U;
    a->desired = rcs_correction_torque(hazard_sign);
    return 1U;
}

static uint8_t rcs_select_axis(RCSAxis_t *a, float angle, float rate,
                               float pred, uint8_t pred_valid)
{
    const float no_fire = 0.75f;
    const float limit_deg = 10.0f;
    const float hard_boundary = 9.95f;
    const float safe_target = 7.0f;
    const float alpha = 221.0f;
    const float brake_margin = 0.3f;
    const float brake_delay_s = 0.080f;
    const float brake_done = 2.0f;
    const float pred_hyst = 0.5f;
    const uint8_t pred_confirm_n = 3U;
    const float reaction_s = 0.040f;
    const float sub5_min_outward = 3.0f;
    const float sub5_growth_margin = 0.25f;
    const uint16_t min_corr_n = 4U;
    const uint16_t max_brake_n = 30U;
    const uint16_t rearm_n = 30U;
    const uint16_t hard_rearm_n = 40U;
    const uint8_t max_events = 4U;
    const uint16_t min_off_n = 12U;

    float abs_angle = tfl_absf(angle);
    float abs_rate = tfl_absf(rate);
    uint8_t hard_threat = (abs_angle >= hard_boundary) ? 1U : 0U;
    float reaction_pred = tfl_clampf(angle + rate * reaction_s, -180.0f, 180.0f);
    uint8_t reaction_growing =
        (tfl_absf(reaction_pred) >= (abs_angle + sub5_growth_margin)) ? 1U : 0U;
    uint8_t sub5_reaction_threat =
        ((abs_angle < no_fire) && (abs_rate >= sub5_min_outward) &&
         (reaction_growing != 0U) &&
         (tfl_absf(reaction_pred) >= limit_deg)) ? 1U : 0U;
    uint8_t touch_growing =
        ((pred_valid != 0U) &&
         (tfl_absf(pred) >= (abs_angle + pred_hyst))) ? 1U : 0U;
    uint8_t sub5_touch_threat =
        ((abs_angle < no_fire) && (pred_valid != 0U) &&
         (abs_rate >= sub5_min_outward) && (touch_growing != 0U) &&
         (tfl_absf(pred) >= limit_deg)) ? 1U : 0U;
    uint8_t sub5_threat =
        ((sub5_reaction_threat != 0U) || (sub5_touch_threat != 0U)) ? 1U : 0U;
    uint8_t mid_reaction_threat =
        ((abs_angle >= no_fire) && (abs_angle < hard_boundary) &&
         (abs_rate >= sub5_min_outward) && (reaction_growing != 0U) &&
         (tfl_absf(reaction_pred) >= limit_deg)) ? 1U : 0U;
    uint8_t mid_pred_threat =
        ((pred_valid != 0U) && (abs_angle >= no_fire) &&
         (abs_angle < hard_boundary) && (tfl_absf(pred) >= limit_deg)) ? 1U : 0U;
    uint8_t pred_threat =
        ((sub5_threat != 0U) || (mid_reaction_threat != 0U) ||
         (mid_pred_threat != 0U)) ? 1U : 0U;
    uint8_t pred_safe =
        ((hard_threat == 0U) && (pred_threat == 0U) &&
         ((pred_valid == 0U) || (tfl_absf(pred) <= (limit_deg - pred_hyst)))) ? 1U : 0U;
    float speed_after_delay = abs_rate + alpha * brake_delay_s;
    float travel_during_delay = abs_rate * brake_delay_s +
        0.5f * alpha * brake_delay_s * brake_delay_s;
    uint8_t pred_confirmed;

    a->stopping = (rate * rate) / (2.0f * alpha);
    a->braking_distance = travel_during_delay +
        (speed_after_delay * speed_after_delay) / (2.0f * alpha) + brake_margin;
    a->target = 0.0f;
    a->switch_error = 0.0f;

    if (a->event_active == 0U)
    {
        if (pred_threat != 0U)
        {
            if (a->pred_count < 255U) a->pred_count++;
        }
        else a->pred_count = 0U;
    }
    pred_confirmed = (a->pred_count >= pred_confirm_n) ? 1U : 0U;

    if ((abs_angle < no_fire) && (sub5_threat == 0U))
    {
        if ((a->event_active != 0U) || (a->phase != 0U) ||
            (a->event_armed == 0U))
            rcs_finish_event(a, 1U);
        else
        {
            a->mode = 0U;
            a->desired = 0;
        }
        return 0U;
    }

    if (a->phase == 4U)
    {
        if (((hard_threat != 0U) || (pred_threat != 0U)) &&
            (a->event_end_age >= min_off_n))
        {
            a->event_armed = 1U;
            a->phase = 0U;
            a->mode = 0U;
        }
        else if (pred_safe != 0U)
        {
            if (a->safe_age >= rearm_n)
            {
                a->event_armed = 1U;
                a->phase = 0U;
                a->mode = 0U;
            }
        }
        else a->safe_age = 0U;

        if ((hard_threat != 0U) && (a->event_end_age >= hard_rearm_n))
        {
            a->event_armed = 1U;
            a->phase = 0U;
            a->mode = 0U;
        }
    }

    if (a->event_active == 0U)
    {
        int8_t threat_sign;
        uint8_t start_result;
        if (a->event_armed == 0U)
        {
            a->mode = 4U;
            a->desired = 0;
            return 0U;
        }
        if ((hard_threat == 0U) && (pred_confirmed == 0U))
        {
            a->mode = 0U;
            a->desired = 0;
            return 0U;
        }

        if (hard_threat != 0U) threat_sign = tfl_sign8(angle);
        else if ((sub5_reaction_threat != 0U) || (mid_reaction_threat != 0U))
            threat_sign = tfl_sign8(reaction_pred);
        else threat_sign = tfl_sign8(pred);
        if (threat_sign == 0) threat_sign = tfl_sign8(rate);

        start_result = rcs_start_event(a, threat_sign, hard_threat, max_events);
        if (start_result == 2U)
        {
            a->desired = 0;
            return 1U;
        }
        if (start_result == 0U)
        {
            a->desired = 0;
            return 0U;
        }
    }

    if (hard_threat != 0U) a->hard_latched = 1U;

    if (a->phase == 1U)
    {
        int8_t correction = rcs_correction_torque(a->hazard_sign);
        float signed_angle;
        float remaining;
        float toward_rate;

        if ((abs_angle < no_fire) && (sub5_threat != 0U))
        {
            a->target = 0.0f;
            a->switch_error = 0.0f;
            a->mode = 1U;
            a->desired = correction;
            return 0U;
        }

        signed_angle = (float)a->hazard_sign * angle;
        remaining = signed_angle - safe_target;
        toward_rate = -(float)a->hazard_sign * rate;
        a->target = (float)a->hazard_sign * safe_target;
        a->switch_error = remaining - a->braking_distance;

        if ((a->hard_latched != 0U) && (a->brake_used != 0U) &&
            (hard_threat == 0U))
        {
            rcs_finish_event(a, 0U);
            return 0U;
        }

        if ((a->hard_latched == 0U) && (pred_valid != 0U) &&
            (a->event_age >= min_corr_n) &&
            (tfl_absf(pred) <= (limit_deg - pred_hyst)))
        {
            rcs_finish_event(a, 0U);
            return 0U;
        }

        if ((a->hard_latched == 0U) && (hard_threat == 0U) &&
            (abs_rate <= brake_done) &&
            ((pred_valid == 0U) || (tfl_absf(pred) <= (limit_deg - pred_hyst))))
        {
            rcs_finish_event(a, 0U);
            return 0U;
        }

        if (toward_rate <= 0.0f)
        {
            a->mode = 1U;
            a->desired = correction;
            return 0U;
        }

        if ((a->hard_latched == 0U) && (hard_threat == 0U) &&
            (toward_rate <= brake_done) && (remaining <= brake_margin))
        {
            rcs_finish_event(a, 0U);
            return 0U;
        }

        if (remaining <= a->braking_distance)
        {
            if ((a->brake_used == 0U) && (toward_rate > brake_done))
            {
                a->phase = 2U;
                a->brake_used = 1U;
                a->brake_age = 0U;
                a->mode = 2U;
                a->desired = rcs_braking_torque(a->hazard_sign);
                return 0U;
            }
            else
            {
                if (a->hard_latched != 0U)
                {
                    a->mode = 1U;
                    a->desired = correction;
                }
                else rcs_finish_event(a, 0U);
                return 0U;
            }
        }

        a->mode = 1U;
        a->desired = correction;
        return 0U;
    }

    if (a->phase == 2U)
    {
        float toward_rate = -(float)a->hazard_sign * rate;
        a->mode = 2U;
        if ((toward_rate <= brake_done) || (a->brake_age >= max_brake_n))
        {
            if (a->hard_latched != 0U)
            {
                if (hard_threat != 0U)
                {
                    a->phase = 1U;
                    a->mode = 1U;
                    a->brake_age = 0U;
                    a->desired = rcs_correction_torque(a->hazard_sign);
                }
                else rcs_finish_event(a, 0U);
            }
            else if (hard_threat != 0U)
            {
                a->hard_latched = 1U;
                a->phase = 1U;
                a->mode = 1U;
                a->brake_age = 0U;
                a->desired = rcs_correction_torque(a->hazard_sign);
            }
            else rcs_finish_event(a, 0U);
            return 0U;
        }
        a->desired = rcs_braking_torque(a->hazard_sign);
        return 0U;
    }

    if (a->phase == 3U)
    {
        float outward_rate = (float)a->hazard_sign * rate;
        a->mode = 5U;
        a->desired = 0;
        if ((hard_threat == 0U) && (sub5_threat == 0U))
            rcs_finish_event(a, 0U);
        else if (outward_rate >= 0.0f)
            rcs_finish_event(a, 1U);
        return 0U;
    }

    a->mode = 4U;
    a->desired = 0;
    return 0U;
}

static void rcs_apply_reference_tracking(RCSAxis_t *a, float angle,
                                         float rate, float ref_angle)
{
    const float max_abs_angle = 9.80f;
    const float start_db = APP_R8R34_RCS_TRACK_START_DB_DEG;
    const float stop_db = APP_R8R34_RCS_TRACK_STOP_DB_DEG;
    const float lookahead = 0.30f;
    const float rate_alpha = APP_R8R34_RCS_TRACK_RATE_ALPHA;
    const uint8_t confirm_n = APP_R8R34_RCS_TRACK_CONFIRM_N;
    const uint16_t pulse_n = 4U;
    float error_now;
    float error_pred;
    float error_mag;
    float rate_hold;
    float track_rate;
    uint16_t next_cooldown_n;
    int8_t error_sign;
    float moving_toward_rate;
    uint8_t crossing_target;
    int8_t control_sign = 0;

    /* R8R34: keep V7.13.4 predictive/hard-safety on the raw rate passed to
     * rcs_select_axis().  Only the low-angle reference-tracking branch gets a
     * light 30 ms-class LPF, preventing a single gyro/rate excursion from
     * creating an upright relay pulse. */
    if (a->track_rate_initialized == 0U)
    {
        a->track_filtered_rate = rate;
        a->track_rate_initialized = 1U;
    }
    else
    {
        a->track_filtered_rate += rate_alpha * (rate - a->track_filtered_rate);
    }
    track_rate = a->track_filtered_rate;

    if ((a->event_active != 0U) || (tfl_absf(angle) >= max_abs_angle))
    {
        a->track_active = 0U;
        a->track_sign = 0;
        a->track_age = 0U;
        a->track_pulse_n = 0U;
        a->track_cooldown = 0U;
        a->track_confirm_count = 0U;
        a->track_confirm_sign = 0;
        return;
    }

    error_now = angle - ref_angle;
    error_pred = error_now + track_rate * lookahead;
    error_mag = tfl_absf(error_pred);

    if (error_mag >= 4.0f)
    {
        rate_hold = 1.00f;
        next_cooldown_n = 16U;
    }
    else if (error_mag >= 2.0f)
    {
        rate_hold = 0.70f;
        next_cooldown_n = 20U;
    }
    else
    {
        rate_hold = 0.40f;
        next_cooldown_n = 25U;
    }

    if (a->track_active != 0U)
    {
        a->mode = 6U;
        a->desired = rcs_correction_torque(a->track_sign);
        a->track_age = tfl_sat_inc16(a->track_age);
        if (a->track_age >= a->track_pulse_n)
        {
            a->track_active = 0U;
            a->track_sign = 0;
            a->track_age = 0U;
            a->track_pulse_n = 0U;
            a->track_cooldown = next_cooldown_n;
            a->track_confirm_count = 0U;
            a->track_confirm_sign = 0;
            a->desired = 0;
        }
        return;
    }

    if (a->track_cooldown > 0U)
    {
        a->track_cooldown--;
        a->track_confirm_count = 0U;
        a->track_confirm_sign = 0;
        a->desired = 0;
        a->mode = 6U;
        return;
    }

    if ((tfl_absf(error_now) <= stop_db) && (tfl_absf(error_pred) <= start_db))
    {
        a->track_confirm_count = 0U;
        a->track_confirm_sign = 0;
        a->desired = 0;
        a->mode = 0U;
        return;
    }

    error_sign = tfl_sign8(error_now);
    moving_toward_rate = (error_sign != 0) ?
        (-(float)error_sign * track_rate) : 0.0f;
    crossing_target =
        ((error_sign != 0) && (tfl_sign8(error_pred) != 0) &&
         (tfl_sign8(error_pred) != error_sign)) ? 1U : 0U;

    if (crossing_target != 0U)
    {
        if (moving_toward_rate >= 1.5f)
            control_sign = (int8_t)(-error_sign);
    }
    else if (moving_toward_rate < rate_hold)
    {
        if (error_mag >= start_db)
        {
            control_sign = tfl_sign8(error_pred);
            if (control_sign == 0) control_sign = error_sign;
        }
    }

    if (control_sign == 0)
    {
        a->track_confirm_count = 0U;
        a->track_confirm_sign = 0;
        a->desired = 0;
        a->mode = (crossing_target != 0U || moving_toward_rate >= rate_hold) ? 6U : 0U;
        return;
    }

    /* R8R34: a fine-tracking pulse must be requested with the same sign for
     * three consecutive 100 Hz samples (30 ms). Hard/predictive safety events
     * are handled earlier and are NOT subject to this confirmation. */
    if (a->track_confirm_sign != control_sign)
    {
        a->track_confirm_sign = control_sign;
        a->track_confirm_count = 1U;
    }
    else if (a->track_confirm_count < 255U)
    {
        a->track_confirm_count++;
    }

    if (a->track_confirm_count < confirm_n)
    {
        a->desired = 0;
        a->mode = 6U;
        return;
    }

    a->track_confirm_count = 0U;
    a->track_confirm_sign = 0;
    a->track_active = 1U;
    a->track_sign = control_sign;
    a->track_age = 0U;
    a->track_pulse_n = pulse_n;
    a->desired = rcs_correction_torque(control_sign);
    a->mode = 6U;
}

static uint8_t rcs_process_output(RCSAxis_t *a)
{
    const uint16_t dead_n = 4U;
    const uint16_t min_off_n = 12U;
    const uint8_t max_reversals = 8U;
    const uint16_t max_on_n = 4U;

    if (a->applied != 0) a->on_age = tfl_sat_inc16(a->on_age);
    else a->on_age = 0U;

    if ((a->applied != 0) && (a->on_age >= max_on_n))
    {
        a->applied = 0;
        a->on_age = 0U;
        a->off_age = 0U;
        return 0U;
    }

    if (a->dead_active != 0U)
    {
        a->applied = 0;
        a->mode = 3U;
        a->dead_age = tfl_sat_inc16(a->dead_age);
        if (a->desired == 0)
        {
            a->pending = 0;
            a->dead_active = 0U;
            a->dead_age = 0U;
            a->off_age = 0U;
            return 0U;
        }
        if (a->desired != a->pending)
        {
            a->pending = a->desired;
            a->dead_age = 0U;
        }
        if (a->dead_age >= dead_n)
        {
            a->applied = a->pending;
            a->on_age = 0U;
            a->pending = 0;
            a->dead_active = 0U;
            a->dead_age = 0U;
            a->off_age = 0U;
        }
        return 0U;
    }

    if (a->desired == a->applied) return 0U;

    if ((a->applied != 0) && (a->desired == -a->applied))
    {
        a->applied = 0;
        a->pending = a->desired;
        a->dead_active = 1U;
        a->dead_age = 0U;
        a->off_age = 0U;
        a->reversal_count++;
        if (a->reversal_count > max_reversals) return 1U;
        return 0U;
    }

    if (a->desired == 0)
    {
        a->applied = 0;
        a->on_age = 0U;
        a->off_age = 0U;
        return 0U;
    }

    if (a->off_age >= min_off_n)
    {
        a->applied = a->desired;
        a->on_age = 0U;
        a->off_age = 0U;
    }
    return 0U;
}

static RCSOutput_t rcs_step(float target_pitch_rad, float target_yaw_rad,
                            float pitch_rad, float yaw_rad,
                            float pitch_rate_rad_s, float yaw_rate_rad_s,
                            float z_cg, float vz, uint8_t hover_state)
{
    RCSOutput_t out;
    float pitch_deg, yaw_deg;
    float pitch_ref_deg, yaw_ref_deg;
    float pitch_rate_dps, yaw_rate_dps;
    float pitch_trend_dps, yaw_trend_dps;
    float height_m;
    uint8_t pred_valid;
    float tgo;
    float pitch_pred_rate, yaw_pred_rate;
    uint8_t event_fault_p, event_fault_y;
    uint8_t chatter_p, chatter_y;

    memset(&out, 0, sizeof(out));
    if (s_rcs.initialized == 0U) rcs_reset();
    s_rcs.sample_count++;

    if ((tfl_finite(target_pitch_rad) == 0U) ||
        (tfl_finite(target_yaw_rad) == 0U) ||
        (tfl_finite(pitch_rad) == 0U) || (tfl_finite(yaw_rad) == 0U) ||
        (tfl_finite(pitch_rate_rad_s) == 0U) ||
        (tfl_finite(yaw_rate_rad_s) == 0U) ||
        (tfl_finite(z_cg) == 0U) || (tfl_finite(vz) == 0U))
    {
        s_rcs.hard_fault = 1U;
        s_rcs.fault_code = 1U;
    }

    if (s_rcs.hard_fault != 0U)
    {
        rcs_force_safe_axis(&s_rcs.pitch);
        rcs_force_safe_axis(&s_rcs.yaw);
        out.fault = s_rcs.fault_code;
        return out;
    }

    if (hover_state >= 4U) s_rcs.landed_latch = 1U;
    if (s_rcs.landed_latch != 0U)
    {
        rcs_force_safe_axis(&s_rcs.pitch);
        rcs_force_safe_axis(&s_rcs.yaw);
        return out;
    }

    pitch_deg = -pitch_rad * TFL_RAD2DEG;
    yaw_deg = -yaw_rad * TFL_RAD2DEG;
    pitch_ref_deg = tfl_clampf(-target_pitch_rad * TFL_RAD2DEG, -4.8f, 4.8f);
    yaw_ref_deg = tfl_clampf(-target_yaw_rad * TFL_RAD2DEG, -5.5f, 5.5f);

    rcs_update_rates(&s_rcs.pitch, pitch_deg,
                     -pitch_rate_rad_s * TFL_RAD2DEG,
                     &pitch_rate_dps, &pitch_trend_dps);
    rcs_update_rates(&s_rcs.yaw, yaw_deg,
                     -yaw_rate_rad_s * TFL_RAD2DEG,
                     &yaw_rate_dps, &yaw_trend_dps);

    if ((tfl_absf(pitch_rate_dps) < 0.50f) &&
        (tfl_absf(pitch_trend_dps) >= 0.80f))
        pitch_rate_dps = pitch_trend_dps;
    if ((tfl_absf(yaw_rate_dps) < 0.50f) &&
        (tfl_absf(yaw_trend_dps) >= 0.80f))
        yaw_rate_dps = yaw_trend_dps;

    height_m = fmaxf(z_cg - TFL_Z_TOUCH_M, 0.0f);
    pred_valid =
        ((height_m >= 0.10f) && (height_m <= 20.0f) && (vz < -0.10f)) ? 1U : 0U;
    tgo = (pred_valid != 0U) ? fminf(height_m / (-vz), 8.0f) : 0.0f;

    if (tfl_absf(pitch_rate_dps) >= 7.0f) pitch_pred_rate = pitch_rate_dps;
    else if (tfl_absf(pitch_trend_dps) >= 1.5f) pitch_pred_rate = pitch_trend_dps;
    else pitch_pred_rate = 0.0f;

    if (tfl_absf(yaw_rate_dps) >= 7.0f) yaw_pred_rate = yaw_rate_dps;
    else if (tfl_absf(yaw_trend_dps) >= 1.5f) yaw_pred_rate = yaw_trend_dps;
    else yaw_pred_rate = 0.0f;

    out.pred_pitch_deg = (pred_valid != 0U) ?
        tfl_clampf(pitch_deg + pitch_pred_rate * tgo, -180.0f, 180.0f) : pitch_deg;
    out.pred_yaw_deg = (pred_valid != 0U) ?
        tfl_clampf(yaw_deg + yaw_pred_rate * tgo, -180.0f, 180.0f) : yaw_deg;
    out.time_to_ground_s = tgo;

    rcs_age_counters(&s_rcs.pitch, 100U);
    rcs_age_counters(&s_rcs.yaw, 100U);

    event_fault_p = rcs_select_axis(&s_rcs.pitch, pitch_deg, pitch_rate_dps,
                                    out.pred_pitch_deg, pred_valid);
    event_fault_y = rcs_select_axis(&s_rcs.yaw, yaw_deg, yaw_rate_dps,
                                    out.pred_yaw_deg, pred_valid);
    if ((event_fault_p != 0U) || (event_fault_y != 0U))
    {
        s_rcs.hard_fault = 1U;
        s_rcs.fault_code = 3U;
        rcs_force_safe_axis(&s_rcs.pitch);
        rcs_force_safe_axis(&s_rcs.yaw);
    }

    if (s_rcs.hard_fault == 0U)
    {
        rcs_apply_reference_tracking(&s_rcs.pitch, pitch_deg,
                                     pitch_rate_dps, pitch_ref_deg);
        rcs_apply_reference_tracking(&s_rcs.yaw, yaw_deg,
                                     yaw_rate_dps, yaw_ref_deg);
    }

    chatter_p = rcs_process_output(&s_rcs.pitch);
    chatter_y = rcs_process_output(&s_rcs.yaw);
    if ((chatter_p != 0U) || (chatter_y != 0U))
    {
        s_rcs.hard_fault = 1U;
        s_rcs.fault_code = 2U;
        rcs_force_safe_axis(&s_rcs.pitch);
        rcs_force_safe_axis(&s_rcs.yaw);
    }

    out.v1 = (s_rcs.pitch.applied == -1) ? 1U : 0U;
    out.v3 = (s_rcs.pitch.applied == 1) ? 1U : 0U;
    out.v5 = (s_rcs.yaw.applied == -1) ? 1U : 0U;
    out.v7 = (s_rcs.yaw.applied == 1) ? 1U : 0U;
    out.upitch = s_rcs.pitch.applied;
    out.uyaw = s_rcs.yaw.applied;
    out.mode_pitch = s_rcs.pitch.mode;
    out.mode_yaw = s_rcs.yaw.mode;
    out.fault = s_rcs.fault_code;
    return out;
}

/* -------------------------------------------------------------------------- */
/* Synthetic 10 s parity/transition profile                                   */
/* -------------------------------------------------------------------------- */

#if (APP_R8R19_FLIGHT_LOGIC_SYNTHETIC_TEST != 0U)
static void build_synthetic_input(uint32_t step, TaragayFlightLogicInput_t *in)
{
    float t = (float)step * TFL_TS_S;
    memset(in, 0, sizeof(*in));
    in->mass_kg = 27.5f;
    in->turns_actual = 0.0f;
    in->health_ok = 1U;
    in->release_event = 1U;

    if (t < 1.0f)
    {
        in->z_cg_m = 2.0f + 2.8f * t;
        in->vz_mps = 1.2f;
        in->x_m = 0.8f;
        in->y_m = -0.6f;
        in->vx_mps = 0.15f;
        in->vy_mps = -0.10f;
        in->pitch_rad = 2.5f * TFL_DEG2RAD;
        in->yaw_rad = -2.0f * TFL_DEG2RAD;
    }
    else if (t < 9.20f)
    {
        float h = t - 1.0f;
        in->z_cg_m = 5.00f + 0.03f * sinf(h * 2.0f);
        in->vz_mps = 0.03f * cosf(h * 2.0f);
        in->x_m = 0.45f * cosf(h * 0.7f);
        in->y_m = 0.35f * sinf(h * 0.8f);
        in->vx_mps = -0.315f * sinf(h * 0.7f);
        in->vy_mps = 0.28f * cosf(h * 0.8f);
        in->pitch_rad = 3.0f * TFL_DEG2RAD * sinf(h * 2.4f);
        in->yaw_rad = 2.5f * TFL_DEG2RAD * cosf(h * 2.1f);
        in->pitch_rate_rad_s = 7.2f * TFL_DEG2RAD * cosf(h * 2.4f);
        in->yaw_rate_rad_s = -5.25f * TFL_DEG2RAD * sinf(h * 2.1f);
    }
    else if (t < 10.00f)
    {
        float q = (t - 9.20f) / 0.80f;
        in->z_cg_m = 5.00f + q * (0.70f - 5.00f);
        in->vz_mps = -0.60f;
        in->x_m = 0.25f * (1.0f - q);
        in->y_m = -0.20f * (1.0f - q);
        in->vx_mps = -0.10f;
        in->vy_mps = 0.08f;
        in->pitch_rad = 0.60f * TFL_DEG2RAD;
        in->yaw_rad = -0.50f * TFL_DEG2RAD;
    }
    else
    {
        in->z_cg_m = 0.421f;
        in->vz_mps = 0.05f;
        in->pitch_rad = 0.10f * TFL_DEG2RAD;
        in->yaw_rad = -0.10f * TFL_DEG2RAD;
    }
}
#endif

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void TaragayFlightLogic_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    hover_reset();
    rcs_reset();
    s_divider_200hz = 0U;
    s_synthetic_step = 0U;
    s_prev_rcs_request_mask = 0U;
    mount_cal_reset();
    s_status.initialized = 1U;
#if (APP_P112R12R8R32_REAL_FLIGHT_LOGIC_PHYSICAL_NEEDLE_REV != 0U)
    /* R8R32: flight logic owns the physical main-needle command path.
     * RCS remains physically isolated in this revision. */
    s_status.compute_only = 0U;
#else
    s_status.compute_only = 1U;
#endif
    s_status.synthetic_input_active =
        (APP_R8R19_FLIGHT_LOGIC_SYNTHETIC_TEST != 0U) ? 1U : 0U;
    s_status.real_input_active =
        (APP_R8R19_FLIGHT_LOGIC_SYNTHETIC_TEST == 0U) ? 1U : 0U;
}

void TaragayFlightLogic_Reset(void)
{
    TaragayFlightLogic_Init();
}

void TaragayFlightLogic_HoldSafe(void)
{
    s_status.valve_cmd = 0.0f;
    s_status.valve_u_cmd = 0.0f;
    s_status.valve_actuator_cmd = 0.0f;
    s_status.valve_turns_target = 0.0f;
    s_status.rcs_v1 = 0U;
    s_status.rcs_v3 = 0U;
    s_status.rcs_v5 = 0U;
    s_status.rcs_v7 = 0U;
    s_status.rcs_requested_mask = 0U;
    s_status.rcs_applied_mask = 0U;
    s_prev_rcs_request_mask = 0U;
}

void TaragayFlightLogic_Step100Hz(const TaragayFlightLogicInput_t *input)
{
    HoverOutput_t hover;
    RCSOutput_t rcs;
    uint8_t rcs_phase = 0U;
    uint8_t legacy_mission = (uint8_t)TARAGAY_MISSION_ARM;
    float target_pitch_rad = 0.0f;
    float target_yaw_rad = 0.0f;

    if ((input == 0) || (tfl_finite(input->x_m) == 0U) ||
        (tfl_finite(input->y_m) == 0U) || (tfl_finite(input->vx_mps) == 0U) ||
        (tfl_finite(input->vy_mps) == 0U) || (tfl_finite(input->z_cg_m) == 0U) ||
        (tfl_finite(input->vz_mps) == 0U) ||
        (tfl_finite(input->pitch_rad) == 0U) ||
        (tfl_finite(input->yaw_rad) == 0U) ||
        (tfl_finite(input->pitch_rate_rad_s) == 0U) ||
        (tfl_finite(input->yaw_rate_rad_s) == 0U) ||
        (tfl_finite(input->turns_actual) == 0U))
    {
        s_status.invalid_input_count++;
        s_status.input_valid = 0U;
        if (s_status.input_reject_reason == 0U)
            s_status.input_reject_reason = 10U; /* non-finite/direct-step input */
        TaragayFlightLogic_HoldSafe();
        return;
    }

    hover = hover_step(input->z_cg_m, input->vz_mps, input->turns_actual,
                       input->health_ok, input->release_event);
    if (hover.state == 4U) { rcs_phase = 1U; legacy_mission = (uint8_t)TARAGAY_MISSION_ASCENT; }
    else if ((hover.state >= 5U) && (hover.state <= 7U)) { rcs_phase = 2U; legacy_mission = (uint8_t)TARAGAY_MISSION_HOVER; }
    else if (hover.state == 8U) { rcs_phase = 4U; legacy_mission = (uint8_t)TARAGAY_MISSION_HOVER; }
    else { rcs_phase = 0U; legacy_mission = (uint8_t)TARAGAY_MISSION_ARM; }
#if (APP_P112R12R8R24_AXIS_SIGN_DIAGNOSTIC_REV != 0U)
    /* R8R24: absolute X/Y is not currently aided.  Do not let drifting
     * horizontal states bias the attitude sign test.  With hpos invalid,
     * RCS stabilizes to a zero pitch/yaw reference only. */
    if (s_status.horizontal_position_valid != 0U)
    {
        horizontal_step(input->x_m, input->y_m, input->vx_mps, input->vy_mps,
                        input->z_cg_m, rcs_phase,
                        &target_pitch_rad, &target_yaw_rad);
        s_status.horizontal_target_gated = 0U;
    }
    else
    {
        target_pitch_rad = 0.0f;
        target_yaw_rad = 0.0f;
        s_status.horizontal_target_gated = 1U;
    }
#else
    horizontal_step(input->x_m, input->y_m, input->vx_mps, input->vy_mps,
                    input->z_cg_m, rcs_phase,
                    &target_pitch_rad, &target_yaw_rad);
    s_status.horizontal_target_gated = 0U;
#endif
    rcs = rcs_step(target_pitch_rad, target_yaw_rad,
                   input->pitch_rad, input->yaw_rad,
                   input->pitch_rate_rad_s, input->yaw_rate_rad_s,
                   input->z_cg_m, input->vz_mps, rcs_phase);

    /* RCS PE9 hard-start rule.
     * Before a valid PE9 separation latch there is NO RCS request, including
     * emergency/preflight pulses. Reset the RCS runtime every pre-release step
     * so a pulse/event cannot be preloaded and leak across the PE9 edge.
     * Immediately after PE9 the normal V7.13.4 controller is allowed to run;
     * SAFE remains all-off. The app-level R8R33 physical gate independently
     * requires the same PE9 flight_active latch. */
    if (s_hover.release_latch == 0U)
    {
        rcs_reset();
        memset(&rcs, 0, sizeof(rcs));
        target_pitch_rad = 0.0f;
        target_yaw_rad = 0.0f;
        s_status.horizontal_target_gated = 1U;
    }
    else if (hover.state >= 8U)
    {
        rcs.v1 = 0U; rcs.v3 = 0U; rcs.v5 = 0U; rcs.v7 = 0U;
    }

    s_status.mission_state = legacy_mission;
    s_status.vertical_state = hover.state;
    s_status.thrust_shortage = hover.thrust_shortage;
    s_status.z_reference_m = hover.z_reference_m;
    s_status.hover_best_s = hover.hover_best_s;
    s_status.target_force_n = hover.target_force_n;
    s_status.valve_cmd = hover.valve_cmd;
    s_status.valve_u_cmd = hover.valve_cmd;
    s_status.valve_actuator_cmd = hover.actuator_cmd;
    s_status.valve_turns_target = hover.turns_target;
    s_status.valve_turns_actual = input->turns_actual;
    s_status.valve_u_actual = hover.u_actual;
    s_status.a_filt_mps2 = hover.a_filt;
    s_status.a_cmd_mps2 = hover.a_cmd;
    s_status.b_hat_mps2 = hover.b_hat;
    s_status.b_auth_mps2 = hover.b_auth;
    s_status.authority_ratio = hover.authority_ratio;
    s_status.u_hover_est = hover.u_hover_est;
    s_status.d_stop_pred_m = hover.d_stop_pred;
    s_status.v_ref_mps = hover.v_ref;
    s_status.release_latched = s_hover.release_latch;
    s_status.vertical_safe_latched = s_hover.safe_latch;
    s_status.authority_latched = s_hover.authority_latch;
    s_status.thrust_id_active = s_hover.identify_active;
    s_status.thrust_id_complete = s_hover.identify_complete;
    s_status.thrust_id_probe_u = s_hover.id_probe_u;
    s_status.release_z0_m = s_hover.z0_ref;
    s_status.target_pitch_rad = target_pitch_rad;
    s_status.target_yaw_rad = target_yaw_rad;
    s_status.rcs_fault = rcs.fault;
    s_status.rcs_v1 = rcs.v1;
    s_status.rcs_v3 = rcs.v3;
    s_status.rcs_v5 = rcs.v5;
    s_status.rcs_v7 = rcs.v7;
    s_status.rcs_pitch_mode = rcs.mode_pitch;
    s_status.rcs_yaw_mode = rcs.mode_yaw;
    s_status.predicted_pitch_deg = rcs.pred_pitch_deg;
    s_status.predicted_yaw_deg = rcs.pred_yaw_deg;
    s_status.time_to_ground_s = rcs.time_to_ground_s;
    s_status.rcs_requested_mask =
        (uint8_t)((rcs.v1 ? 0x01U : 0U) |
                  (rcs.v3 ? 0x02U : 0U) |
                  (rcs.v5 ? 0x04U : 0U) |
                  (rcs.v7 ? 0x08U : 0U));
    /* R8R24: latch rising-edge request events so short 40 ms valve requests
     * remain visible even when UART snapshots are sparse/deferred. */
    if (((s_status.rcs_requested_mask & 0x01U) != 0U) && ((s_prev_rcs_request_mask & 0x01U) == 0U))
        s_status.rcs_v1_event_count++;
    if (((s_status.rcs_requested_mask & 0x02U) != 0U) && ((s_prev_rcs_request_mask & 0x02U) == 0U))
        s_status.rcs_v3_event_count++;
    if (((s_status.rcs_requested_mask & 0x04U) != 0U) && ((s_prev_rcs_request_mask & 0x04U) == 0U))
        s_status.rcs_v5_event_count++;
    if (((s_status.rcs_requested_mask & 0x08U) != 0U) && ((s_prev_rcs_request_mask & 0x08U) == 0U))
        s_status.rcs_v7_event_count++;
    s_prev_rcs_request_mask = s_status.rcs_requested_mask;
#if ((APP_P112R12R8R25_RCS_RELAY_AXIS_BENCH_REV != 0U) || \
     (APP_P112R12R8R26_IMU_ROCKET_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R27_THREE_POSE_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R28_FIVE_POSE_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R30_FIXED_IMU_ROCKET_FRAME_REV != 0U))
    /* R8R25..R8R30 pressureless relay bench: report the last actually-applied relay mask.
     * This is one control step behind the just-computed request by construction. */
    s_status.rcs_applied_mask = SolenoidOutput_GetAppliedMask();
#else
    /* R8R19/R8R24 are compute-only: applied physical mask is intentionally zero. */
    s_status.rcs_applied_mask = 0U;
#endif

    s_status.input_z_cg_m = input->z_cg_m;
    s_status.input_vz_mps = input->vz_mps;
    s_status.input_x_m = input->x_m;
    s_status.input_y_m = input->y_m;
    s_status.input_vx_mps = input->vx_mps;
    s_status.input_vy_mps = input->vy_mps;
    s_status.input_pitch_deg = input->pitch_rad * TFL_RAD2DEG;
    s_status.input_yaw_deg = input->yaw_rad * TFL_RAD2DEG;
    s_status.input_pitch_rate_dps = input->pitch_rate_rad_s * TFL_RAD2DEG;
    s_status.input_yaw_rate_dps = input->yaw_rate_rad_s * TFL_RAD2DEG;
    s_status.input_valid = 1U;
    s_status.input_reject_reason = 0U;
    s_status.step_count++;
    s_status.logic_elapsed_ms = s_status.step_count * 10UL;
}

void TaragayFlightLogic_Service200Hz(void)
{
    TaragayFlightLogicInput_t in;

    s_divider_200hz++;
    if (s_divider_200hz < 2U) return;
    s_divider_200hz = 0U;

#if (APP_R8R19_FLIGHT_LOGIC_SYNTHETIC_TEST != 0U)
    build_synthetic_input(s_synthetic_step, &in);
    TaragayFlightLogic_Step100Hz(&in);
    s_synthetic_step++;
    s_status.logic_elapsed_ms = s_synthetic_step * 10UL;
#else
    {
        const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
        const SensorData_t *sensor = SensorManager_GetDataPtr();
        uint32_t now_us = micros();
        uint32_t eskf_age_us = 0xFFFFFFFFUL;
        uint32_t imu_age_us = 0xFFFFFFFFUL;
        float height_agl;

        s_status.real_input_active = 1U;
        s_status.synthetic_input_active = 0U;
        s_status.input_valid = 0U;
        s_status.input_reject_reason = 0U;

        if ((eskf == 0) || (sensor == 0))
        {
            s_status.input_reject_reason = 1U; /* source pointer unavailable */
            s_status.invalid_input_count++;
            TaragayFlightLogic_HoldSafe();
            return;
        }

        s_status.horizontal_position_valid = eskf->horizontal_position_valid;
        s_status.vertical_position_valid = eskf->vertical_position_valid;
        s_status.eskf_origin_zeroed = eskf->origin_zeroed;
        s_status.eskf_output_inhibited = eskf->output_inhibited;

        if (eskf->last_public_output_timestamp_us != 0UL)
            eskf_age_us = (uint32_t)(now_us - eskf->last_public_output_timestamp_us);
        if (sensor->imu_sample_timestamp_us != 0UL)
            imu_age_us = (uint32_t)(now_us - sensor->imu_sample_timestamp_us);
        s_status.input_eskf_age_ms =
            (eskf_age_us == 0xFFFFFFFFUL) ? 0xFFFFFFFFUL : (eskf_age_us / 1000UL);
        s_status.input_imu_age_ms =
            (imu_age_us == 0xFFFFFFFFUL) ? 0xFFFFFFFFUL : (imu_age_us / 1000UL);

        if ((eskf->initialized == 0U) || (eskf->healthy == 0U))
            s_status.input_reject_reason = 2U;
        else if (eskf->vertical_position_valid == 0U)
            s_status.input_reject_reason = 3U;
        else if (eskf->output_inhibited != 0U)
            s_status.input_reject_reason = 4U;
        else if (eskf->covariance_integrity_ok == 0U)
            s_status.input_reject_reason = 5U;
        else if (sensor->imu_valid == 0U)
            s_status.input_reject_reason = 6U;
        else if ((eskf_age_us == 0xFFFFFFFFUL) ||
                 (eskf_age_us > APP_R8R22_MAX_ESKF_AGE_US))
            s_status.input_reject_reason = 7U;
        else if ((imu_age_us == 0xFFFFFFFFUL) ||
                 (imu_age_us > APP_R8R22_MAX_IMU_AGE_US))
            s_status.input_reject_reason = 8U;
        else if (eskf->origin_zeroed == 0U)
            s_status.input_reject_reason = 9U;

        if (s_status.input_reject_reason != 0U)
        {
            s_status.invalid_input_count++;
            TaragayFlightLogic_HoldSafe();
            return;
        }

        height_agl = eskf->lidar_reference_m + eskf->position_z_m;
        if (height_agl < 0.0f) height_agl = 0.0f;
        memset(&in, 0, sizeof(in));
        in.x_m = eskf->position_x_m;
        in.y_m = eskf->position_y_m;
        in.vx_mps = eskf->velocity_x_mps;
        in.vy_mps = eskf->velocity_y_mps;
        in.z_cg_m = TFL_Z_TOUCH_M + height_agl;
        in.vz_mps = eskf->velocity_z_mps;
#if (APP_P112R12R8R30_FIXED_IMU_ROCKET_FRAME_REV != 0U)
        /* R8R30: apply the accepted R8R29 matrix immediately. There is no
         * runtime pose-capture gate; invalid fixed-transform state is treated
         * as a mount-calibration fault and holds every control output safe. */
        if ((s_mount_cal.valid == 0U) || (s_mount_cal.fault != 0U) ||
            (s_mount_cal.phase != APP_R8R30_FIXED_CAL_PHASE) ||
            (mount_cal_correct_attitude_rates(sensor, eskf,
                                              &in.pitch_rad, &in.yaw_rad,
                                              &in.pitch_rate_rad_s,
                                              &in.yaw_rate_rad_s) == 0U))
        {
            s_status.input_reject_reason = 12U;
            TaragayFlightLogic_HoldSafe();
            return;
        }
#elif (APP_P112R12R8R28_FIVE_POSE_FRAME_CAL_REV != 0U)
        /* R8R28/R8R29 legacy calibration path retained for provenance only.
         * It is compile-time bypassed by R8R30. */
        if (mount_cal_step(sensor, eskf) == 0U)
        {
            s_status.input_reject_reason =
                (s_mount_cal.phase == 11U) ? 12U : 11U;
            TaragayFlightLogic_HoldSafe();
            return;
        }
        if (mount_cal_correct_attitude_rates(sensor, eskf,
                                             &in.pitch_rad, &in.yaw_rad,
                                             &in.pitch_rate_rad_s,
                                             &in.yaw_rate_rad_s) == 0U)
        {
            s_status.input_reject_reason = 12U;
            TaragayFlightLogic_HoldSafe();
            return;
        }
#else
        /* R8R24 fallback: direct ESKF roll/pitch source mapping. */
        in.pitch_rad = eskf->roll_deg * TFL_DEG2RAD;
        in.yaw_rad = eskf->pitch_deg * TFL_DEG2RAD;
        in.pitch_rate_rad_s =
            (sensor->gyro_x_filtered_dps - eskf->gyro_bias_x_dps) * TFL_DEG2RAD;
        in.yaw_rate_rad_s =
            (sensor->gyro_y_filtered_dps - eskf->gyro_bias_y_dps) * TFL_DEG2RAD;
#endif
        in.mass_kg = APP_GENERATED_FC_MODEL_MASS_KG; /* compatibility only */
        in.turns_actual = tfl_needle_turns_actual_fast();
        in.health_ok = 1U; /* all required ESKF/IMU gates above passed */
        in.release_event = (PreflightTrigger_IsFlightActive() != 0U) ? 1U : 0U;
        TaragayFlightLogic_Step100Hz(&in);
    }
#endif
}

TaragayFlightLogicStatus_t TaragayFlightLogic_GetStatus(void)
{
    TaragayFlightLogicStatus_t copy = s_status;
#if (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV != 0U)
    /* R8R35: telemetry must describe the physical relay state, not the
     * pre-handoff value sampled inside the control step. */
    copy.rcs_applied_mask = SolenoidOutput_GetAppliedMask();
#endif
    return copy;
}
