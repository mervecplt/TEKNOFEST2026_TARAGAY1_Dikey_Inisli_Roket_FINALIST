#ifndef TARAGAY_FLIGHT_LOGIC_H
#define TARAGAY_FLIGHT_LOGIC_H

#include <stdint.h>

typedef enum
{
    TARAGAY_MISSION_ARM = 0,
    TARAGAY_MISSION_ASCENT = 1,
    TARAGAY_MISSION_HOVER = 2,
    TARAGAY_MISSION_DESCENT = 3,
    TARAGAY_MISSION_TOUCHDOWN = 4
} TaragayMissionState_t;

typedef struct
{
    float x_m;
    float y_m;
    float vx_mps;
    float vy_mps;
    float z_cg_m;
    float vz_mps;
    float pitch_rad;
    float yaw_rad;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    float mass_kg; /* legacy compatibility; latest vertical controller ignores it */
    float turns_actual;
    uint8_t health_ok;
    uint8_t release_event;
} TaragayFlightLogicInput_t;

typedef struct
{
    uint8_t initialized;
    uint8_t compute_only;
    uint8_t synthetic_input_active;
    uint8_t real_input_active;
    uint8_t input_valid;
    uint8_t input_reject_reason;
    uint8_t horizontal_position_valid;
    uint8_t vertical_position_valid;
    uint8_t eskf_origin_zeroed;
    uint8_t eskf_output_inhibited;
    uint8_t mission_state;
    uint8_t thrust_shortage;
    uint8_t rcs_fault;
    uint8_t rcs_v1;
    uint8_t rcs_v3;
    uint8_t rcs_v5;
    uint8_t rcs_v7;
    uint8_t rcs_requested_mask;
    uint8_t rcs_applied_mask;
    uint8_t rcs_pitch_mode;
    uint8_t rcs_yaw_mode;
    uint8_t horizontal_target_gated;
    uint32_t rcs_v1_event_count;
    uint32_t rcs_v3_event_count;
    uint32_t rcs_v5_event_count;
    uint32_t rcs_v7_event_count;
    uint32_t step_count;
    uint32_t invalid_input_count;
    uint32_t logic_elapsed_ms;
    uint32_t input_eskf_age_ms;
    uint32_t input_imu_age_ms;
    float z_reference_m;
    float hover_best_s;
    float target_force_n;
    float valve_cmd;
    float target_pitch_rad;
    float target_yaw_rad;
    float predicted_pitch_deg;
    float predicted_yaw_deg;
    float time_to_ground_s;
    float input_z_cg_m;
    float input_vz_mps;
    float input_x_m;
    float input_y_m;
    float input_vx_mps;
    float input_vy_mps;
    float input_pitch_deg;
    float input_yaw_deg;
    float input_pitch_rate_dps;
    float input_yaw_rate_dps;

    /* R14p3b/R14p4 adaptive vertical controller diagnostics. Append-only so
     * existing telemetry consumers keep their legacy offsets/meaning. */
    uint8_t vertical_state;       /* 0 INIT,1 SELF_CHECK,2 PREPOSITION,3 READY,4 ASCENT,5 CAPTURE,6 HOVER,7 DEGRADED,8 SAFE */
    uint8_t release_latched;
    uint8_t vertical_safe_latched;
    uint8_t authority_latched;
    uint8_t thrust_id_active;
    uint8_t thrust_id_complete;
    float valve_u_cmd;            /* normalized controller flow/effectiveness request 0..1 */
    float valve_actuator_cmd;     /* physical 0..1 travel command for configured 3-turn needle */
    float valve_turns_target;
    float valve_turns_actual;
    float valve_u_actual;
    float a_filt_mps2;
    float a_cmd_mps2;
    float b_hat_mps2;
    float b_auth_mps2;
    float authority_ratio;
    float u_hover_est;
    float d_stop_pred_m;
    float v_ref_mps;
    float thrust_id_probe_u;
    float release_z0_m;

    /* R8R30 fixed IMU/sensor-frame -> physical rocket-frame transform.
     * Phase 12 means FIXED_MATRIX_LOADED. The matrix is row-major and maps
     * vectors expressed in the current SensorManager/ESKF body frame into the
     * physical rocket body frame. Legacy calibration fields are retained as
     * append-only telemetry; tilt/quality values are frozen R8R29 provenance. */
    uint8_t mount_cal_phase;
    uint8_t mount_cal_valid;
    uint8_t mount_cal_fault;
    uint16_t mount_cal_upright_samples;
    uint16_t mount_cal_tilt_samples;      /* +X capture samples */
    uint16_t mount_cal_y_tilt_samples;    /* +Y capture samples */
    uint16_t mount_cal_neg_x_samples;     /* -X capture samples */
    uint16_t mount_cal_neg_y_samples;     /* -Y capture samples */
    float mount_cal_tilt_deg;             /* +X tilt angle */
    float mount_cal_y_tilt_deg;           /* +Y tilt angle */
    float mount_cal_neg_x_tilt_deg;
    float mount_cal_neg_y_tilt_deg;
    float mount_cal_x_opposition;
    float mount_cal_y_opposition;
    float mount_cal_xy_angle_deg;
    float mount_cal_axis_agreement;
    float mount_cal_z_agreement;
    float mount_cal_ortho_error;
    float mount_cal_det;
    float mount_r00;
    float mount_r01;
    float mount_r02;
    float mount_r10;
    float mount_r11;
    float mount_r12;
    float mount_r20;
    float mount_r21;
    float mount_r22;
} TaragayFlightLogicStatus_t;

void TaragayFlightLogic_Init(void);
void TaragayFlightLogic_Reset(void);
void TaragayFlightLogic_Step100Hz(const TaragayFlightLogicInput_t *input);
void TaragayFlightLogic_Service200Hz(void);
void TaragayFlightLogic_HoldSafe(void);
TaragayFlightLogicStatus_t TaragayFlightLogic_GetStatus(void);

#endif
