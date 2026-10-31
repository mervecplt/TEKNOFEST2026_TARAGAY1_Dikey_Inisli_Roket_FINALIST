#ifndef TARAGAY_CONTROL_TUNING_H
#define TARAGAY_CONTROL_TUNING_H

#include "Common/app_config.h"

/* TARAGAY-T1 R14p3b/R14p4 STM32 tuning surface.
 *
 * IMPORTANT: Normal tuning changes should be made in THIS FILE only.
 * The controller intentionally does not use tank pressure, model mass or a
 * direct thrust sensor. It adapts thrust effectiveness from vertical motion
 * and the measured needle-valve position.
 *
 * Hardware note (2026-09-08 correction): physical needle command travel is
 * limited to 3.0 turns. The full measured 0..3 turn Cv map is used; command
 * conversion maps normalized flow -> target turns -> normalized actuator travel.
 * The physical turn limit itself comes from app_config.h so it is not duplicated.
 */

#define TGY_CTRL_PROFILE_NAME             "R14p3b_R14p4_STM32_3TURN_PE9_RCS"

#define TGY_CTRL_TS_S                     0.01f
#define TGY_CTRL_G                        9.80665f

/* Mission geometry */
#define TGY_CTRL_Z_TARGET_DELTA_M          3.00f
#define TGY_CTRL_Z_HARD_LIMIT_M            4.00f

/* Physical needle geometry */
#define TGY_CTRL_VALVE_MAX_TURNS           ((float)APP_R8R32_NEEDLE_MAX_TURNS)
#define TGY_CTRL_TURN_RATE_CLOSE_TPS       4.85f
#define TGY_CTRL_CV_MAX_HW                 1.82f
#define TGY_CTRL_PREPOSITION_TURN_TOL      0.05f
#define TGY_CTRL_PREPOSITION_CONFIRM_S     0.15f

/* Startup/self-check */
#define TGY_CTRL_INIT_HOLD_S               0.04f
#define TGY_CTRL_HEALTH_CONFIRM_S          0.20f

/* ASCENT */
#define TGY_CTRL_V_UP_MAX                  1.40f
#define TGY_CTRL_KZ_ASC                    0.45f
#define TGY_CTRL_KV_ASC                    2.80f
#define TGY_CTRL_A_ASC_UP                  3.00f
#define TGY_CTRL_A_ASC_DN                 -3.50f

/* Predictive braking */
#define TGY_CTRL_A_PRED_BRAKE              2.50f
#define TGY_CTRL_PRED_MARGIN               0.15f
#define TGY_CTRL_V_TARGET_MAX              0.25f

/* CAPTURE */
#define TGY_CTRL_KZ_CAPTURE                1.00f
#define TGY_CTRL_KV_CAPTURE                3.20f
#define TGY_CTRL_KI_CAPTURE                0.15f
#define TGY_CTRL_V_CAPTURE_MAX             1.30f
#define TGY_CTRL_A_CAPTURE_PROFILE         1.40f
#define TGY_CTRL_V_CAPTURE_DOWN_MAX        0.25f
#define TGY_CTRL_A_CAPTURE_UP              2.20f
#define TGY_CTRL_A_CAPTURE_DN             -3.00f

/* HOVER */
#define TGY_CTRL_KZ_HOVER                  1.20f
#define TGY_CTRL_KV_HOVER                  3.00f
#define TGY_CTRL_KI_HOVER                  0.20f
#define TGY_CTRL_V_HOVER_MAX               0.25f
#define TGY_CTRL_A_HOVER_UP                2.00f
#define TGY_CTRL_A_HOVER_DN               -2.50f
#define TGY_CTRL_IZ_MAX                    1.00f
#define TGY_CTRL_Z_HOVER_BAND              0.15f
#define TGY_CTRL_V_HOVER_BAND              0.20f
#define TGY_CTRL_HOVER_CONFIRM_S           0.30f

/* Effectiveness estimator */
#define TGY_CTRL_B_INIT                    14.50f
#define TGY_CTRL_B_MIN                      4.00f
#define TGY_CTRL_B_MAX                     30.00f
#define TGY_CTRL_A_FILTER_ALPHA             0.08f
#define TGY_CTRL_B_FILTER_ALPHA             0.015f
#define TGY_CTRL_EST_MIN_U                  0.20f

/* Fast authority estimator/supervisor */
#define TGY_CTRL_AUTH_ALPHA_DOWN            0.18f
#define TGY_CTRL_AUTH_ALPHA_UP              0.02f
#define TGY_CTRL_AUTH_RATIO_TRIP            1.00f
#define TGY_CTRL_AUTH_DEMAND_U              0.90f
#define TGY_CTRL_AUTH_U_SAT                 0.985f
#define TGY_CTRL_AUTH_A_NEG                -0.05f
#define TGY_CTRL_AUTH_CONFIRM_S             0.15f

/* Release-time thrust identification */
#define TGY_CTRL_ID_PROBE_START             0.30f
#define TGY_CTRL_ID_PROBE_STEP              0.05f
#define TGY_CTRL_ID_PROBE_U_MAX             0.90f
#define TGY_CTRL_ID_STEP_WAIT_S             0.10f
#define TGY_CTRL_ID_AIRBORNE_V              0.08f
#define TGY_CTRL_ID_AIRBORNE_DZ             0.03f
#define TGY_CTRL_ID_AIRBORNE_CONFIRM_S      0.03f
#define TGY_CTRL_ID_SETTLE_S                0.05f
#define TGY_CTRL_ID_SAMPLE_S                0.10f
#define TGY_CTRL_ID_AUTHORITY_MIN           1.03f
#define TGY_CTRL_ID_TIMEOUT_S               2.50f
#define TGY_CTRL_ID_U_MATCH_TOL             0.08f

/* Identification -> ASCENT soft handover */
#define TGY_CTRL_HANDOVER_RATE_UP            0.60f
#define TGY_CTRL_HANDOVER_DONE_TOL           0.005f

#endif
