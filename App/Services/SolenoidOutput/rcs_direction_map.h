#ifndef APP_SERVICES_RCS_DIRECTION_MAP_H
#define APP_SERVICES_RCS_DIRECTION_MAP_H

#include <stdint.h>

/*
 * P83/R46 selective hardening: single authoritative logical->physical RCS map.
 *
 * Logical domain (controller / interlock / telemetry):
 *   bit0 = X+ correction request
 *   bit1 = X- correction request
 *   bit2 = Y+ correction request
 *   bit3 = Y- correction request
 *
 * Physical channel domain (relay inputs):
 *   bit0 = IN1 / PB15
 *   bit1 = IN2 / PE15
 *   bit2 = IN3 / PE11
 *   bit3 = IN4 / PE7
 *
 * Bench result 2026-09-09:
 *   IMU/ESKF/control signs are correct, while installed RCS hardware effect is
 *   opposite on both X and Y. Therefore the inversion belongs ONLY here.
 */
#define RCS_DIRECTION_MAP_REVISION              1U

#define RCS_LOGICAL_X_POS_BIT                 0x01U
#define RCS_LOGICAL_X_NEG_BIT                 0x02U
#define RCS_LOGICAL_Y_POS_BIT                 0x04U
#define RCS_LOGICAL_Y_NEG_BIT                 0x08U

#define RCS_PHYSICAL_IN1_BIT                  0x01U
#define RCS_PHYSICAL_IN2_BIT                  0x02U
#define RCS_PHYSICAL_IN3_BIT                  0x04U
#define RCS_PHYSICAL_IN4_BIT                  0x08U

static inline uint8_t RcsDirectionMap_LogicalToPhysical(uint8_t logical_mask)
{
    uint8_t physical_mask = 0U;

    /* X pair swap: logical X- -> IN1, logical X+ -> IN2. */
    if ((logical_mask & RCS_LOGICAL_X_NEG_BIT) != 0U)
    {
        physical_mask |= RCS_PHYSICAL_IN1_BIT;
    }
    if ((logical_mask & RCS_LOGICAL_X_POS_BIT) != 0U)
    {
        physical_mask |= RCS_PHYSICAL_IN2_BIT;
    }

    /* Y pair swap: logical Y- -> IN3, logical Y+ -> IN4. */
    if ((logical_mask & RCS_LOGICAL_Y_NEG_BIT) != 0U)
    {
        physical_mask |= RCS_PHYSICAL_IN3_BIT;
    }
    if ((logical_mask & RCS_LOGICAL_Y_POS_BIT) != 0U)
    {
        physical_mask |= RCS_PHYSICAL_IN4_BIT;
    }

    return physical_mask;
}

#endif
