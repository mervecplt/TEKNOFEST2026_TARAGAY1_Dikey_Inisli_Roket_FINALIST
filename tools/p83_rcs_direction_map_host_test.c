#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "Services/SolenoidOutput/rcs_direction_map.h"

static void expect(uint8_t logical, uint8_t expected, const char *name)
{
    const uint8_t actual = RcsDirectionMap_LogicalToPhysical(logical);
    if (actual != expected)
    {
        fprintf(stderr, "FAIL %s logical=0x%02X expected=0x%02X actual=0x%02X\n",
                name, logical, expected, actual);
        exit(1);
    }
}

int main(void)
{
    expect(RCS_LOGICAL_X_POS_BIT, RCS_PHYSICAL_IN2_BIT, "X+ -> IN2");
    expect(RCS_LOGICAL_X_NEG_BIT, RCS_PHYSICAL_IN1_BIT, "X- -> IN1");
    expect(RCS_LOGICAL_Y_POS_BIT, RCS_PHYSICAL_IN4_BIT, "Y+ -> IN4");
    expect(RCS_LOGICAL_Y_NEG_BIT, RCS_PHYSICAL_IN3_BIT, "Y- -> IN3");
    expect(0U, 0U, "SAFE");
    expect((uint8_t)(RCS_LOGICAL_X_POS_BIT | RCS_LOGICAL_Y_NEG_BIT),
           (uint8_t)(RCS_PHYSICAL_IN2_BIT | RCS_PHYSICAL_IN3_BIT),
           "combined X+ Y-");

    puts("PASS: P83 RCS logical->physical direction map");
    return 0;
}
