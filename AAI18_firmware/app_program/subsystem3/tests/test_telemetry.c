/**
 * @file test_telemetry.c
 * @brief Binary letter reporting, guards and backwards-schema rejection.
 */
#include <stdio.h>
#include <string.h>
#include "../../msg_defs.h"

static unsigned int g_tests = 0U;
static unsigned int g_failed = 0U;

static void check(bool b_valid)
{
    g_tests++;
    if (!b_valid)
    {
        g_failed++;
    }
}

int main(void)
{
    msg_telemetry_t original = {0};
    msg_telemetry_t decoded = {0};
    uint8_t bytes[MSG_LEN_TELEMETRY] = {0U};
    uint8_t letter = 0U;
    uint8_t direction = 0U;
    uint32_t invalid = 0U;

    check(13U == sizeof(bytes));
    check(MSG_LEN_TELEMETRY == msg_pack_telemetry(&original, bytes,
                                                 sizeof(bytes)));
    check(msg_unpack_telemetry(bytes, sizeof(bytes), &decoded));
    check(0U == decoded.last_letter);
    for (letter = (uint8_t)'A'; letter <= (uint8_t)'Z'; letter++)
    {
        for (direction = (uint8_t)MSG_BC_STRAIGHT;
             direction <= (uint8_t)MSG_BC_U_TURN; direction++)
        {
            original.last_letter = letter;
            original.last_barcode = direction;
            original.fault_mask = 0x03FFU;
            original.encoder_delta = 65535U;
            check(MSG_LEN_TELEMETRY == msg_pack_telemetry(
                &original, bytes, sizeof(bytes)));
            check(msg_unpack_telemetry(bytes, sizeof(bytes), &decoded));
            check((letter == decoded.last_letter)
                  && (direction == decoded.last_barcode));
            check(0x03FFU == decoded.fault_mask);
            check(65535U == decoded.encoder_delta);
        }
    }
    for (invalid = 27U; invalid <= 255U; invalid++)
    {
        bytes[12] = (uint8_t)invalid;
        check(!msg_unpack_telemetry(bytes, sizeof(bytes), &decoded));
    }
    bytes[12] = 1U;
    check(!msg_unpack_telemetry(bytes, sizeof(bytes) - 1U, &decoded));
    bytes[0] = 2U;
    check(!msg_unpack_telemetry(bytes, sizeof(bytes), &decoded));
    original.last_letter = (uint8_t)'a';
    check(MSG_LEN_TELEMETRY == msg_pack_telemetry(&original, bytes,
                                                 sizeof(bytes)));
    check(0U == bytes[12]);
    check(msg_saturation_seen());
    printf("Telemetry regressions: %u/%u passed\n", g_tests - g_failed,
           g_tests);
    return ((0U == g_failed) ? 0 : 1);
}
