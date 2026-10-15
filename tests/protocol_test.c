#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "protocol.h"

int main(void)
{
    static const uint8_t start[] = {'S', 'T', 'A', 'R', 'T'};
    static const uint8_t stop[] = {'S', 'T', 'O', 'P'};
    static const uint8_t unsafe[] = {'S', 'T', 'O', 'P', 'X'};
    buddy1_telemetry_t sample = {0};
    char json[384];
    char too_small[5] = {'x', 'x', 'x', 'x', 'z'};
    uint32_t length;

    assert(buddy1_parse_command(start, sizeof(start)) ==
           BUDDY1_COMMAND_START);
    assert(buddy1_parse_command(stop, sizeof(stop)) ==
           BUDDY1_COMMAND_STOP);
    assert(buddy1_parse_command(unsafe, sizeof(unsafe)) ==
           BUDDY1_COMMAND_INVALID);
    assert(buddy1_parse_command(stop, 3U) == BUDDY1_COMMAND_INVALID);
    assert(buddy1_parse_command(NULL, 4U) == BUDDY1_COMMAND_INVALID);

    sample.uptime_ms = UINT32_MAX;
    sample.speed_left_mm_s = INT32_MIN;
    sample.speed_right_mm_s = INT32_MAX;
    sample.valid_fields = BUDDY1_VALID_MOTION;
    length = buddy1_format_telemetry(json, sizeof(json), &sample);
    assert(length > 0U);
    assert(length == strlen(json));
    assert(strstr(json, "\"ms\":4294967295") != NULL);
    assert(strstr(json, "\"speed_l\":-2147483648") != NULL);
    assert(strstr(json, "\"speed_r\":2147483647") != NULL);
    assert(strstr(json, "\"valid\":1") != NULL);
    assert(buddy1_format_telemetry(too_small, 4U, &sample) == 0U);
    assert(too_small[0] == '\0');
    assert(too_small[4] == 'z');

    assert(buddy1_format_u32(json, sizeof(json), 0U) == 1U);
    assert(strcmp(json, "0") == 0);
    return 0;
}
