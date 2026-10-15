#include "protocol.h"

#include <stddef.h>

typedef struct {
    char *buffer;
    uint32_t capacity;
    uint32_t length;
    int valid;
} text_writer_t;

static void append_character(text_writer_t *writer, char character)
{
    if (!writer->valid || (writer->length + 1U >= writer->capacity)) {
        writer->valid = 0;
        return;
    }
    writer->buffer[writer->length] = character;
    writer->length++;
}

static void append_text(text_writer_t *writer, const char *text)
{
    while (*text != '\0') {
        append_character(writer, *text);
        text++;
    }
}

static void append_unsigned(text_writer_t *writer, uint32_t number)
{
    char digits[10];
    uint32_t count = 0U;

    do {
        digits[count] = (char)('0' + (number % 10U));
        number /= 10U;
        count++;
    } while ((number != 0U) && (count < sizeof(digits)));
    while (count != 0U) {
        count--;
        append_character(writer, digits[count]);
    }
}

static void append_signed(text_writer_t *writer, int32_t number)
{
    uint32_t magnitude;

    if (number < 0) {
        append_character(writer, '-');
        magnitude = (uint32_t)(-(number + 1)) + 1U;
    } else {
        magnitude = (uint32_t)number;
    }
    append_unsigned(writer, magnitude);
}

static uint32_t finish_writer(text_writer_t *writer)
{
    if (!writer->valid) {
        writer->buffer[0] = '\0';
        return 0U;
    }
    writer->buffer[writer->length] = '\0';
    return writer->length;
}

enum buddy1_command buddy1_parse_command(const uint8_t *payload, uint32_t length)
{
    if (payload == NULL) {
        return BUDDY1_COMMAND_INVALID;
    }
    if ((length == 5U) && (payload[0] == 'S') && (payload[1] == 'T') &&
        (payload[2] == 'A') && (payload[3] == 'R') && (payload[4] == 'T')) {
        return BUDDY1_COMMAND_START;
    }
    if ((length == 4U) && (payload[0] == 'S') && (payload[1] == 'T') &&
        (payload[2] == 'O') && (payload[3] == 'P')) {
        return BUDDY1_COMMAND_STOP;
    }
    return BUDDY1_COMMAND_INVALID;
}

uint32_t buddy1_format_u32(char *buffer, uint32_t capacity, uint32_t value)
{
    text_writer_t writer = {buffer, capacity, 0U, 1};

    if ((buffer == NULL) || (capacity == 0U)) {
        return 0U;
    }
    append_unsigned(&writer, value);
    return finish_writer(&writer);
}

uint32_t buddy1_format_telemetry(char *buffer, uint32_t capacity,
                                 const buddy1_telemetry_t *sample)
{
    text_writer_t writer = {buffer, capacity, 0U, 1};

    if ((buffer == NULL) || (sample == NULL) || (capacity == 0U)) {
        return 0U;
    }
    append_text(&writer, "{\"ms\":");
    append_unsigned(&writer, sample->uptime_ms);
    append_text(&writer, ",\"seq\":");
    append_unsigned(&writer, sample->sequence);
    append_text(&writer, ",\"state\":");
    append_unsigned(&writer, sample->mission_state);
    append_text(&writer, ",\"valid\":");
    append_unsigned(&writer, sample->valid_fields);
    append_text(&writer, ",\"fault\":");
    append_unsigned(&writer, sample->fault_flags);
    append_text(&writer, ",\"speed_l\":");
    append_signed(&writer, sample->speed_left_mm_s);
    append_text(&writer, ",\"speed_r\":");
    append_signed(&writer, sample->speed_right_mm_s);
    append_text(&writer, ",\"enc_l\":");
    append_signed(&writer, sample->encoder_left);
    append_text(&writer, ",\"enc_r\":");
    append_signed(&writer, sample->encoder_right);
    append_text(&writer, ",\"line\":");
    append_signed(&writer, sample->line_error);
    append_text(&writer, ",\"barcode\":");
    append_signed(&writer, sample->barcode);
    append_text(&writer, ",\"hump_mm\":");
    append_signed(&writer, sample->hump_peak_mm);
    append_text(&writer, ",\"distance_mm\":");
    append_signed(&writer, sample->obstacle_distance_mm);
    append_text(&writer, ",\"width_mm\":");
    append_signed(&writer, sample->obstacle_width_mm);
    append_character(&writer, '}');
    return finish_writer(&writer);
}
