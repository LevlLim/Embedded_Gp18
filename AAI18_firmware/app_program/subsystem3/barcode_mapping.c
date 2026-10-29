 /**
 * @file barcode_mapping.c
 * @brief Temporary professor mapping, isolated from Code 39 decoding.
 */
#include "barcode_mapping.h"

/* EDIT THIS STRING when the professor supplies the mapping.
 * Positions correspond to A..Z. L=left, R=right, S=straight, U=U-turn.
 * Keep exactly 26 uppercase command characters, then rebuild/flash the UF2.
 */
static char const g_barcode_mapping[] =
    /* ABCDEFGHIJKLMNOPQRSTUVWXYZ */
      "LRSUSSSSSSSSSSSSSSSSSSSSSS";

_Static_assert(sizeof(g_barcode_mapping) == sizeof("ABCDEFGHIJKLMNOPQRSTUVWXYZ"),
               "Barcode mapping must contain exactly 26 commands for A..Z");

static msg_barcode_cmd_t command_direction(char command)
{
    msg_barcode_cmd_t direction = MSG_BC_INVALID;

    switch (command)
    {
        case 'L': direction = MSG_BC_LEFT; break;
        case 'R': direction = MSG_BC_RIGHT; break;
        case 'S': direction = MSG_BC_STRAIGHT; break;
        case 'U': direction = MSG_BC_U_TURN; break;
        default: break;
    }
    return (direction);
}

static bool mapping_text_is_valid(char const * p_text)
{
    uint8_t index = 0U;

    if (NULL == p_text)
    {
        return (false);
    }
    for (index = 0U; index < 26U; index++)
    {
        if (MSG_BC_INVALID == command_direction(p_text[index]))
        {
            return (false);
        }
    }
    return ('\0' == p_text[26]);
}

bool barcode_mapping_is_valid(void)
{
    return (mapping_text_is_valid(g_barcode_mapping));
}

msg_barcode_cmd_t barcode_mapping_direction(uint8_t letter)
{
    msg_barcode_cmd_t direction = MSG_BC_INVALID;

    if (((uint8_t)'A' <= letter) && (letter <= (uint8_t)'Z')
        && barcode_mapping_is_valid())
    {
        direction = command_direction(g_barcode_mapping[letter - (uint8_t)'A']);
    }
    return (direction);
}

char const * barcode_mapping_name(msg_barcode_cmd_t direction)
{
    char const * p_name = "invalid";

    switch (direction)
    {
        case MSG_BC_LEFT:
            p_name = "left";
            break;
        case MSG_BC_RIGHT:
            p_name = "right";
            break;
        case MSG_BC_STRAIGHT:
            p_name = "straight";
            break;
        case MSG_BC_U_TURN:
            p_name = "U-turn";
            break;
        default:
            break;
    }
    return (p_name);
}
