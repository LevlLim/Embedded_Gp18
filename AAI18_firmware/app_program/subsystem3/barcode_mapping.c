/**
 * @file barcode_mapping.c
 * @brief Temporary professor mapping, isolated from Code 39 decoding.
 */
#include "barcode_mapping.h"

msg_barcode_cmd_t barcode_mapping_direction(uint8_t letter)
{
    msg_barcode_cmd_t direction = MSG_BC_INVALID;

    if (((uint8_t)'A' <= letter) && (letter <= (uint8_t)'Z'))
    {
        direction = MSG_BC_STRAIGHT;
        switch (letter)
        {
            case (uint8_t)'A':
                direction = MSG_BC_LEFT;
                break;
            case (uint8_t)'B':
                direction = MSG_BC_RIGHT;
                break;
            case (uint8_t)'D':
                direction = MSG_BC_U_TURN;
                break;
            default:
                /* C and E..Z deliberately map to STRAIGHT. */
                break;
        }
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

