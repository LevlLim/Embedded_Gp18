/** Data-format validation, independent of the professor's letter assignments. */
#include <stdio.h>
#include "../barcode_mapping.c"

static unsigned int g_checks = 0U;
static unsigned int g_failures = 0U;

static void check(bool condition)
{
    g_checks++;
    if (!condition)
    {
        g_failures++;
        printf("Mapping failure at check %u\n", g_checks);
    }
}

int main(void)
{
    unsigned int value = 0U;

    check(MSG_BC_LEFT == command_direction('L'));
    check(MSG_BC_RIGHT == command_direction('R'));
    check(MSG_BC_STRAIGHT == command_direction('S'));
    check(MSG_BC_U_TURN == command_direction('U'));
    check(MSG_BC_INVALID == command_direction('X'));
    check(MSG_BC_INVALID == command_direction('l'));
    check(MSG_BC_INVALID == command_direction('\0'));
    check(barcode_mapping_is_valid());
    check(mapping_text_is_valid("LLLLLLLLLLLLLLLLLLLLLLLLLL"));
    check(mapping_text_is_valid("RRRRRRRRRRRRRRRRRRRRRRRRRR"));
    check(mapping_text_is_valid("SSSSSSSSSSSSSSSSSSSSSSSSSS"));
    check(mapping_text_is_valid("UUUUUUUUUUUUUUUUUUUUUUUUUU"));
    check(!mapping_text_is_valid(NULL));
    check(!mapping_text_is_valid(""));
    check(!mapping_text_is_valid("LRSU"));
    check(!mapping_text_is_valid("SSSSSSSSSSSSSSSSSSSSSSSSSSS"));
    check(!mapping_text_is_valid("XSSSSSSSSSSSSSSSSSSSSSSSSS"));
    check(!mapping_text_is_valid("SSSSSSSSSSSSSSSSSSSSSSSSSX"));
    for (value = 0U; value <= 255U; value++)
    {
        msg_barcode_cmd_t direction = barcode_mapping_direction((uint8_t)value);
        if ((value >= (unsigned int)'A') && (value <= (unsigned int)'Z'))
        {
            check(direction >= MSG_BC_STRAIGHT && direction <= MSG_BC_U_TURN);
        }
        else
        {
            check(MSG_BC_INVALID == direction);
        }
    }
    printf("Mapping regressions: %u/%u passed\n", g_checks - g_failures, g_checks);
    return ((0U == g_failures) ? 0 : 1);
}
