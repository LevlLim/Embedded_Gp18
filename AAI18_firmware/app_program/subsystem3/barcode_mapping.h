/**
 * @file barcode_mapping.h
 * @brief Replaceable alphabet policy; no sensing or motor access.
 */
#ifndef BARCODE_MAPPING_H
#define BARCODE_MAPPING_H
#include "../msg_defs.h"

/** True only when all 26 configured commands are L, R, S or U. */
bool barcode_mapping_is_valid(void);

/** Map an uppercase letter. Bad configuration/nonletters return INVALID. */
msg_barcode_cmd_t barcode_mapping_direction(uint8_t letter);

/** Printable direction name, used only when reporting a complete decode. */
char const * barcode_mapping_name(msg_barcode_cmd_t direction);
#endif /* BARCODE_MAPPING_H */
