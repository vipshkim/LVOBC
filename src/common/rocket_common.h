#ifndef ROCKET_COMMON_H
#define ROCKET_COMMON_H

#include <stddef.h>
#include <stdint.h>

double rocket_decode_param_number(uint8_t type, float value);
float rocket_encode_param_value(uint8_t type, double value);
void rocket_format_param_value(uint8_t type, float value, char *out, size_t out_len);
int rocket_param_value_matches(uint8_t type, float actual, double expected, double eps);

#endif
