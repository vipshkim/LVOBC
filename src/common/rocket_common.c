#include "rocket_common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <mavlink.h>

double rocket_decode_param_number(uint8_t type, float value) {
    mavlink_param_union_t u;
    memset(&u, 0, sizeof(u));
    u.type = type;
    u.param_float = value;

    switch (type) {
        case MAV_PARAM_TYPE_UINT8:  return (double)u.param_uint8;
        case MAV_PARAM_TYPE_INT8:   return (double)u.param_int8;
        case MAV_PARAM_TYPE_UINT16: return (double)u.param_uint16;
        case MAV_PARAM_TYPE_INT16:  return (double)u.param_int16;
        case MAV_PARAM_TYPE_UINT32: return (double)u.param_uint32;
        case MAV_PARAM_TYPE_INT32:  return (double)u.param_int32;
        case MAV_PARAM_TYPE_REAL32:
        case MAV_PARAM_TYPE_REAL64:
        default: return (double)value;
    }
}

float rocket_encode_param_value(uint8_t type, double value) {
    mavlink_param_union_t u;
    memset(&u, 0, sizeof(u));
    u.type = type;

    switch (type) {
        case MAV_PARAM_TYPE_UINT8:  u.param_uint8 = (uint8_t)llround(value); break;
        case MAV_PARAM_TYPE_INT8:   u.param_int8 = (int8_t)llround(value); break;
        case MAV_PARAM_TYPE_UINT16: u.param_uint16 = (uint16_t)llround(value); break;
        case MAV_PARAM_TYPE_INT16:  u.param_int16 = (int16_t)llround(value); break;
        case MAV_PARAM_TYPE_UINT32: u.param_uint32 = (uint32_t)llround(value); break;
        case MAV_PARAM_TYPE_INT32:  u.param_int32 = (int32_t)llround(value); break;
        case MAV_PARAM_TYPE_REAL32:
        case MAV_PARAM_TYPE_REAL64:
        default: u.param_float = (float)value; break;
    }
    return u.param_float;
}

void rocket_format_param_value(uint8_t type, float value, char *out, size_t out_len) {
    double d = rocket_decode_param_number(type, value);
    switch (type) {
        case MAV_PARAM_TYPE_UINT8:
            snprintf(out, out_len, "%u", (unsigned)llround(d));
            break;
        case MAV_PARAM_TYPE_INT8:
            snprintf(out, out_len, "%d", (int)llround(d));
            break;
        case MAV_PARAM_TYPE_UINT16:
            snprintf(out, out_len, "%u", (unsigned)llround(d));
            break;
        case MAV_PARAM_TYPE_INT16:
            snprintf(out, out_len, "%d", (int)llround(d));
            break;
        case MAV_PARAM_TYPE_UINT32:
            snprintf(out, out_len, "%u", (unsigned)llround(d));
            break;
        case MAV_PARAM_TYPE_INT32:
            snprintf(out, out_len, "%d", (int)llround(d));
            break;
        case MAV_PARAM_TYPE_REAL64:
            snprintf(out, out_len, "%.9g", d);
            break;
        case MAV_PARAM_TYPE_REAL32:
        default:
            snprintf(out, out_len, "%.9g", d);
            break;
    }
}

int rocket_param_value_matches(uint8_t type, float actual, double expected, double eps) {
    double decoded = rocket_decode_param_number(type, actual);
    if (type == MAV_PARAM_TYPE_REAL32 || type == MAV_PARAM_TYPE_REAL64) {
        return fabs(decoded - expected) < eps;
    }
    return llround(decoded) == llround(expected);
}
