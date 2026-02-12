#include "rocket_common.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

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

int rocket_single_instance_acquire(const char *name, char *err, size_t err_len) {
    if (!name || !name[0]) {
        if (err && err_len > 0) snprintf(err, err_len, "invalid lock name");
        return -1;
    }

    char lock_path[256];
    snprintf(lock_path, sizeof(lock_path), "/tmp/%s.lock", name);

    int fd = open(lock_path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        if (errno == EACCES) {
            if (err && err_len > 0) {
                snprintf(err, err_len, "already running or locked by another user (%s)", lock_path);
            }
            return -1;
        }
        if (err && err_len > 0) {
            snprintf(err, err_len, "open(%s): %s", lock_path, strerror(errno));
        }
        return -1;
    }

    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        int saved = errno;
        if (err && err_len > 0) {
            if (saved == EWOULDBLOCK || saved == EAGAIN) {
                snprintf(err, err_len, "already running (%s locked)", lock_path);
            } else {
                snprintf(err, err_len, "flock(%s): %s", lock_path, strerror(saved));
            }
        }
        close(fd);
        return -1;
    }

    int flags = fcntl(fd, F_GETFD);
    if (flags >= 0) (void)fcntl(fd, F_SETFD, flags | FD_CLOEXEC);

    (void)ftruncate(fd, 0);
    {
        char pidbuf[32];
        int n = snprintf(pidbuf, sizeof(pidbuf), "%ld\n", (long)getpid());
        if (n > 0) (void)write(fd, pidbuf, (size_t)n);
    }

    if (err && err_len > 0) err[0] = '\0';
    return fd;
}

void rocket_single_instance_release(int lock_fd) {
    if (lock_fd >= 0) {
        (void)close(lock_fd);
    }
}
