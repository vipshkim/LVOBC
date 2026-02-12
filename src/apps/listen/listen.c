#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>
#include "mavlink_get_info.h"
#include "rocket_mav_common.h"

#define DEFAULT_LISTEN_IP MAV_DEFAULT_IP
#define DEFAULT_LISTEN_PORT 15551
#define SELECT_TIMEOUT_SEC 0.25

static volatile sig_atomic_t g_running = 1;

static void handle_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void normalize_name(char *dst, size_t dst_len, const char *src) {
    if (!dst || dst_len == 0) return;
    size_t i = 0;
    for (; src && src[i] && i + 1 < dst_len; ++i) {
        unsigned char c = (unsigned char)src[i];
        if (c == '-' || c == ' ') c = '_';
        dst[i] = (char)toupper(c);
    }
    dst[i] = '\0';
}

static int parse_msg_id(const char *s, uint32_t *out_id) {
    if (!s || !s[0]) return 0;
    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (!end || *end != '\0') return 0;
    if (v < 0 || v > (long)UINT32_MAX) return 0;
    if (out_id) *out_id = (uint32_t)v;
    return 1;
}

static size_t type_size(mavlink_message_type_t type) {
    switch (type) {
        case MAVLINK_TYPE_CHAR:
        case MAVLINK_TYPE_UINT8_T:
        case MAVLINK_TYPE_INT8_T:
            return 1;
        case MAVLINK_TYPE_UINT16_T:
        case MAVLINK_TYPE_INT16_T:
            return 2;
        case MAVLINK_TYPE_UINT32_T:
        case MAVLINK_TYPE_INT32_T:
        case MAVLINK_TYPE_FLOAT:
            return 4;
        case MAVLINK_TYPE_UINT64_T:
        case MAVLINK_TYPE_INT64_T:
        case MAVLINK_TYPE_DOUBLE:
            return 8;
        default:
            return 0;
    }
}

static int field_within_payload(const mavlink_message_t *msg, const mavlink_field_info_t *field) {
    if (!msg || !field) return 0;
    size_t unit = type_size(field->type);
    if (unit == 0) return 0;
    size_t len = (field->array_length > 0) ? (unit * field->array_length) : unit;
    return (field->wire_offset + len) <= msg->len;
}

static void print_scalar_value(const mavlink_message_t *msg, mavlink_message_type_t type, uint8_t offset) {
    switch (type) {
        case MAVLINK_TYPE_CHAR: {
            char v = _MAV_RETURN_char(msg, offset);
            if (isprint((unsigned char)v)) {
                printf("'%c'", v);
            } else {
                printf("%d", (int)v);
            }
            break;
        }
        case MAVLINK_TYPE_UINT8_T:
            printf("%u", (unsigned)_MAV_RETURN_uint8_t(msg, offset));
            break;
        case MAVLINK_TYPE_INT8_T:
            printf("%d", (int)_MAV_RETURN_int8_t(msg, offset));
            break;
        case MAVLINK_TYPE_UINT16_T:
            printf("%u", (unsigned)_MAV_RETURN_uint16_t(msg, offset));
            break;
        case MAVLINK_TYPE_INT16_T:
            printf("%d", (int)_MAV_RETURN_int16_t(msg, offset));
            break;
        case MAVLINK_TYPE_UINT32_T:
            printf("%" PRIu32, (uint32_t)_MAV_RETURN_uint32_t(msg, offset));
            break;
        case MAVLINK_TYPE_INT32_T:
            printf("%" PRId32, (int32_t)_MAV_RETURN_int32_t(msg, offset));
            break;
        case MAVLINK_TYPE_UINT64_T:
            printf("%" PRIu64, (uint64_t)_MAV_RETURN_uint64_t(msg, offset));
            break;
        case MAVLINK_TYPE_INT64_T:
            printf("%" PRId64, (int64_t)_MAV_RETURN_int64_t(msg, offset));
            break;
        case MAVLINK_TYPE_FLOAT:
            printf("%.6g", (double)_MAV_RETURN_float(msg, offset));
            break;
        case MAVLINK_TYPE_DOUBLE:
            printf("%.6g", _MAV_RETURN_double(msg, offset));
            break;
        default:
            printf("?");
            break;
    }
}

static void print_array_value(const mavlink_message_t *msg, const mavlink_field_info_t *field) {
    if (field->type == MAVLINK_TYPE_CHAR) {
        size_t len = field->array_length;
        if (len > 255) len = 255;
        char buf[256];
        if (len > 0) {
            _MAV_RETURN_char_array(msg, buf, (uint8_t)len, (uint8_t)field->wire_offset);
        }
        buf[len] = '\0';
        for (size_t i = 0; i < len; ++i) {
            if (buf[i] == '\0') break;
            if (!isprint((unsigned char)buf[i])) buf[i] = '.';
        }
        printf("\"%s\"", buf);
        return;
    }

    size_t unit = type_size(field->type);
    size_t len = field->array_length;
    printf("[");
    for (size_t i = 0; i < len; ++i) {
        if (i) printf(",");
        uint8_t ofs = (uint8_t)(field->wire_offset + i * unit);
        print_scalar_value(msg, field->type, ofs);
    }
    printf("]");
}

static void print_flags_base_mode(uint8_t base_mode) {
    const struct { uint8_t bit; const char *name; } flags[] = {
        {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"},
        {MAV_MODE_FLAG_TEST_ENABLED, "MAV_MODE_FLAG_TEST_ENABLED"},
        {MAV_MODE_FLAG_AUTO_ENABLED, "MAV_MODE_FLAG_AUTO_ENABLED"},
        {MAV_MODE_FLAG_GUIDED_ENABLED, "MAV_MODE_FLAG_GUIDED_ENABLED"},
        {MAV_MODE_FLAG_STABILIZE_ENABLED, "MAV_MODE_FLAG_STABILIZE_ENABLED"},
        {MAV_MODE_FLAG_HIL_ENABLED, "MAV_MODE_FLAG_HIL_ENABLED"},
        {MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED"},
        {MAV_MODE_FLAG_SAFETY_ARMED, "MAV_MODE_FLAG_SAFETY_ARMED"},
    };

    printf("0x%02X(", base_mode);
    int first = 1;
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i) {
        if ((base_mode & flags[i].bit) == 0) continue;
        if (!first) printf("|");
        printf("%s", flags[i].name);
        first = 0;
    }
    if (first) printf("-");
    printf(")");
}

static void print_message_fields(const mavlink_message_t *msg,
                                 const mavlink_message_info_t *info,
                                 int expand_flags) {
    printf("%s sys=%u comp=%u seq=%u ",
           info->name ? info->name : "MSG",
           msg->sysid,
           msg->compid,
           msg->seq);

    for (unsigned i = 0; i < info->num_fields; ++i) {
        const mavlink_field_info_t *field = &info->fields[i];
        if (i) printf(" ");
        printf("%s=", field->name ? field->name : "field");
        if (!field_within_payload(msg, field)) {
            printf("<truncated>");
            continue;
        }
        if (expand_flags &&
            info->msgid == MAVLINK_MSG_ID_HEARTBEAT &&
            field->array_length == 0 &&
            field->type == MAVLINK_TYPE_UINT8_T &&
            field->name &&
            strcmp(field->name, "base_mode") == 0) {
            uint8_t base_mode = _MAV_RETURN_uint8_t(msg, (uint8_t)field->wire_offset);
            print_flags_base_mode(base_mode);
            continue;
        }
        if (field->array_length > 0) {
            print_array_value(msg, field);
        } else {
            print_scalar_value(msg, field->type, (uint8_t)field->wire_offset);
        }
    }
    printf("\n");
}

static void print_message_raw(const mavlink_message_t *msg) {
    printf("ID_%u sys=%u comp=%u seq=%u len=%u payload=", (unsigned)msg->msgid, msg->sysid, msg->compid, msg->seq, msg->len);
    const uint8_t *payload = (const uint8_t *)_MAV_PAYLOAD(msg);
    for (uint8_t i = 0; i < msg->len; ++i) {
        printf("%02X", payload[i]);
    }
    printf("\n");
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s <MESSAGE|MSGID> [options]\n"
            "Options:\n"
            "  -l <ip>    listen ip (default %s)\n"
            "  -p <port>  listen port (default from ROCKET_MAV_TOOLS_LISTEN_PORT or %d)\n"
            "  -d <sys>   filter by sysid (device)\n"
            "  -c <comp>  filter by compid\n"
            "  -n <cnt>   stop after cnt messages (default 0 = run)\n"
            "  -t <sec>   stop after seconds (default 0 = run)\n"
            "  -F         expand known flag fields (HEARTBEAT.base_mode)\n"
            "  -h         help\n",
            prog, DEFAULT_LISTEN_IP, DEFAULT_LISTEN_PORT);
}

int main(int argc, char **argv) {
    const char *listen_ip = DEFAULT_LISTEN_IP;
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    const char *msg_arg = NULL;
    int filter_sys = -1;
    int filter_comp = -1;
    int max_msgs = 0;
    double max_sec = 0.0;
    int expand_flags = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            filter_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            filter_comp = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            max_msgs = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            max_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-F") == 0) {
            expand_flags = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-' && !msg_arg) {
            msg_arg = argv[i];
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (!msg_arg) {
        usage(argv[0]);
        return 1;
    }

    uint32_t target_id = 0;
    const mavlink_message_info_t *target_info = NULL;
    if (parse_msg_id(msg_arg, &target_id)) {
        target_info = mavlink_get_message_info_by_id(target_id);
    } else {
        char name_buf[64];
        normalize_name(name_buf, sizeof(name_buf), msg_arg);
        target_info = mavlink_get_message_info_by_name(name_buf);
        if (!target_info) {
            fprintf(stderr, "unknown MAVLink message name: %s\n", name_buf);
            return 1;
        }
        target_id = target_info->msgid;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)listen_port);
    if (inet_pton(AF_INET, listen_ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid ip: %s\n", listen_ip);
        close(fd);
        return 1;
    }
    if (bind(fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "failed to bind %s:%d: %s\n", listen_ip, listen_port, strerror(errno));
        close(fd);
        return 1;
    }

    printf("listen: %s:%d msg=%s(%u)", listen_ip, listen_port,
           target_info && target_info->name ? target_info->name : "ID", target_id);
    if (filter_sys >= 0) printf(" sys=%d", filter_sys);
    if (filter_comp >= 0) printf(" comp=%d", filter_comp);
    printf("\n");

    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    int msg_count = 0;
    double start = monotonic_seconds();

    while (g_running) {
        if (max_msgs > 0 && msg_count >= max_msgs) break;
        if (max_sec > 0.0 && (monotonic_seconds() - start) >= max_sec) break;

        struct timeval tv;
        tv.tv_sec = (time_t)SELECT_TIMEOUT_SEC;
        tv.tv_usec = (suseconds_t)((SELECT_TIMEOUT_SEC - (double)tv.tv_sec) * 1e6);

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (ready == 0) continue;

        uint8_t buf[2048];
        ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recvfrom");
            break;
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;
            if (msg.msgid != target_id) continue;
            if (filter_sys >= 0 && msg.sysid != (uint8_t)filter_sys) continue;
            if (filter_comp >= 0 && msg.compid != (uint8_t)filter_comp) continue;

            const mavlink_message_info_t *info = target_info ? target_info : mavlink_get_message_info_by_id(msg.msgid);
            if (info) {
                print_message_fields(&msg, info, expand_flags);
            } else {
                print_message_raw(&msg);
            }
            fflush(stdout);
            msg_count++;
            if (max_msgs > 0 && msg_count >= max_msgs) break;
        }
    }

    close(fd);
    return 0;
}
