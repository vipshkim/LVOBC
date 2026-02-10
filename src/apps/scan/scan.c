// Combined MAVLink scan + parameter fetcher.
// - Scans incoming MAVLink messages to list unique message names.
// - Requests parameters and saves to /tmp (RAM) as config.params.

#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>
#include "rocket_common.h"
#include "rocket_mav_common.h"

#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_RED     "\033[31m"
#define CLR_GREEN   "\033[32m"
#define CLR_YELLOW  "\033[33m"
#define CLR_BLUE    "\033[34m"
#define CLR_MAGENTA "\033[35m"
#define CLR_CYAN    "\033[36m"

#define DEFAULT_LISTEN_IP MAV_DEFAULT_IP
#define DEFAULT_LISTEN_PORT 14553
#define DEFAULT_TARGET_IP MAV_DEFAULT_IP
#define DEFAULT_TARGET_PORT 14653
#define DEFAULT_SYSID MAV_DEFAULT_SYSID
#define DEFAULT_COMPID 193
#define DEFAULT_TARGET_SYS MAV_DEFAULT_TARGET_SYS
#define DEFAULT_TARGET_COMP MAV_DEFAULT_TARGET_COMP

#define DEFAULT_SCAN_DURATION_SEC 5.0
#define DEFAULT_PARAM_TIMEOUT_SEC 10.0
#define DEFAULT_SERIAL_BAUD 921600
#define PARAM_INITIAL_LIST_RETRY_SEC 8.0
#define PARAM_INITIAL_LIST_MAX_SEND 2
#define PARAM_MISSING_TRIGGER_IDLE_SEC 1.5
#define PARAM_MISSING_RETRY_INTERVAL_SEC 1.0
#define PARAM_MISSING_BATCH_SIZE 10
#define PARAM_MISSING_MAX_RETRY_PER_INDEX 3
#define SELECT_TIMEOUT_SEC 0.2
#define PARAM_PROGRESS_PRINT_SEC 0.5
#define PARAM_UNKNOWN_SOURCE_IDLE_DONE_SEC 2.5
#define ODOM_STREAM_HZ 100
#define ODOM_STREAM_INTERVAL_US (1000000 / ODOM_STREAM_HZ)
#define ODOM_STREAM_REQUEST_DELAY_SEC 1.0
#define ODOM_STREAM_REQUEST_RETRY_SEC 2.0
#define ODOM_STREAM_REQUEST_MAX_SEND 3
#define DEFAULT_OUTPUT MAV_DEFAULT_TMP_PARAMS
#define DEFAULT_PERSISTENT_OUTPUT MAV_DEFAULT_PERSIST_PARAMS

typedef struct {
    char **items;
    size_t count;
} topic_list_t;

typedef struct {
    char name[17];
    int index;
    char value[64];
    uint8_t type;
    float raw_value;
    double parsed_value;
    bool has_parsed_value;
} param_entry_t;

typedef struct {
    param_entry_t *items;
    size_t count;
} param_list_t;

typedef struct {
    uint8_t sysid;
    uint8_t compid;
    struct sockaddr_in addr;
    bool has_addr;
    char src_text[64];
    bool list_sent_once;
    bool source_reported;
    int list_req_sent;
    double last_list_req;
    double last_param_recv;
    double last_missing_req;
    bool exhausted;
    int param_count;
    int received_count;
    bool *received;
    uint8_t *read_retry_count;
    size_t received_len;
    int missing_cursor;
    param_list_t params;
} param_source_t;

typedef struct {
    param_source_t *items;
    size_t count;
} param_source_list_t;

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static int topic_exists(const topic_list_t *list, const char *value) {
    for (size_t i = 0; i < list->count; ++i) {
        if (strcmp(list->items[i], value) == 0) return 1;
    }
    return 0;
}

static void topic_add(topic_list_t *list, const char *value) {
    if (topic_exists(list, value)) return;
    char **next = realloc(list->items, (list->count + 1) * sizeof(char *));
    if (!next) return;
    list->items = next;
    list->items[list->count] = strdup(value);
    if (!list->items[list->count]) return;
    list->count++;
}

static int compare_strings(const void *a, const void *b) {
    const char *lhs = *(const char **)a;
    const char *rhs = *(const char **)b;
    return strcmp(lhs, rhs);
}

static void topics_free(topic_list_t *list) {
    for (size_t i = 0; i < list->count; ++i) free(list->items[i]);
    free(list->items);
    list->items = NULL;
    list->count = 0;
}

static void params_init(param_list_t *list) {
    list->items = NULL;
    list->count = 0;
}

static void params_free(param_list_t *list) {
    free(list->items);
    list->items = NULL;
    list->count = 0;
}

static void param_source_init(param_source_t *src, uint8_t sysid, uint8_t compid, const struct sockaddr_in *addr, const char *src_text) {
    memset(src, 0, sizeof(*src));
    src->sysid = sysid;
    src->compid = compid;
    if (addr) {
        src->addr = *addr;
        src->has_addr = true;
    }
    if (src_text && src_text[0]) {
        snprintf(src->src_text, sizeof(src->src_text), "%s", src_text);
    } else {
        snprintf(src->src_text, sizeof(src->src_text), "unknown");
    }
    src->last_param_recv = monotonic_seconds();
    src->param_count = -1;
    params_init(&src->params);
}

static void param_source_free(param_source_t *src) {
    free(src->received);
    free(src->read_retry_count);
    src->received = NULL;
    src->read_retry_count = NULL;
    src->received_len = 0;
    params_free(&src->params);
}

static void source_list_free(param_source_list_t *list) {
    for (size_t i = 0; i < list->count; ++i) {
        param_source_free(&list->items[i]);
    }
    free(list->items);
    list->items = NULL;
    list->count = 0;
}

static int source_find_index(const param_source_list_t *list, uint8_t sysid, uint8_t compid) {
    for (size_t i = 0; i < list->count; ++i) {
        if (list->items[i].sysid == sysid && list->items[i].compid == compid) {
            return (int)i;
        }
    }
    return -1;
}

static param_source_t *source_get_or_add(param_source_list_t *list,
                                         uint8_t sysid,
                                         uint8_t compid,
                                         const struct sockaddr_in *addr,
                                         const char *src_text,
                                         bool *added) {
    int idx = source_find_index(list, sysid, compid);
    if (idx >= 0) {
        param_source_t *existing = &list->items[idx];
        if (addr) {
            existing->addr = *addr;
            existing->has_addr = true;
        }
        if (src_text && src_text[0]) {
            snprintf(existing->src_text, sizeof(existing->src_text), "%s", src_text);
        }
        if (added) *added = false;
        return existing;
    }

    param_source_t *next = realloc(list->items, (list->count + 1) * sizeof(param_source_t));
    if (!next) return NULL;
    list->items = next;
    param_source_init(&list->items[list->count], sysid, compid, addr, src_text);
    if (added) *added = true;
    list->count++;
    return &list->items[list->count - 1];
}

static int params_find(const param_list_t *list, const char *name) {
    for (size_t i = 0; i < list->count; ++i) {
        if (strcmp(list->items[i].name, name) == 0) return (int)i;
    }
    return -1;
}

static int parse_double(const char *s, double *out) {
    if (!s || !out) return 0;
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno != 0 || end == s) return 0;
    *out = v;
    return 1;
}

static double decode_param_number(uint8_t type, float value) {
    return rocket_decode_param_number(type, value);
}

static float encode_param_value(uint8_t type, double value) {
    return rocket_encode_param_value(type, value);
}

static int params_get_value(const param_list_t *list, const char *name, double *out, uint8_t *out_type) {
    int idx = params_find(list, name);
    if (idx < 0) return 0;
    if (out) {
        if (list->items[idx].has_parsed_value) {
            *out = list->items[idx].parsed_value;
        } else {
            *out = decode_param_number(list->items[idx].type, list->items[idx].raw_value);
        }
    }
    if (out_type) *out_type = list->items[idx].type;
    return 1;
}

static void params_add_or_update(param_list_t *list, const char *name, int index, uint8_t type, float value);
static void params_add_or_update_parsed(param_list_t *list, const char *name, double value);

static void trim(char *s) {
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        len--;
    }
}

static void params_load_text_file(param_list_t *list, const char *path) {
    if (!path || !path[0]) return;
    FILE *fp = fopen(path, "r");
    if (!fp) return;

    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        trim(line);
        if (line[0] == '\0' || line[0] == '#') continue;

        if (strncmp(line, "param set-default ", 18) == 0) {
            char name[64] = {0};
            char value_str[64] = {0};
            if (sscanf(line + 18, "%63s %63s", name, value_str) == 2) {
                double v;
                if (parse_double(value_str, &v)) {
                    params_add_or_update_parsed(list, name, v);
                }
            }
            continue;
        }

        char *eq = strchr(line, '=');
        if (eq) {
            *eq = '\0';
            char *name = line;
            char *value_str = eq + 1;
            trim(name);
            trim(value_str);
            double v;
            if (name[0] != '\0' && parse_double(value_str, &v)) {
                params_add_or_update_parsed(list, name, v);
            }
            continue;
        }

        char name[64] = {0};
        char value_str[64] = {0};
        if (sscanf(line, "%63s %63s", name, value_str) == 2) {
            double v;
            if (parse_double(value_str, &v)) {
                params_add_or_update_parsed(list, name, v);
            }
        }
    }
    fclose(fp);
}

static param_source_t *select_primary_source(param_source_list_t *sources, int target_sys, int target_comp) {
    if (!sources) return NULL;
    for (size_t i = 0; i < sources->count; ++i) {
        if ((int)sources->items[i].sysid == target_sys &&
            (int)sources->items[i].compid == target_comp &&
            sources->items[i].params.count > 0) {
            return &sources->items[i];
        }
    }
    for (size_t i = 0; i < sources->count; ++i) {
        if (sources->items[i].params.count > 0) {
            return &sources->items[i];
        }
    }
    return NULL;
}

static int values_differ(uint8_t type, double a, double b) {
    switch (type) {
        case MAV_PARAM_TYPE_UINT8:
        case MAV_PARAM_TYPE_INT8:
        case MAV_PARAM_TYPE_UINT16:
        case MAV_PARAM_TYPE_INT16:
        case MAV_PARAM_TYPE_UINT32:
        case MAV_PARAM_TYPE_INT32:
            return llround(a) != llround(b);
        default:
            return fabs(a - b) > 1e-6;
    }
}

static void format_param_value(uint8_t type, float value, char *out, size_t out_len) {
    rocket_format_param_value(type, value, out, out_len);
}

static void params_add_or_update(param_list_t *list, const char *name, int index, uint8_t type, float value) {
    char formatted[64];
    format_param_value(type, value, formatted, sizeof(formatted));

    int pos = params_find(list, name);
    if (pos >= 0) {
        list->items[pos].index = index;
        list->items[pos].type = type;
        list->items[pos].raw_value = value;
        list->items[pos].parsed_value = 0.0;
        list->items[pos].has_parsed_value = false;
        snprintf(list->items[pos].value, sizeof(list->items[pos].value), "%s", formatted);
        return;
    }

    param_entry_t *next = realloc(list->items, (list->count + 1) * sizeof(param_entry_t));
    if (!next) return;
    list->items = next;

    snprintf(list->items[list->count].name, sizeof(list->items[list->count].name), "%s", name);
    list->items[list->count].index = index;
    list->items[list->count].type = type;
    list->items[list->count].raw_value = value;
    list->items[list->count].parsed_value = 0.0;
    list->items[list->count].has_parsed_value = false;
    snprintf(list->items[list->count].value, sizeof(list->items[list->count].value), "%s", formatted);
    list->count++;
}

static void params_add_or_update_parsed(param_list_t *list, const char *name, double value) {
    int pos = params_find(list, name);
    if (pos >= 0) {
        list->items[pos].index = -1;
        list->items[pos].type = MAV_PARAM_TYPE_REAL32;
        list->items[pos].raw_value = (float)value;
        list->items[pos].parsed_value = value;
        list->items[pos].has_parsed_value = true;
        snprintf(list->items[pos].value, sizeof(list->items[pos].value), "%.17g", value);
        return;
    }

    param_entry_t *next = realloc(list->items, (list->count + 1) * sizeof(param_entry_t));
    if (!next) return;
    list->items = next;

    snprintf(list->items[list->count].name, sizeof(list->items[list->count].name), "%s", name);
    list->items[list->count].index = -1;
    list->items[list->count].type = MAV_PARAM_TYPE_REAL32;
    list->items[list->count].raw_value = (float)value;
    list->items[list->count].parsed_value = value;
    list->items[list->count].has_parsed_value = true;
    snprintf(list->items[list->count].value, sizeof(list->items[list->count].value), "%.17g", value);
    list->count++;
}

static int compare_entries(const void *a, const void *b) {
    const param_entry_t *lhs = (const param_entry_t *)a;
    const param_entry_t *rhs = (const param_entry_t *)b;
    if (lhs->index >= 0 && rhs->index >= 0 && lhs->index != rhs->index) {
        return lhs->index - rhs->index;
    }
    return strcmp(lhs->name, rhs->name);
}

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return 0;
    }
}

static int open_serial_port(const char *device, int baud) {
    speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
        fprintf(stderr, "Unsupported baud rate: %d\n", baud);
        return -1;
    }

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "open(%s) failed: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        fprintf(stderr, "tcgetattr(%s) failed: %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
#ifdef CRTSCTS
    tio.c_cflag |= CRTSCTS;
#endif

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        fprintf(stderr, "cfset*speed(%s) failed: %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        fprintf(stderr, "tcsetattr(%s) failed: %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static ssize_t send_mavlink_packet(int io_fd, bool serial_mode, const struct sockaddr_in *dest, const uint8_t *buf, size_t len) {
    if (serial_mode) {
        return write(io_fd, buf, len);
    }
    return sendto(io_fd, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void send_param_request_list(int io_fd, bool serial_mode, const struct sockaddr_in *dest, uint8_t sysid, uint8_t compid, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_request_list_pack(sysid, compid, &msg, target_sys, target_comp);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static void send_param_request_read(int io_fd, bool serial_mode, const struct sockaddr_in *dest, uint8_t sysid, uint8_t compid,
                                    uint8_t target_sys, uint8_t target_comp, int16_t param_index) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    char param_id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN] = {0};

    mavlink_msg_param_request_read_pack(sysid, compid, &msg, target_sys, target_comp, param_id, param_index);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static void send_param_set(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                           uint8_t sysid, uint8_t compid, uint8_t target_sys, uint8_t target_comp,
                           const char *param_id, uint8_t param_type, double value) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    char idbuf[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN] = {0};
    snprintf(idbuf, sizeof(idbuf), "%s", param_id);

    float encoded = encode_param_value(param_type, value);
    mavlink_msg_param_set_pack(sysid, compid, &msg,
                               target_sys, target_comp,
                               idbuf, encoded, param_type);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static void send_set_message_interval(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                                      uint8_t sysid, uint8_t compid, uint8_t target_sys, uint8_t target_comp,
                                      uint32_t message_id, int32_t interval_us) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        (float)message_id,
        (float)interval_us,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static int count_missing_params(const bool *received, int param_count) {
    if (!received || param_count <= 0) return 0;
    int missing = 0;
    for (int i = 0; i < param_count; ++i) {
        if (!received[i]) missing++;
    }
    return missing;
}

static void print_section(const char *title, const char *color) {
    printf("%s%s%s%s\n", CLR_BOLD, color, title, CLR_RESET);
}

static void clear_progress_line(bool inline_mode, bool *line_active) {
    if (!inline_mode || !line_active || !*line_active) return;
    printf("\r\033[2K");
    fflush(stdout);
    *line_active = false;
}

static void print_param_progress(const param_source_list_t *sources,
                                 double elapsed_sec,
                                 bool timed_out,
                                 bool inline_mode,
                                 bool *line_active) {
    size_t source_count = sources ? sources->count : 0;
    size_t done_sources = 0;
    size_t unknown_sources = 0;
    size_t rows_seen = 0;
    int total_expected = 0;
    int total_received = 0;

    for (size_t i = 0; i < source_count; ++i) {
        const param_source_t *src = &sources->items[i];
        rows_seen += src->params.count;
        if (src->param_count > 0) {
            int received = src->received_count;
            if (received < 0) received = 0;
            if (received > src->param_count) received = src->param_count;
            total_expected += src->param_count;
            total_received += received;
            if (received >= src->param_count || src->exhausted) {
                done_sources++;
            }
        } else {
            unknown_sources++;
            if (src->exhausted) done_sources++;
        }
    }

    char line[256];

    if (source_count == 0) {
        snprintf(line, sizeof(line), "%sPARAM%s progress | waiting HEARTBEAT source | t=%.1fs%s",
                 CLR_YELLOW, CLR_RESET, elapsed_sec, timed_out ? " | timeout" : "");
    } else if (total_expected > 0) {
        double pct = (double)total_received * 100.0 / (double)total_expected;
        if (pct < 0.0) pct = 0.0;
        if (pct > 100.0) pct = 100.0;

        char bar[21];
        int filled = (int)lround((pct / 100.0) * 20.0);
        if (filled < 0) filled = 0;
        if (filled > 20) filled = 20;
        for (int i = 0; i < 20; ++i) {
            bar[i] = (i < filled) ? '#' : '.';
        }
        bar[20] = '\0';

        snprintf(line, sizeof(line),
                 "%sPARAM%s progress [%s] %6.2f%% (%d/%d) src=%zu done=%zu unknown=%zu t=%.1fs%s",
                 CLR_YELLOW, CLR_RESET, bar, pct, total_received, total_expected,
                 source_count, done_sources, unknown_sources, elapsed_sec,
                 timed_out ? " | timeout" : "");
    } else {
        snprintf(line, sizeof(line),
                 "%sPARAM%s progress | waiting param_count metadata | src=%zu rows=%zu t=%.1fs%s",
                 CLR_YELLOW, CLR_RESET, source_count, rows_seen, elapsed_sec,
                 timed_out ? " | timeout" : "");
    }

    if (inline_mode) {
        printf("\r\033[2K%s", line);
        fflush(stdout);
        if (line_active) *line_active = true;
    } else {
        printf("%s\n", line);
        if (line_active) *line_active = false;
    }
}

static void format_addr(const struct sockaddr_in *addr, char *out, size_t out_len) {
    char ip[INET_ADDRSTRLEN] = {0};
    if (!inet_ntop(AF_INET, &addr->sin_addr, ip, sizeof(ip))) {
        snprintf(out, out_len, "?:?");
        return;
    }
    snprintf(out, out_len, "%s:%u", ip, (unsigned)ntohs(addr->sin_port));
}

static void usage(const char *prog,
                  const char *listen_ip_default,
                  int listen_port_default,
                  const char *target_ip_default,
                  int target_port_default,
                  int serial_baud_default,
                  const char *output_default,
                  int target_sys_default,
                  int target_comp_default,
                  int sysid_default,
                  int compid_default) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  -l <ip>      : listen IP (default %s)\n"
        "  -q <port>    : listen port (default %d)\n"
        "  -u <ip>      : target IP (default %s)\n"
        "  -p <port>    : target port (default %d)\n"
        "  -S <dev>     : serial device for direct FCU access (ex: /dev/ttyAMA0)\n"
        "  -B <baud>    : serial baud (default %d)\n"
        "  -o <path>    : output file (default %s)\n"
        "  -t <sysid>   : target sysid (default %d, or auto from HEARTBEAT)\n"
        "  -k <compid>  : target compid (default %d, or auto from HEARTBEAT)\n"
        "  -s <sysid>   : sender sysid (default %d)\n"
        "  -c <compid>  : sender compid (default %d)\n"
        "  -d <sec>     : scan duration (default %.0f)\n"
        "  -D <sec>     : param timeout (default %.0f)\n"
        "  --no-scan    : disable message scan\n"
        "  --no-params  : disable parameter fetch\n"
        "  --autopilot  : only accept HEARTBEAT from compid=AUTOPILOT1\n"
        "  --wait-full  : wait until full parameter set (ignores timeout unless no sources)\n"
        "  --allow-full-rewrite : allow full rewrite of persistent file if patch fails\n"
        "  --upload     : upload params from persistent file (diff vs /tmp)\n"
        "  --upload-path <path> : upload params from specific file\n",
        prog,
        listen_ip_default,
        listen_port_default,
        target_ip_default,
        target_port_default,
        serial_baud_default,
        output_default,
        target_sys_default,
        target_comp_default,
        sysid_default,
        compid_default,
        DEFAULT_SCAN_DURATION_SEC,
        DEFAULT_PARAM_TIMEOUT_SEC);
}

typedef struct {
    size_t start;
    size_t len;
} line_span_t;

typedef struct {
    char key[64];
    const char *line_ptr;
    size_t line_len;
    bool used;
} param_line_ref_t;

static int append_bytes(char **buf, size_t *len, size_t *cap, const char *src, size_t src_len) {
    if (!buf || !len || !cap) return 0;
    if (src_len == 0) return 1;
    if (!*buf || *cap == 0) {
        *cap = (src_len > 4096) ? (src_len * 2) : 4096;
        *buf = malloc(*cap);
        if (!*buf) return 0;
        *len = 0;
    }
    if (*len + src_len > *cap) {
        size_t new_cap = *cap;
        while (*len + src_len > new_cap) {
            new_cap *= 2;
        }
        char *next = realloc(*buf, new_cap);
        if (!next) return 0;
        *buf = next;
        *cap = new_cap;
    }
    memcpy(*buf + *len, src, src_len);
    *len += src_len;
    return 1;
}

static int parse_param_key_from_line(const char *line, size_t len, char *key_out, size_t key_out_len) {
    if (!line || len == 0 || !key_out || key_out_len < 2) return 0;

    size_t s = 0;
    while (s < len && (line[s] == ' ' || line[s] == '\t')) s++;
    if (s >= len) return 0;
    if (line[s] == '#' || line[s] == '\n' || line[s] == '\r') return 0;

    size_t eq = s;
    while (eq < len && line[eq] != '=' && line[eq] != '\n' && line[eq] != '\r') eq++;
    if (eq >= len || line[eq] != '=') return 0;

    size_t e = eq;
    while (e > s && (line[e - 1] == ' ' || line[e - 1] == '\t')) e--;
    if (e <= s) return 0;

    size_t key_len = e - s;
    if (key_len + 1 > key_out_len) return 0;
    memcpy(key_out, line + s, key_len);
    key_out[key_len] = '\0';
    return 1;
}

static int param_ref_find(const param_line_ref_t *refs, size_t count, const char *key) {
    for (size_t i = 0; i < count; ++i) {
        if (strcmp(refs[i].key, key) == 0) return (int)i;
    }
    return -1;
}

static int text_appendf(char **buf, size_t *len, size_t *cap, const char *fmt, ...) {
    if (!buf || !len || !cap || !fmt) return 0;
    if (!*buf || *cap == 0) {
        *cap = 4096;
        *buf = calloc(*cap, 1);
        if (!*buf) return 0;
        *len = 0;
    }

    while (1) {
        va_list ap;
        va_start(ap, fmt);
        int need = vsnprintf(*buf + *len, *cap - *len, fmt, ap);
        va_end(ap);
        if (need < 0) return 0;

        size_t remaining = *cap - *len;
        if ((size_t)need < remaining) {
            *len += (size_t)need;
            return 1;
        }

        size_t new_cap = *cap * 2;
        while (new_cap - *len <= (size_t)need) {
            new_cap *= 2;
        }
        char *next = realloc(*buf, new_cap);
        if (!next) return 0;
        *buf = next;
        *cap = new_cap;
    }
}

static int build_params_text(char **out_buf, size_t *out_len, const param_source_list_t *sources,
                             const char *target_ip, int target_port, int target_sys, int target_comp) {
    char *buf = NULL;
    size_t len = 0;
    size_t cap = 0;

    bool wrote_any_section = false;
    for (size_t i = 0; i < sources->count; ++i) {
        const param_source_t *src = &sources->items[i];
        if (src->params.count == 0) continue;
        if (!text_appendf(&buf, &len, &cap, "# MAVLink params saved from %s (tsys=%u tcomp=%u)\n",
                          src->src_text, src->sysid, src->compid)) {
            free(buf);
            return 0;
        }
        for (size_t j = 0; j < src->params.count; ++j) {
            if (!text_appendf(&buf, &len, &cap, "%s=%s\n", src->params.items[j].name, src->params.items[j].value)) {
                free(buf);
                return 0;
            }
        }
        if (!text_appendf(&buf, &len, &cap, "\n")) {
            free(buf);
            return 0;
        }
        wrote_any_section = true;
    }

    if (!wrote_any_section) {
        if (!text_appendf(&buf, &len, &cap, "# MAVLink params saved from %s:%d (tsys=%d tcomp=%d)\n",
                          target_ip, target_port, target_sys, target_comp)) {
            free(buf);
            return 0;
        }
    }

    *out_buf = buf;
    *out_len = len;
    return 1;
}

static int write_raw_file(const char *path, const char *data, size_t len) {
    FILE *fp = fopen(path, "wb");
    if (!fp) return 0;
    if (len > 0 && fwrite(data, 1, len, fp) != len) {
        fclose(fp);
        return 0;
    }
    fclose(fp);
    return 1;
}

static int read_raw_file(const char *path, char **out, size_t *out_len) {
    *out = NULL;
    *out_len = 0;
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        if (errno == ENOENT) return 1;
        return 0;
    }
    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return 0;
    }
    long sz = ftell(fp);
    if (sz < 0) {
        fclose(fp);
        return 0;
    }
    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return 0;
    }
    char *buf = NULL;
    if (sz > 0) {
        buf = malloc((size_t)sz);
        if (!buf) {
            fclose(fp);
            return 0;
        }
        if (fread(buf, 1, (size_t)sz, fp) != (size_t)sz) {
            free(buf);
            fclose(fp);
            return 0;
        }
    }
    fclose(fp);
    *out = buf;
    *out_len = (size_t)sz;
    return 1;
}

static size_t build_line_spans(const char *buf, size_t len, line_span_t **out_spans) {
    *out_spans = NULL;
    if (len == 0) return 0;

    size_t cap = 64;
    size_t count = 0;
    line_span_t *spans = malloc(cap * sizeof(line_span_t));
    if (!spans) return 0;

    size_t start = 0;
    for (size_t i = 0; i < len; ++i) {
        if (buf[i] == '\n') {
            if (count == cap) {
                cap *= 2;
                line_span_t *next = realloc(spans, cap * sizeof(line_span_t));
                if (!next) {
                    free(spans);
                    return 0;
                }
                spans = next;
            }
            spans[count].start = start;
            spans[count].len = i - start + 1;
            count++;
            start = i + 1;
        }
    }
    if (start < len) {
        if (count == cap) {
            cap *= 2;
            line_span_t *next = realloc(spans, cap * sizeof(line_span_t));
            if (!next) {
                free(spans);
                return 0;
            }
            spans = next;
        }
        spans[count].start = start;
        spans[count].len = len - start;
        count++;
    }

    *out_spans = spans;
    return count;
}

static int patch_fixed_layout_file(const char *path,
                                   const char *old_data, size_t old_len,
                                   const char *new_data, size_t new_len) {
    if (!path || !old_data || !new_data) return 0;
    if (old_len == 0 || new_len == 0) return 0;
    if (old_len != new_len) return 0;

    line_span_t *old_spans = NULL;
    line_span_t *new_spans = NULL;
    size_t old_count = build_line_spans(old_data, old_len, &old_spans);
    size_t new_count = build_line_spans(new_data, new_len, &new_spans);
    if (old_count == 0 || new_count == 0 || old_count != new_count) {
        free(old_spans);
        free(new_spans);
        return 0;
    }

    for (size_t i = 0; i < old_count; ++i) {
        if (old_spans[i].len != new_spans[i].len) {
            free(old_spans);
            free(new_spans);
            return 0;
        }
    }

    FILE *fp = fopen(path, "r+b");
    if (!fp) {
        free(old_spans);
        free(new_spans);
        return 0;
    }

    for (size_t i = 0; i < old_count; ++i) {
        if (memcmp(old_data + old_spans[i].start, new_data + new_spans[i].start, old_spans[i].len) == 0) {
            continue;
        }
        if (fseek(fp, (long)old_spans[i].start, SEEK_SET) != 0) {
            fclose(fp);
            free(old_spans);
            free(new_spans);
            return 0;
        }
        if (fwrite(new_data + new_spans[i].start, 1, new_spans[i].len, fp) != new_spans[i].len) {
            fclose(fp);
            free(old_spans);
            free(new_spans);
            return 0;
        }
    }

    fclose(fp);
    free(old_spans);
    free(new_spans);
    return 1;
}

static int patch_file_linewise(const char *path, const char *old_data, size_t old_len,
                               const char *new_data, size_t new_len, int *patched_lines) {
    *patched_lines = 0;
    if (!path || !new_data || new_len == 0) return 0;
    if (!old_data) {
        old_data = "";
        old_len = 0;
    }

    line_span_t *old_spans = NULL;
    line_span_t *new_spans = NULL;
    size_t old_count = build_line_spans(old_data, old_len, &old_spans);
    size_t new_count = build_line_spans(new_data, new_len, &new_spans);
    if (new_count == 0) {
        free(old_spans);
        free(new_spans);
        return 0;
    }

    // Build a key->line reference table from latest (/tmp) data.
    param_line_ref_t *refs = calloc(new_count, sizeof(param_line_ref_t));
    if (!refs) {
        free(old_spans);
        free(new_spans);
        return 0;
    }
    size_t ref_count = 0;
    for (size_t i = 0; i < new_count; ++i) {
        const char *line_ptr = new_data + new_spans[i].start;
        size_t line_len = new_spans[i].len;
        char key[64];
        if (!parse_param_key_from_line(line_ptr, line_len, key, sizeof(key))) continue;

        int idx = param_ref_find(refs, ref_count, key);
        if (idx < 0) {
            idx = (int)ref_count++;
            snprintf(refs[idx].key, sizeof(refs[idx].key), "%s", key);
        }
        refs[idx].line_ptr = line_ptr;
        refs[idx].line_len = line_len;
        refs[idx].used = false;
    }

    // Build merged text preserving old structure; only differing key lines get replaced.
    char *merged = NULL;
    size_t merged_len = 0;
    size_t merged_cap = 0;
    int diff_count = 0;

    for (size_t i = 0; i < old_count; ++i) {
        const char *old_line = old_data + old_spans[i].start;
        size_t old_line_len = old_spans[i].len;
        const char *line_to_write = old_line;
        size_t line_len_to_write = old_line_len;

        char key[64];
        if (parse_param_key_from_line(old_line, old_line_len, key, sizeof(key))) {
            int idx = param_ref_find(refs, ref_count, key);
            if (idx >= 0) {
                refs[idx].used = true;
                line_to_write = refs[idx].line_ptr;
                line_len_to_write = refs[idx].line_len;
                if (old_line_len != line_len_to_write ||
                    memcmp(old_line, line_to_write, old_line_len) != 0) {
                    diff_count++;
                }
            }
        }

        if (!append_bytes(&merged, &merged_len, &merged_cap, line_to_write, line_len_to_write)) {
            free(merged);
            free(refs);
            free(old_spans);
            free(new_spans);
            return 0;
        }
    }

    // Append brand-new parameter lines that were not present in old file.
    for (size_t i = 0; i < ref_count; ++i) {
        if (refs[i].used) continue;
        if (merged_len > 0 && merged[merged_len - 1] != '\n') {
            if (!append_bytes(&merged, &merged_len, &merged_cap, "\n", 1)) {
                free(merged);
                free(refs);
                free(old_spans);
                free(new_spans);
                return 0;
            }
        }
        if (!append_bytes(&merged, &merged_len, &merged_cap, refs[i].line_ptr, refs[i].line_len)) {
            free(merged);
            free(refs);
            free(old_spans);
            free(new_spans);
            return 0;
        }
        diff_count++;
    }

    if (diff_count == 0) {
        free(merged);
        free(refs);
        free(old_spans);
        free(new_spans);
        return 1;
    }

    if (!patch_fixed_layout_file(path, old_data, old_len, merged, merged_len) &&
        !write_raw_file(path, merged, merged_len)) {
        free(merged);
        free(refs);
        free(old_spans);
        free(new_spans);
        return 0;
    }

    free(merged);
    free(refs);
    free(old_spans);
    free(new_spans);
    *patched_lines = diff_count;
    return 1;
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_SCAN_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_SCAN_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    const char *target_ip = mav_cfg_get_str(MAV_CFG_KEY_SCAN_TARGET_IP, DEFAULT_TARGET_IP);
    int target_port = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_PORT, DEFAULT_TARGET_PORT);
    const char *serial_device = NULL;
    int serial_baud = mav_cfg_get_int(MAV_CFG_KEY_SCAN_SERIAL_BAUD, DEFAULT_SERIAL_BAUD);
    const char *output_path = mav_cfg_get_str(
        MAV_CFG_KEY_PARAMS_TMP_PATH,
        mav_cfg_get_str(MAV_CFG_KEY_LEGACY_PARAMS_TMP_PATH, DEFAULT_OUTPUT));
    const char *persistent_path = mav_cfg_get_str(
        MAV_CFG_KEY_PARAMS_PERSIST_PATH,
        mav_cfg_get_str(MAV_CFG_KEY_LEGACY_PARAMS_PERSIST_PATH, DEFAULT_PERSISTENT_OUTPUT));
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_SYS, DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_COMP, DEFAULT_TARGET_COMP);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_SCAN_SYSID, DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_SCAN_COMPID, DEFAULT_COMPID);
    double scan_duration = mav_cfg_get_double(MAV_CFG_KEY_SCAN_DURATION_SEC, DEFAULT_SCAN_DURATION_SEC);
    double param_timeout = mav_cfg_get_double(MAV_CFG_KEY_SCAN_PARAM_TIMEOUT_SEC, DEFAULT_PARAM_TIMEOUT_SEC);
    bool enable_scan = true;
    bool enable_params = true;
    bool scan_requested = true;
    bool auto_target_sys = true;
    bool auto_target_comp = true;
    bool wait_for_autopilot = false;
    bool wait_full = false;
    bool allow_full_rewrite = false;
    bool upload_params = false;
    const char *upload_path = NULL;
    bool auto_target_reject_reported = false;
    int param_req_recv = 0;
    bool auto_target_addr = false;
    bool fetch_all_targets = false;
    bool odom_seen = false;
    bool odom_seen_reported = false;
    int odom_request_sent = 0;
    double odom_last_request = 0.0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-q") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            target_ip = argv[++i];
            auto_target_addr = false;
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            target_port = atoi(argv[++i]);
            auto_target_addr = false;
        } else if (strcmp(argv[i], "-S") == 0 && i + 1 < argc) {
            serial_device = argv[++i];
            auto_target_addr = false;
        } else if (strcmp(argv[i], "-B") == 0 && i + 1 < argc) {
            serial_baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_path = argv[++i];
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
            auto_target_sys = false;
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
            auto_target_comp = false;
        } else if (strcmp(argv[i], "--autopilot") == 0) {
            wait_for_autopilot = true;
        } else if (strcmp(argv[i], "--wait-full") == 0) {
            wait_full = true;
        } else if (strcmp(argv[i], "--allow-full-rewrite") == 0) {
            allow_full_rewrite = true;
        } else if (strcmp(argv[i], "--upload") == 0) {
            upload_params = true;
        } else if (strcmp(argv[i], "--upload-path") == 0 && i + 1 < argc) {
            upload_params = true;
            upload_path = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            scan_duration = atof(argv[++i]);
        } else if (strcmp(argv[i], "-D") == 0 && i + 1 < argc) {
            param_timeout = atof(argv[++i]);
        } else if (strcmp(argv[i], "--no-scan") == 0) {
            enable_scan = false;
            scan_requested = false;
        } else if (strcmp(argv[i], "--no-params") == 0) {
            enable_params = false;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0], listen_ip, listen_port, target_ip, target_port, serial_baud, output_path,
                  target_sys, target_comp, sysid, compid);
            return 0;
        } else {
            usage(argv[0], listen_ip, listen_port, target_ip, target_port, serial_baud, output_path,
                  target_sys, target_comp, sysid, compid);
            return 1;
        }
    }

    if (!enable_scan && !enable_params) {
        fprintf(stderr, "Nothing to do: both scan and params are disabled.\n");
        return 1;
    }

    bool serial_mode = (serial_device != NULL);
    int io_fd = -1;
    if (serial_mode) {
        io_fd = open_serial_port(serial_device, serial_baud);
        if (io_fd < 0) return 1;
    } else {
        io_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (io_fd < 0) {
            perror("socket");
            return 1;
        }

        int reuse = 1;
        setsockopt(io_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in listen_addr;
        memset(&listen_addr, 0, sizeof(listen_addr));
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons(listen_port);
        if (inet_pton(AF_INET, listen_ip, &listen_addr.sin_addr) != 1) {
            fprintf(stderr, "Invalid listen IP: %s\n", listen_ip);
            close(io_fd);
            return 1;
        }

        if (bind(io_fd, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) != 0) {
            fprintf(stderr, "bind(%s:%d) failed: %s\n", listen_ip, listen_port, strerror(errno));
            close(io_fd);
            return 1;
        }
    }

    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_port);
    if (inet_pton(AF_INET, target_ip, &target_addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid target IP: %s\n", target_ip);
        close(io_fd);
        return 1;
    }

    topic_list_t topics = {.items = NULL, .count = 0};
    param_source_list_t sources = {.items = NULL, .count = 0};

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double start = monotonic_seconds();
    fetch_all_targets = (auto_target_sys && auto_target_comp);

    if (enable_params && !fetch_all_targets && !(auto_target_sys || auto_target_comp)) {
        bool added = false;
        param_source_t *src = source_get_or_add(&sources, (uint8_t)target_sys, (uint8_t)target_comp,
                                                serial_mode ? NULL : &target_addr,
                                                serial_mode ? "serial:manual-target" : "udp:manual-target",
                                                &added);
        if (!src) {
            fprintf(stderr, "%sPARAM%s failed to create target source slot\n", CLR_RED, CLR_RESET);
            topics_free(&topics);
            close(io_fd);
            return 1;
        }
        (void)added;
    }

    if (enable_params) {
        if (fetch_all_targets) {
            printf("%sPARAM%s Waiting for HEARTBEAT; requesting params from all non-self sources\n",
                   CLR_YELLOW, CLR_RESET);
        } else if (auto_target_sys || auto_target_comp) {
            printf("%sPARAM%s Waiting for HEARTBEAT to auto-detect target (current tsys=%d tcomp=%d)\n",
                   CLR_YELLOW, CLR_RESET, target_sys, target_comp);
        } else {
            if (serial_mode) {
                printf("%sPARAM%s Requesting parameters over serial %s@%d (tsys=%d tcomp=%d)\n",
                       CLR_YELLOW, CLR_RESET, serial_device, serial_baud, target_sys, target_comp);
            } else {
                printf("%sPARAM%s Requesting parameters from %s:%d (tsys=%d tcomp=%d)\n",
                       CLR_YELLOW, CLR_RESET, target_ip, target_port, target_sys, target_comp);
            }
        }
    }
    if (enable_scan) {
        if (serial_mode) {
            printf("%sTOPIC%s Scanning MAVLink topics on serial %s@%d for %.1fs\n",
                   CLR_CYAN, CLR_RESET, serial_device, serial_baud, scan_duration);
        } else {
            printf("%sTOPIC%s Scanning MAVLink topics on %s:%d for %.1fs\n",
                   CLR_CYAN, CLR_RESET, listen_ip, listen_port, scan_duration);
        }
    }

    bool scan_reported = false;
    bool scan_extend_reported = false;
    double next_progress_print = 0.0;
    bool inline_progress = isatty(STDOUT_FILENO);
    bool progress_line_active = false;

    while (1) {
        double now = monotonic_seconds();
        double elapsed = now - start;

        bool base_scan_done = !enable_scan || (scan_duration >= 0 && (now - start) >= scan_duration);
        bool params_timed_out = (param_timeout >= 0 && (now - start) >= param_timeout);
        bool params_all_done = (enable_params && sources.count > 0);
        for (size_t si = 0; si < sources.count; ++si) {
            param_source_t *src = &sources.items[si];
            bool src_done = false;
            if (src->param_count > 0 && src->received_count >= src->param_count) {
                src_done = true;
            } else if (src->exhausted) {
                src_done = true;
            } else if (src->param_count <= 0 && src->list_sent_once &&
                       src->last_list_req > 0.0 &&
                       (now - src->last_list_req) >= PARAM_UNKNOWN_SOURCE_IDLE_DONE_SEC &&
                       (now - src->last_param_recv) >= PARAM_UNKNOWN_SOURCE_IDLE_DONE_SEC) {
                src_done = true;
            } else if (!src->source_reported &&
                       src->list_req_sent >= PARAM_INITIAL_LIST_MAX_SEND &&
                       src->last_list_req > 0.0 &&
                       (now - src->last_list_req) >= PARAM_INITIAL_LIST_RETRY_SEC) {
                src_done = true;
            }

            if (!src_done) {
                params_all_done = false;
                break;
            }
        }
        bool params_done = false;
        if (!enable_params) {
            params_done = true;
        } else if (params_all_done) {
            params_done = true;
        } else if (!wait_full && params_timed_out) {
            params_done = true;
        } else if (wait_full && params_timed_out && sources.count == 0) {
            params_done = true;
        }

        bool scan_extended = (enable_scan && base_scan_done && enable_params && !params_done);
        bool scan_done = base_scan_done && !scan_extended;

        if (scan_extended && !scan_extend_reported) {
            clear_progress_line(inline_progress, &progress_line_active);
            printf("%sTOPIC%s Scan duration reached; continuing topic scan while parameter download is in progress\n",
                   CLR_CYAN, CLR_RESET);
            scan_extend_reported = true;
        }

        if (!scan_reported && scan_requested && scan_done) {
            scan_reported = true;
            enable_scan = false;
        }

        if (scan_done && params_done) break;

        if (enable_params && !params_done && now >= next_progress_print) {
            print_param_progress(&sources, elapsed, params_timed_out, inline_progress, &progress_line_active);
            next_progress_print = now + PARAM_PROGRESS_PRINT_SEC;
        }

        if (enable_params && !params_done) {
            bool have_targets = (sources.count > 0);
            if (!have_targets && fetch_all_targets) {
                // Wait until at least one non-self HEARTBEAT source appears.
            }

            for (size_t si = 0; si < sources.count; ++si) {
                param_source_t *src = &sources.items[si];
                const struct sockaddr_in *req_addr = (serial_mode || !src->has_addr) ? &target_addr : &src->addr;

                bool should_send_list = (!src->list_sent_once);
                if (should_send_list) {
                    send_param_request_list(io_fd, serial_mode, req_addr, (uint8_t)sysid, (uint8_t)compid,
                                            src->sysid, src->compid);
                    src->list_sent_once = true;
                    src->last_list_req = now;
                    src->list_req_sent++;

                    clear_progress_line(inline_progress, &progress_line_active);
                    if (serial_mode) {
                        printf("%sPARAM%s Sent PARAM_REQUEST_LIST #%d -> serial:%s@%d (tsys=%u tcomp=%u)\n",
                               CLR_YELLOW, CLR_RESET, src->list_req_sent, serial_device, serial_baud, src->sysid, src->compid);
                    } else {
                        char req_dst[64];
                        format_addr(req_addr, req_dst, sizeof(req_dst));
                        printf("%sPARAM%s Sent PARAM_REQUEST_LIST #%d -> %s (tsys=%u tcomp=%u)\n",
                               CLR_YELLOW, CLR_RESET, src->list_req_sent, req_dst, src->sysid, src->compid);
                    }
                }

                if (src->param_count > 0 && src->received_count < src->param_count &&
                    src->received && src->read_retry_count &&
                    (now - src->last_param_recv) >= PARAM_MISSING_TRIGGER_IDLE_SEC &&
                    (src->last_missing_req <= 0.0 || (now - src->last_missing_req) >= PARAM_MISSING_RETRY_INTERVAL_SEC)) {
                    int batch_sent = 0;
                    for (int checked = 0; checked < src->param_count && batch_sent < PARAM_MISSING_BATCH_SIZE; ++checked) {
                        int idx = (src->missing_cursor + checked) % src->param_count;
                        if (src->received[idx]) continue;
                        if (src->read_retry_count[idx] >= PARAM_MISSING_MAX_RETRY_PER_INDEX) continue;

                        send_param_request_read(io_fd, serial_mode, req_addr, (uint8_t)sysid, (uint8_t)compid,
                                                src->sysid, src->compid, (int16_t)idx);
                        src->read_retry_count[idx]++;
                        batch_sent++;
                        src->missing_cursor = (idx + 1) % src->param_count;
                    }

                    int missing_total = count_missing_params(src->received, src->param_count);
                    int retryable_missing = 0;
                    for (int i = 0; i < src->param_count; ++i) {
                        if (!src->received[i] && src->read_retry_count[i] < PARAM_MISSING_MAX_RETRY_PER_INDEX) {
                            retryable_missing++;
                        }
                    }

                    if (batch_sent > 0) {
                        src->last_missing_req = now;
                        clear_progress_line(inline_progress, &progress_line_active);
                        printf("%sPARAM%s Re-requesting %d missing index parameter(s), remaining %d/%d (tsys=%u tcomp=%u)\n",
                               CLR_YELLOW, CLR_RESET, batch_sent, missing_total, src->param_count, src->sysid, src->compid);
                    } else if (missing_total > 0 && retryable_missing == 0) {
                        clear_progress_line(inline_progress, &progress_line_active);
                        fprintf(stderr, "%sPARAM%s Partial parameter set: received %d/%d, retries exhausted (tsys=%u tcomp=%u)\n",
                                CLR_RED, CLR_RESET, src->received_count, src->param_count, src->sysid, src->compid);
                        src->exhausted = true;
                    }
                }
            }
        }

        if (enable_scan && !odom_seen &&
            odom_request_sent < ODOM_STREAM_REQUEST_MAX_SEND &&
            (now - start) >= ODOM_STREAM_REQUEST_DELAY_SEC &&
            (odom_last_request <= 0.0 || (now - odom_last_request) >= ODOM_STREAM_REQUEST_RETRY_SEC)) {
            if (fetch_all_targets) {
                if (sources.count > 0) {
                    int sent_to = 0;
                    for (size_t si = 0; si < sources.count; ++si) {
                        param_source_t *src = &sources.items[si];
                        const struct sockaddr_in *req_addr = (serial_mode || !src->has_addr) ? &target_addr : &src->addr;
                        send_set_message_interval(io_fd, serial_mode, req_addr,
                                                  (uint8_t)sysid, (uint8_t)compid,
                                                  src->sysid, src->compid,
                                                  MAVLINK_MSG_ID_ODOMETRY, ODOM_STREAM_INTERVAL_US);
                        sent_to++;
                    }
                    odom_request_sent++;
                    odom_last_request = now;
                    clear_progress_line(inline_progress, &progress_line_active);
                    printf("%sTOPIC%s Requested ODOMETRY stream %dHz (#%d, targets=%d)\n",
                           CLR_CYAN, CLR_RESET, ODOM_STREAM_HZ, odom_request_sent, sent_to);
                } else {
                    send_set_message_interval(io_fd, serial_mode, &target_addr,
                                              (uint8_t)sysid, (uint8_t)compid,
                                              (uint8_t)target_sys, (uint8_t)target_comp,
                                              MAVLINK_MSG_ID_ODOMETRY, ODOM_STREAM_INTERVAL_US);
                    odom_request_sent++;
                    odom_last_request = now;
                    clear_progress_line(inline_progress, &progress_line_active);
                    printf("%sTOPIC%s Requested ODOMETRY stream %dHz (#%d -> fallback tsys=%d tcomp=%d)\n",
                           CLR_CYAN, CLR_RESET, ODOM_STREAM_HZ, odom_request_sent, target_sys, target_comp);
                }
            } else {
                send_set_message_interval(io_fd, serial_mode, &target_addr,
                                          (uint8_t)sysid, (uint8_t)compid,
                                          (uint8_t)target_sys, (uint8_t)target_comp,
                                          MAVLINK_MSG_ID_ODOMETRY, ODOM_STREAM_INTERVAL_US);
                odom_request_sent++;
                odom_last_request = now;
                clear_progress_line(inline_progress, &progress_line_active);
                printf("%sTOPIC%s Requested ODOMETRY stream %dHz (#%d -> tsys=%d tcomp=%d)\n",
                       CLR_CYAN, CLR_RESET, ODOM_STREAM_HZ, odom_request_sent, target_sys, target_comp);
            }
        }

        struct timeval tv;
        tv.tv_sec = (time_t)SELECT_TIMEOUT_SEC;
        tv.tv_usec = (suseconds_t)((SELECT_TIMEOUT_SEC - tv.tv_sec) * 1e6);

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(io_fd, &rfds);

        int ready = select(io_fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (ready == 0) continue;

        unsigned char buf[2048];
        struct sockaddr_in src_addr;
        memset(&src_addr, 0, sizeof(src_addr));
        ssize_t n = 0;
        if (serial_mode) {
            n = read(io_fd, buf, sizeof(buf));
        } else {
            socklen_t src_len = sizeof(src_addr);
            n = recvfrom(io_fd, buf, sizeof(buf), 0, (struct sockaddr *)&src_addr, &src_len);
        }
        if (n < 0) {
            if (errno == EINTR) continue;
            if (serial_mode && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            perror(serial_mode ? "read(serial)" : "recvfrom");
            break;
        }
        if (n == 0) continue;
        char src_text[64];
        if (serial_mode) {
            snprintf(src_text, sizeof(src_text), "serial:%s", serial_device);
        } else {
            format_addr(&src_addr, src_text, sizeof(src_text));
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) continue;

            if (!odom_seen && message.msgid == MAVLINK_MSG_ID_ODOMETRY) {
                odom_seen = true;
            }

            if (enable_scan) {
                const char *name = NULL;
#if (MAVLINK_USE_MESSAGE_INFO) && !defined(MAVLINK_NO_HELPERS)
                const mavlink_message_info_t *info = mavlink_get_message_info_by_id(message.msgid);
                if (info && info->name) name = info->name;
#endif
                char fallback[32];
                if (!name) {
                    snprintf(fallback, sizeof(fallback), "MSG_ID_%u", message.msgid);
                    name = fallback;
                }
                topic_add(&topics, name);
            }

            if (odom_seen && !odom_seen_reported) {
                clear_progress_line(inline_progress, &progress_line_active);
                printf("%sTOPIC%s ODOMETRY stream detected\n", CLR_GREEN, CLR_RESET);
                odom_seen_reported = true;
            }

            if (enable_params && message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&message, &hb);

                bool is_self = (message.sysid == (uint8_t)sysid && message.compid == (uint8_t)compid);
                if (fetch_all_targets) {
                    bool looks_like_autopilot = (message.compid == MAV_COMP_ID_AUTOPILOT1);
                    if (is_self) {
                        if (!auto_target_reject_reported) {
                            clear_progress_line(inline_progress, &progress_line_active);
                            printf("%sPARAM%s Ignoring self HEARTBEAT: tsys=%u tcomp=%u (src %s)\n",
                                   CLR_YELLOW, CLR_RESET, message.sysid, message.compid, src_text);
                            auto_target_reject_reported = true;
                        }
                    } else if (wait_for_autopilot && !looks_like_autopilot) {
                        if (!auto_target_reject_reported) {
                            clear_progress_line(inline_progress, &progress_line_active);
                            printf("%sPARAM%s Ignoring non-autopilot HEARTBEAT: tsys=%u tcomp=%u type=%u autopilot=%u (src %s)\n",
                                   CLR_YELLOW, CLR_RESET, message.sysid, message.compid, hb.type, hb.autopilot, src_text);
                            auto_target_reject_reported = true;
                        }
                    } else {
                        bool added = false;
                        param_source_t *src = source_get_or_add(&sources, message.sysid, message.compid,
                                                                serial_mode ? NULL : &src_addr, src_text, &added);
                        if (src && added) {
                            clear_progress_line(inline_progress, &progress_line_active);
                            printf("%sPARAM%s Discovered source: tsys=%u tcomp=%u type=%u autopilot=%u (src %s)\n",
                                   CLR_YELLOW, CLR_RESET, src->sysid, src->compid, hb.type, hb.autopilot, src->src_text);
                        }
                    }
                } else if (auto_target_sys || auto_target_comp) {
                    bool looks_like_autopilot = (message.compid == MAV_COMP_ID_AUTOPILOT1);
                    if (is_self || !looks_like_autopilot) {
                        if (!auto_target_reject_reported) {
                            clear_progress_line(inline_progress, &progress_line_active);
                            printf("%sPARAM%s Ignoring HEARTBEAT during auto-target: tsys=%u tcomp=%u type=%u autopilot=%u (src %s)\n",
                                   CLR_YELLOW, CLR_RESET, message.sysid, message.compid, hb.type, hb.autopilot, src_text);
                            auto_target_reject_reported = true;
                        }
                    } else {
                        if (auto_target_sys) target_sys = message.sysid;
                        if (auto_target_comp) target_comp = message.compid;
                        if (auto_target_addr && !serial_mode) {
                            target_addr = src_addr;
                        }

                        bool added = false;
                        param_source_t *src = source_get_or_add(&sources, (uint8_t)target_sys, (uint8_t)target_comp,
                                                                serial_mode ? NULL : &target_addr, src_text, &added);
                        if (src && added) {
                            clear_progress_line(inline_progress, &progress_line_active);
                            printf("%sPARAM%s Auto-detected target from HEARTBEAT: tsys=%u tcomp=%u (src %s)\n",
                                   CLR_YELLOW, CLR_RESET, src->sysid, src->compid, src->src_text);
                        }
                    }
                }
            }

            if (message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
                param_req_recv++;
                if (param_req_recv <= 3) {
                    clear_progress_line(inline_progress, &progress_line_active);
                    printf("%sPARAM%s Received PARAM_REQUEST_LIST from sysid=%u compid=%u (src %s)\n",
                           CLR_MAGENTA, CLR_RESET, message.sysid, message.compid, src_text);
                } else if (param_req_recv == 4) {
                    clear_progress_line(inline_progress, &progress_line_active);
                    printf("%sPARAM%s Received PARAM_REQUEST_LIST ... (further messages suppressed)\n",
                           CLR_MAGENTA, CLR_RESET);
                }
            }

            if (enable_params && message.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
                mavlink_param_value_t pv;
                mavlink_msg_param_value_decode(&message, &pv);

                char name[17];
                memcpy(name, pv.param_id, 16);
                name[16] = '\0';
                if (name[0] == '\0') continue;

                param_source_t *src = source_get_or_add(&sources, message.sysid, message.compid,
                                                        serial_mode ? NULL : &src_addr, src_text, NULL);
                if (!src) {
                    clear_progress_line(inline_progress, &progress_line_active);
                    fprintf(stderr, "%sPARAM%s Failed to allocate source slot for PARAM_VALUE tsys=%u tcomp=%u\n",
                            CLR_RED, CLR_RESET, message.sysid, message.compid);
                    continue;
                }

                if (pv.param_count > 0) {
                    if (src->param_count < 0 || src->param_count != pv.param_count) {
                        src->param_count = pv.param_count;
                        if ((size_t)src->param_count != src->received_len) {
                            bool *new_received = calloc((size_t)src->param_count, sizeof(bool));
                            uint8_t *new_retry = calloc((size_t)src->param_count, sizeof(uint8_t));
                            if (!new_received || !new_retry) {
                                clear_progress_line(inline_progress, &progress_line_active);
                                fprintf(stderr, "%sPARAM%s Memory allocation failed for parameter tracking (%d)\n",
                                        CLR_RED, CLR_RESET, src->param_count);
                                free(new_received);
                                free(new_retry);
                                src->exhausted = true;
                            } else {
                                free(src->received);
                                free(src->read_retry_count);
                                src->received = new_received;
                                src->read_retry_count = new_retry;
                                src->received_len = (size_t)src->param_count;
                                src->received_count = 0;
                                src->missing_cursor = 0;
                                src->exhausted = false;
                            }
                        }
                    }
                }

                if (src->param_count > 0 && pv.param_index < src->param_count && src->received) {
                    if (!src->received[pv.param_index]) {
                        src->received[pv.param_index] = true;
                        src->received_count++;
                    }
                }

                params_add_or_update(&src->params, name, (int)pv.param_index, pv.param_type, pv.param_value);
                src->last_param_recv = now;

                if (!src->source_reported) {
                    clear_progress_line(inline_progress, &progress_line_active);
                    printf("%sPARAM%s PARAM_VALUE received from %s (sysid=%u compid=%u)\n",
                           CLR_GREEN, CLR_RESET, src->src_text, src->sysid, src->compid);
                    src->source_reported = true;
                }
            }
        }
    }

    clear_progress_line(inline_progress, &progress_line_active);

    if (enable_params) {
        print_section("PARAM", CLR_YELLOW);
        size_t total_param_rows = 0;
        size_t active_sources = 0;
        for (size_t si = 0; si < sources.count; ++si) {
            if (sources.items[si].params.count > 0) {
                total_param_rows += sources.items[si].params.count;
                active_sources++;
            }
        }

        if (total_param_rows == 0) {
            fprintf(stderr, "%sNo parameters received.%s\n", CLR_RED, CLR_RESET);
        } else {
            for (size_t si = 0; si < sources.count; ++si) {
                if (sources.items[si].params.count == 0) continue;
                if (sources.items[si].param_count > 0 && sources.items[si].received_count < sources.items[si].param_count) {
                    fprintf(stderr, "%sPartial parameter set for tsys=%u tcomp=%u: received %d/%d%s\n",
                            CLR_YELLOW,
                            sources.items[si].sysid,
                            sources.items[si].compid,
                            sources.items[si].received_count,
                            sources.items[si].param_count,
                            CLR_RESET);
                }
                qsort(sources.items[si].params.items, sources.items[si].params.count, sizeof(param_entry_t), compare_entries);
            }

            char *params_text = NULL;
            size_t params_text_len = 0;
            if (!build_params_text(&params_text, &params_text_len, &sources, target_ip, target_port, target_sys, target_comp)) {
                fprintf(stderr, "%sFailed to build parameter text buffer%s\n", CLR_RED, CLR_RESET);
                source_list_free(&sources);
                topics_free(&topics);
                close(io_fd);
                return 1;
            }

            if (!write_raw_file(output_path, params_text, params_text_len)) {
                fprintf(stderr, "%sFailed to write %s: %s%s\n", CLR_RED, output_path, strerror(errno), CLR_RESET);
            } else {
                printf("%sSaved %zu params from %zu source(s) to %s%s\n",
                       CLR_GREEN, total_param_rows, active_sources, output_path, CLR_RESET);
            }

            if (strcmp(output_path, persistent_path) != 0) {
                char *old_persistent = NULL;
                size_t old_persistent_len = 0;
                if (!read_raw_file(persistent_path, &old_persistent, &old_persistent_len)) {
                    fprintf(stderr, "%sFailed to read %s: %s%s\n", CLR_RED, persistent_path, strerror(errno), CLR_RESET);
                } else {
                    if (old_persistent_len == params_text_len &&
                        (params_text_len == 0 || memcmp(old_persistent, params_text, params_text_len) == 0)) {
                        printf("%sPersistent file unchanged, skipped disk write: %s%s\n",
                               CLR_CYAN, persistent_path, CLR_RESET);
                    } else {
                        int patched_lines = 0;
                        if (patch_file_linewise(persistent_path, old_persistent, old_persistent_len,
                                                params_text, params_text_len, &patched_lines)) {
                            if (patched_lines > 0) {
                                printf("%sPatched %d changed line(s) in %s%s\n",
                                       CLR_GREEN, patched_lines, persistent_path, CLR_RESET);
                            } else {
                                printf("%sPersistent file already up to date: %s%s\n",
                                       CLR_CYAN, persistent_path, CLR_RESET);
                            }
                        } else {
                            if (allow_full_rewrite) {
                                if (!write_raw_file(persistent_path, params_text, params_text_len)) {
                                    fprintf(stderr, "%sFailed to write %s: %s%s\n",
                                            CLR_RED, persistent_path, strerror(errno), CLR_RESET);
                                } else {
                                    printf("%sUpdated persistent snapshot (full rewrite): %s%s\n",
                                           CLR_GREEN, persistent_path, CLR_RESET);
                                }
                            } else {
                                fprintf(stderr,
                                        "%sPersistent snapshot patch not applicable (line layout changed); "
                                        "skipped full rewrite by policy: %s%s\n",
                                        CLR_YELLOW, persistent_path, CLR_RESET);
                            }
                        }
                    }
                }
                free(old_persistent);
            }

            if (upload_params) {
                const char *desired_path = upload_path ? upload_path : persistent_path;
                param_source_t *primary = select_primary_source(&sources, target_sys, target_comp);
                if (!primary || primary->params.count == 0) {
                    fprintf(stderr, "%sPARAM%s Upload skipped: no live parameter source available%s\n",
                            CLR_RED, CLR_RESET, CLR_RESET);
                } else {
                    param_list_t desired;
                    param_list_t current_tmp;
                    params_init(&desired);
                    params_init(&current_tmp);
                    params_load_text_file(&desired, desired_path);
                    params_load_text_file(&current_tmp, output_path);

                    int sent = 0;
                    int skipped_same = 0;
                    int skipped_missing = 0;
                    for (size_t i = 0; i < desired.count; ++i) {
                        const char *name = desired.items[i].name;
                        double desired_val = desired.items[i].raw_value;

                        double tmp_val = 0.0;
                        uint8_t tmp_type = MAV_PARAM_TYPE_REAL32;
                        if (params_get_value(&current_tmp, name, &tmp_val, &tmp_type)) {
                            if (!values_differ(tmp_type, desired_val, tmp_val)) {
                                skipped_same++;
                                continue;
                            }
                        }

                        double live_val = 0.0;
                        uint8_t live_type = MAV_PARAM_TYPE_REAL32;
                        if (!params_get_value(&primary->params, name, &live_val, &live_type)) {
                            skipped_missing++;
                            continue;
                        }
                        if (!values_differ(live_type, desired_val, live_val)) {
                            skipped_same++;
                            continue;
                        }

                        send_param_set(io_fd, serial_mode, &target_addr,
                                       (uint8_t)sysid, (uint8_t)compid,
                                       (uint8_t)primary->sysid, (uint8_t)primary->compid,
                                       name, live_type, desired_val);
                        sent++;
                    }

                    if (sent > 0) {
                        printf("%sPARAM%s Upload queued: %d changed parameter(s) from %s%s\n",
                               CLR_GREEN, CLR_RESET, sent, desired_path, CLR_RESET);
                    } else {
                        printf("%sPARAM%s Upload skipped: no differences found (%d missing, %d unchanged)%s\n",
                               CLR_CYAN, CLR_RESET, skipped_missing, skipped_same, CLR_RESET);
                    }

                    params_free(&desired);
                    params_free(&current_tmp);
                }
            }
            free(params_text);
        }
    }

    if (scan_requested) {
        clear_progress_line(inline_progress, &progress_line_active);
        print_section("TOPIC LIST", CLR_CYAN);
        if (topics.count == 0) {
            printf("%sNo MAVLink messages received during scan.%s\n", CLR_RED, CLR_RESET);
        } else {
            qsort(topics.items, topics.count, sizeof(char *), compare_strings);
            for (size_t i = 0; i < topics.count; ++i) {
                printf("%s- %s%s\n", CLR_CYAN, topics.items[i], CLR_RESET);
            }
        }
    }

    source_list_free(&sources);
    topics_free(&topics);
    close(io_fd);
    return 0;
}
