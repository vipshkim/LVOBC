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
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>
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
#define DEFAULT_PARAM_TIMEOUT_SEC 30.0
#define DEFAULT_SERIAL_BAUD 921600
#define PARAM_INITIAL_LIST_RETRY_SEC 8.0
#define PARAM_INITIAL_LIST_MAX_SEND 2
#define PARAM_MISSING_TRIGGER_IDLE_SEC 1.5
#define PARAM_MISSING_RETRY_INTERVAL_SEC 1.0
#define PARAM_MISSING_BATCH_SIZE 10
#define PARAM_MISSING_MAX_RETRY_PER_INDEX 3
#define SELECT_TIMEOUT_SEC 0.2
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
} param_entry_t;

typedef struct {
    param_entry_t *items;
    size_t count;
} param_list_t;

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

static int params_find(const param_list_t *list, const char *name) {
    for (size_t i = 0; i < list->count; ++i) {
        if (strcmp(list->items[i].name, name) == 0) return (int)i;
    }
    return -1;
}

static void format_param_value(uint8_t type, float value, char *out, size_t out_len) {
    switch (type) {
        case MAV_PARAM_TYPE_UINT8:
            snprintf(out, out_len, "%u", (unsigned)lrintf(value));
            break;
        case MAV_PARAM_TYPE_INT8:
            snprintf(out, out_len, "%d", (int)lrintf(value));
            break;
        case MAV_PARAM_TYPE_UINT16:
            snprintf(out, out_len, "%u", (unsigned)lrintf(value));
            break;
        case MAV_PARAM_TYPE_INT16:
            snprintf(out, out_len, "%d", (int)lrintf(value));
            break;
        case MAV_PARAM_TYPE_UINT32:
            snprintf(out, out_len, "%u", (unsigned)lrintf(value));
            break;
        case MAV_PARAM_TYPE_INT32:
            snprintf(out, out_len, "%d", (int)lrintf(value));
            break;
        case MAV_PARAM_TYPE_REAL64:
            snprintf(out, out_len, "%.9g", (double)value);
            break;
        case MAV_PARAM_TYPE_REAL32:
        default:
            snprintf(out, out_len, "%.9g", (double)value);
            break;
    }
}

static void params_add_or_update(param_list_t *list, const char *name, int index, uint8_t type, float value) {
    char formatted[64];
    format_param_value(type, value, formatted, sizeof(formatted));

    int pos = params_find(list, name);
    if (pos >= 0) {
        list->items[pos].index = index;
        snprintf(list->items[pos].value, sizeof(list->items[pos].value), "%s", formatted);
        return;
    }

    param_entry_t *next = realloc(list->items, (list->count + 1) * sizeof(param_entry_t));
    if (!next) return;
    list->items = next;

    snprintf(list->items[list->count].name, sizeof(list->items[list->count].name), "%s", name);
    list->items[list->count].index = index;
    snprintf(list->items[list->count].value, sizeof(list->items[list->count].value), "%s", formatted);
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
        "  --no-params  : disable parameter fetch\n",
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

static int write_params_file(const char *path, const param_list_t *params,
                             const char *target_ip, int target_port, int target_sys, int target_comp) {
    FILE *fp = fopen(path, "w");
    if (!fp) return 0;
    fprintf(fp, "# MAVLink params saved from %s:%d (tsys=%d tcomp=%d)\n",
            target_ip, target_port, target_sys, target_comp);
    for (size_t i = 0; i < params->count; ++i) {
        fprintf(fp, "%s=%s\n", params->items[i].name, params->items[i].value);
    }
    fclose(fp);
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
    double scan_duration = DEFAULT_SCAN_DURATION_SEC;
    double param_timeout = DEFAULT_PARAM_TIMEOUT_SEC;
    bool enable_scan = true;
    bool enable_params = true;
    bool scan_requested = true;
    bool auto_target_sys = true;
    bool auto_target_comp = true;
    bool auto_target_seen = false;
    bool param_src_reported = false;
    int param_req_sent = 0;
    int param_req_recv = 0;
    bool auto_target_addr = false;

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
    param_list_t params;
    params_init(&params);

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double start = monotonic_seconds();
    double last_list_req = 0.0;
    double last_param_recv = start;
    double last_missing_req = 0.0;
    bool initial_list_sent = false;
    bool params_exhausted = false;

    int param_count = -1;
    int received_count = 0;
    bool *received = NULL;
    uint8_t *read_retry_count = NULL;
    size_t received_len = 0;
    int missing_cursor = 0;

    if (enable_params) {
        if (auto_target_sys || auto_target_comp) {
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

    while (1) {
        double now = monotonic_seconds();

        bool scan_done = !enable_scan || (scan_duration >= 0 && (now - start) >= scan_duration);
        bool params_done = !enable_params || (param_timeout >= 0 && (now - start) >= param_timeout) ||
            (param_count > 0 && received_count >= param_count) || params_exhausted;

        if (!scan_reported && scan_requested && scan_done) {
            print_section("TOPIC LIST", CLR_CYAN);
            if (topics.count == 0) {
                printf("%sNo MAVLink messages received during scan.%s\n", CLR_RED, CLR_RESET);
            } else {
                qsort(topics.items, topics.count, sizeof(char *), compare_strings);
                for (size_t i = 0; i < topics.count; ++i) {
                    printf("%s- %s%s\n", CLR_CYAN, topics.items[i], CLR_RESET);
                }
            }
            fflush(stdout);
            scan_reported = true;
            enable_scan = false;
        }

        if (scan_done && params_done) break;

        if (enable_params && !params_done) {
            if ((auto_target_sys || auto_target_comp) && !auto_target_seen) {
                // Wait for HEARTBEAT to auto-detect target before requesting params.
            } else if (!initial_list_sent ||
                       (!param_src_reported &&
                        param_req_sent < PARAM_INITIAL_LIST_MAX_SEND &&
                        (now - last_list_req) >= PARAM_INITIAL_LIST_RETRY_SEC)) {
                send_param_request_list(io_fd, serial_mode, &target_addr, (uint8_t)sysid, (uint8_t)compid,
                                        (uint8_t)target_sys, (uint8_t)target_comp);
                last_list_req = now;
                param_req_sent++;
                initial_list_sent = true;
                if (serial_mode) {
                    printf("%sPARAM%s Sent PARAM_REQUEST_LIST #%d -> serial:%s@%d (tsys=%d tcomp=%d)\n",
                           CLR_YELLOW, CLR_RESET, param_req_sent, serial_device, serial_baud, target_sys, target_comp);
                } else {
                    char req_dst[64];
                    format_addr(&target_addr, req_dst, sizeof(req_dst));
                    printf("%sPARAM%s Sent PARAM_REQUEST_LIST #%d -> %s (tsys=%d tcomp=%d)\n",
                           CLR_YELLOW, CLR_RESET, param_req_sent, req_dst, target_sys, target_comp);
                }
            } else if (param_count > 0 && received_count < param_count &&
                       received && read_retry_count &&
                       (now - last_param_recv) >= PARAM_MISSING_TRIGGER_IDLE_SEC &&
                       (last_missing_req <= 0.0 || (now - last_missing_req) >= PARAM_MISSING_RETRY_INTERVAL_SEC)) {
                int batch_sent = 0;
                for (int checked = 0; checked < param_count && batch_sent < PARAM_MISSING_BATCH_SIZE; ++checked) {
                    int idx = (missing_cursor + checked) % param_count;
                    if (received[idx]) continue;
                    if (read_retry_count[idx] >= PARAM_MISSING_MAX_RETRY_PER_INDEX) continue;

                    send_param_request_read(io_fd, serial_mode, &target_addr, (uint8_t)sysid, (uint8_t)compid,
                                            (uint8_t)target_sys, (uint8_t)target_comp, (int16_t)idx);
                    read_retry_count[idx]++;
                    batch_sent++;
                    missing_cursor = (idx + 1) % param_count;
                }

                int missing_total = count_missing_params(received, param_count);
                int retryable_missing = 0;
                for (int i = 0; i < param_count; ++i) {
                    if (!received[i] && read_retry_count[i] < PARAM_MISSING_MAX_RETRY_PER_INDEX) {
                        retryable_missing++;
                    }
                }

                if (batch_sent > 0) {
                    last_missing_req = now;
                    printf("%sPARAM%s Re-requesting %d missing index parameter(s), remaining %d/%d\n",
                           CLR_YELLOW, CLR_RESET, batch_sent, missing_total, param_count);
                } else if (missing_total > 0 && retryable_missing == 0) {
                    fprintf(stderr, "%sPARAM%s Partial parameter set: received %d/%d, retries exhausted for missing indices\n",
                            CLR_RED, CLR_RESET, received_count, param_count);
                    params_exhausted = true;
                }
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

            if ((auto_target_sys || auto_target_comp) && !auto_target_seen && message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                if (auto_target_sys) target_sys = message.sysid;
                if (auto_target_comp) target_comp = message.compid;
                auto_target_seen = true;
                printf("%sPARAM%s Auto-detected target from HEARTBEAT: tsys=%d tcomp=%d (src %s)\n",
                       CLR_YELLOW, CLR_RESET, target_sys, target_comp, src_text);
                if (auto_target_addr && !serial_mode) {
                    target_addr = src_addr;
                    printf("%sPARAM%s Using HEARTBEAT source for param requests: %s\n",
                           CLR_YELLOW, CLR_RESET, src_text);
                }
            }

            if (message.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
                param_req_recv++;
                if (param_req_recv <= 3) {
                    printf("%sPARAM%s Received PARAM_REQUEST_LIST from sysid=%u compid=%u (src %s)\n",
                           CLR_MAGENTA, CLR_RESET, message.sysid, message.compid, src_text);
                } else if (param_req_recv == 4) {
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

                if (pv.param_count > 0) {
                    if (param_count < 0 || param_count != pv.param_count) {
                        param_count = pv.param_count;
                        if ((size_t)param_count != received_len) {
                            bool *new_received = calloc((size_t)param_count, sizeof(bool));
                            uint8_t *new_retry = calloc((size_t)param_count, sizeof(uint8_t));
                            if (!new_received || !new_retry) {
                                fprintf(stderr, "%sPARAM%s Memory allocation failed for parameter tracking (%d)\n",
                                        CLR_RED, CLR_RESET, param_count);
                                free(new_received);
                                free(new_retry);
                                params_exhausted = true;
                            } else {
                                free(received);
                                free(read_retry_count);
                                received = new_received;
                                read_retry_count = new_retry;
                                received_len = (size_t)param_count;
                                received_count = 0;
                                missing_cursor = 0;
                                params_exhausted = false;
                            }
                        }
                    }
                }

                if (param_count > 0 && pv.param_index >= 0 && pv.param_index < param_count && received) {
                    if (!received[pv.param_index]) {
                        received[pv.param_index] = true;
                        received_count++;
                    }
                }

                params_add_or_update(&params, name, (int)pv.param_index, pv.param_type, pv.param_value);
                last_param_recv = now;

                if (!param_src_reported) {
                    printf("%sPARAM%s PARAM_VALUE received from %s (sysid=%u compid=%u)\n",
                           CLR_GREEN, CLR_RESET, src_text, message.sysid, message.compid);
                    param_src_reported = true;
                }
            }
        }
    }

    if (scan_requested && !scan_reported) {
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

    if (enable_params) {
        print_section("PARAMETER LIST", CLR_YELLOW);
        if (params.count == 0) {
            fprintf(stderr, "%sNo parameters received.%s\n", CLR_RED, CLR_RESET);
        } else {
            if (param_count > 0 && received_count < param_count) {
                fprintf(stderr, "%sPartial parameter set: received %d/%d%s\n",
                        CLR_YELLOW, received_count, param_count, CLR_RESET);
            }
            qsort(params.items, params.count, sizeof(param_entry_t), compare_entries);

            if (!write_params_file(output_path, &params, target_ip, target_port, target_sys, target_comp)) {
                fprintf(stderr, "%sFailed to write %s: %s%s\n", CLR_RED, output_path, strerror(errno), CLR_RESET);
            } else {
                printf("%sSaved %zu params to %s%s\n", CLR_GREEN, params.count, output_path, CLR_RESET);
            }

            if (strcmp(output_path, persistent_path) != 0) {
                if (!write_params_file(persistent_path, &params, target_ip, target_port, target_sys, target_comp)) {
                    fprintf(stderr, "%sFailed to write %s: %s%s\n", CLR_RED, persistent_path, strerror(errno), CLR_RESET);
                } else {
                    printf("%sSaved %zu params to %s%s\n", CLR_GREEN, params.count, persistent_path, CLR_RESET);
                }
            }
        }
    }

    free(received);
    free(read_retry_count);
    params_free(&params);
    topics_free(&topics);
    close(io_fd);
    return 0;
}
