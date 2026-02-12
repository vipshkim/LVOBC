#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
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
#include <sys/stat.h>
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
#define CLR_CYAN    "\033[36m"

#define DEFAULT_TIMEOUT_SEC 2.5
#define DEFAULT_RETRIES 3
#define PX4_CUSTOM_MAIN_MODE_MANUAL 1

typedef struct {
    const char *name;
    double test_value;
    int optional;
} mode_param_t;

static const mode_param_t k_mode_params[] = {
    // LaunchVehicle test-mode whitelist: only these are altered by `mode test`.
    {"COM_ARM_WO_GPS", 1, 0},
    {"COM_RC_IN_MODE", 5, 0},
    {"NAV_DLL_ACT", 0, 0},
    {"CBRK_IO_SAFETY", 22027, 1},
    {"CBRK_SUPPLY_CHK", 894281, 1},
    {"COM_ARM_HFLT_CHK", 0, 0},
    {"COM_ARM_MAG_ANG", -1, 0},
    {"COM_ARM_MAG_STR", 0, 0},
    {"EKF2_MAG_CHECK", 0, 1},
    {"COM_POS_LOW_ACT", 0, 0},
    {"COM_POS_FS_EPH", -1, 0},
    {"EKF2_GPS_CHECK", 0, 1},
    {"EKF2_REQ_FIX", 0, 1},
    {"EKF2_REQ_NSATS", 0, 1},
    {"EKF2_REQ_EPH", 1000, 1},
    {"EKF2_REQ_EPV", 1000, 1},
};

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void fill_param_id(char *dst, size_t dst_len, const char *name) {
    memset(dst, 0, dst_len);
    size_t n = strlen(name);
    if (n > dst_len) n = dst_len;
    memcpy(dst, name, n);
}

static void default_mode_path(char *out, size_t out_len) {
    const char *cfg = mav_cfg_get_str(MAV_CFG_KEY_MODE_STATE_PATH, NULL);
    if (cfg && cfg[0]) {
        snprintf(out, out_len, "%s", cfg);
        return;
    }
    const char *home = getenv("HOME");
    if (home && home[0]) {
        snprintf(out, out_len, "%s/.config/rocket-mav/mode.env", home);
        return;
    }
    snprintf(out, out_len, "/tmp/rocket-mav-mode.env");
}

static int ensure_parent_dir(const char *path) {
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s", path);
    char *slash = strrchr(tmp, '/');
    if (!slash || slash == tmp) return 0;
    *slash = '\0';
    if (mkdir(tmp, 0755) == 0) return 0;
    if (errno == EEXIST) return 0;
    return -1;
}

static ssize_t send_udp(int fd, const struct sockaddr_in *dest, const uint8_t *buf, size_t len) {
    return sendto(fd, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static int request_param_value_once(int fd, const struct sockaddr_in *dest,
                                    int sysid, int compid, int target_sys, int target_comp,
                                    const char *param_name, double timeout_sec,
                                    mavlink_param_value_t *out_pv) {
    char param_id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN] = {0};
    fill_param_id(param_id, sizeof(param_id), param_name);

    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_param_request_read_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                                        (uint8_t)target_sys, (uint8_t)target_comp,
                                        param_id, -1);
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)send_udp(fd, dest, tx, tx_len);

    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t rmsg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        uint8_t rx[2048];
        ssize_t n = recvfrom(fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &rmsg, &status)) continue;
            if (rmsg.msgid != MAVLINK_MSG_ID_PARAM_VALUE) continue;

            mavlink_param_value_t pv;
            mavlink_msg_param_value_decode(&rmsg, &pv);
            char name[17];
            memcpy(name, pv.param_id, 16);
            name[16] = '\0';
            if (strcmp(name, param_name) != 0) continue;

            if (out_pv) *out_pv = pv;
            return 1;
        }
    }

    return 0;
}

static int set_param_value_once(int fd, const struct sockaddr_in *dest,
                                int sysid, int compid, int target_sys, int target_comp,
                                const char *param_name, uint8_t type, double value) {
    char param_id[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN] = {0};
    fill_param_id(param_id, sizeof(param_id), param_name);

    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    float encoded = rocket_encode_param_value(type, value);
    mavlink_msg_param_set_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                               (uint8_t)target_sys, (uint8_t)target_comp,
                               param_id, encoded, type);
    uint16_t len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)send_udp(fd, dest, tx, len);
    return 1;
}

static int request_heartbeat_once(int fd, int target_sys, int target_comp, double timeout_sec,
                                  mavlink_heartbeat_t *out_hb) {
    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t rmsg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        uint8_t rx[2048];
        ssize_t n = recvfrom(fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &rmsg, &status)) continue;
            if (rmsg.msgid != MAVLINK_MSG_ID_HEARTBEAT) continue;
            if (target_sys > 0 && rmsg.sysid != (uint8_t)target_sys) continue;
            if (target_comp > 0 && rmsg.compid != (uint8_t)target_comp) continue;

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&rmsg, &hb);
            if (out_hb) *out_hb = hb;
            return 1;
        }
    }

    return 0;
}

static int request_global_position_int_once(int fd, int target_sys, int target_comp, double timeout_sec,
                                            mavlink_global_position_int_t *out_gp) {
    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t rmsg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        uint8_t rx[2048];
        ssize_t n = recvfrom(fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &rmsg, &status)) continue;
            if (rmsg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT) continue;
            if (target_sys > 0 && rmsg.sysid != (uint8_t)target_sys) continue;
            if (target_comp > 0 && rmsg.compid != (uint8_t)target_comp) continue;

            mavlink_global_position_int_t gp;
            mavlink_msg_global_position_int_decode(&rmsg, &gp);
            if (out_gp) *out_gp = gp;
            return 1;
        }
    }

    return 0;
}

static int wait_command_ack_once(int fd, uint16_t command, double timeout_sec, uint8_t *out_result) {
    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t rmsg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        uint8_t rx[2048];
        ssize_t n = recvfrom(fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &rmsg, &status)) continue;
            if (rmsg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) continue;
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&rmsg, &ack);
            if (ack.command != command) continue;
            if (out_result) *out_result = ack.result;
            return 1;
        }
    }

    return 0;
}

static void custom_mode_to_main_sub(uint32_t custom_mode, uint8_t *main_mode, uint8_t *sub_mode) {
    if (main_mode) *main_mode = (uint8_t)((custom_mode >> 16) & 0xFFu);
    if (sub_mode) *sub_mode = (uint8_t)((custom_mode >> 24) & 0xFFu);
}

static int send_set_mode_once(int fd, const struct sockaddr_in *dest,
                              int sysid, int compid, int target_sys, int target_comp,
                              uint8_t base_mode, uint8_t custom_main, uint8_t custom_sub) {
    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    const uint8_t bm = (uint8_t)(base_mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);

    mavlink_msg_command_long_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                                  (uint8_t)target_sys, (uint8_t)target_comp,
                                  MAV_CMD_DO_SET_MODE,
                                  0,
                                  (float)bm,
                                  (float)custom_main,
                                  (float)custom_sub,
                                  0.f, 0.f, 0.f, 0.f);
    uint16_t len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)send_udp(fd, dest, tx, len);
    return 1;
}

static int request_message_once(int fd, const struct sockaddr_in *dest,
                                int sysid, int compid, int target_sys, int target_comp,
                                uint32_t message_id) {
    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                                  (uint8_t)target_sys, (uint8_t)target_comp,
                                  MAV_CMD_REQUEST_MESSAGE,
                                  0,
                                  (float)message_id,
                                  0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    uint16_t len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)send_udp(fd, dest, tx, len);
    return 1;
}

static int send_set_home_once(int fd, const struct sockaddr_in *dest,
                              int sysid, int compid, int target_sys, int target_comp,
                              int use_current, double latitude_deg, double longitude_deg, double altitude_m) {
    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                                  (uint8_t)target_sys, (uint8_t)target_comp,
                                  MAV_CMD_DO_SET_HOME,
                                  0,
                                  (float)(use_current ? 1.0 : 0.0),
                                  NAN,  /* roll not set */
                                  NAN,  /* pitch not set */
                                  NAN,  /* yaw not set */
                                  (float)latitude_deg,
                                  (float)longitude_deg,
                                  (float)altitude_m);
    uint16_t len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)send_udp(fd, dest, tx, len);
    return 1;
}

typedef struct {
    char key[128];
    char value[64];
} kv_t;

static int load_mode_file(const char *path, kv_t *out, size_t *count) {
    *count = 0;
    FILE *fp = fopen(path, "r");
    if (!fp) return 0;
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        mav_cfg_trim(line);
        if (line[0] == '\0' || line[0] == '#') continue;
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        mav_cfg_trim(key);
        mav_cfg_trim(val);
        if (key[0] == '\0') continue;
        if (*count >= 256) break;
        snprintf(out[*count].key, sizeof(out[*count].key), "%s", key);
        snprintf(out[*count].value, sizeof(out[*count].value), "%s", val);
        (*count)++;
    }
    fclose(fp);
    return 1;
}

static const char *find_kv(const kv_t *items, size_t count, const char *key) {
    for (size_t i = 0; i < count; ++i) {
        if (strcmp(items[i].key, key) == 0) return items[i].value;
    }
    return NULL;
}

static void kv_set(kv_t *items, size_t *count, const char *key, const char *value) {
    for (size_t i = 0; i < *count; ++i) {
        if (strcmp(items[i].key, key) == 0) {
            snprintf(items[i].value, sizeof(items[i].value), "%s", value);
            return;
        }
    }
    if (*count >= 256) return;
    snprintf(items[*count].key, sizeof(items[*count].key), "%s", key);
    snprintf(items[*count].value, sizeof(items[*count].value), "%s", value);
    (*count)++;
}

static void kv_copy_orig_entries(const kv_t *src, size_t src_count, kv_t *dst, size_t *dst_count) {
    for (size_t i = 0; i < src_count; ++i) {
        if (strncmp(src[i].key, "ORIG_", 5) != 0) continue;
        kv_set(dst, dst_count, src[i].key, src[i].value);
    }
}

static int read_param_from_snapshot(const char *path, const char *param_name, double *out_value) {
    if (!path || !param_name || !out_value) return 0;
    FILE *fp = fopen(path, "r");
    if (!fp) return 0;

    char line[512];
    int found = 0;
    while (fgets(line, sizeof(line), fp)) {
        mav_cfg_trim(line);
        if (line[0] == '\0' || line[0] == '#') continue;
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        mav_cfg_trim(key);
        mav_cfg_trim(val);
        if (strcmp(key, param_name) != 0) continue;

        char *endp = NULL;
        double parsed = strtod(val, &endp);
        if (endp == val || (endp && *endp != '\0')) continue;
        *out_value = parsed;
        found = 1;
        break;
    }
    fclose(fp);
    return found;
}

static int save_mode_file(const char *path, const char *mode,
                          const kv_t *orig, size_t orig_count) {
    if (ensure_parent_dir(path) != 0) return -1;

    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *fp = fopen(tmp, "w");
    if (!fp) return -1;

    fprintf(fp, "# rocket-mav mode state\n");
    fprintf(fp, "MODE=%s\n", mode);
    for (size_t i = 0; i < orig_count; ++i) {
        if (strcmp(orig[i].key, "MODE") == 0) continue;
        fprintf(fp, "%s=%s\n", orig[i].key, orig[i].value);
    }

    if (fclose(fp) != 0) {
        unlink(tmp);
        return -1;
    }
    if (rename(tmp, path) != 0) {
        unlink(tmp);
        return -1;
    }
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s <normal|test|launch> [options]\n"
            "Options:\n"
            "  -f <path>  mode state file path (default ~/.config/rocket-mav/mode.env)\n"
            "  -u <ip>    target ip (default from config)\n"
            "  -p <port>  target port (default from config)\n"
            "  -t <sys>   target sysid (default from config)\n"
            "  -k <comp>  target compid (default from config)\n"
            "  -s <sys>   sender sysid (default from config)\n"
            "  -c <comp>  sender compid (default from config)\n"
            "  -d <sec>   timeout per try (default %.1f)\n"
            "  -r <cnt>   retries (default %d)\n",
            prog, DEFAULT_TIMEOUT_SEC, DEFAULT_RETRIES);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const char *mode = argv[1];
    if (strcmp(mode, "test") != 0 && strcmp(mode, "normal") != 0 && strcmp(mode, "launch") != 0) {
        usage(argv[0]);
        return 1;
    }

    char mode_path[PATH_MAX];
    default_mode_path(mode_path, sizeof(mode_path));

    const char *target_ip = mav_cfg_get_str(MAV_CFG_KEY_SCAN_TARGET_IP, MAV_DEFAULT_IP);
    int target_port = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_PORT, 15641);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    double timeout_sec = mav_cfg_get_double(MAV_CFG_KEY_PARAM_FETCH_TIMEOUT_SEC, DEFAULT_TIMEOUT_SEC);
    int retries = mav_cfg_get_int(MAV_CFG_KEY_PARAM_FETCH_RETRIES, DEFAULT_RETRIES);

    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            snprintf(mode_path, sizeof(mode_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            target_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            timeout_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            retries = atoi(argv[++i]);
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)target_port);
    if (inet_pton(AF_INET, target_ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "Invalid target IP: %s\n", target_ip);
        close(fd);
        return 1;
    }

    kv_t saved[256];
    size_t saved_count = 0;
    load_mode_file(mode_path, saved, &saved_count);
    const char *saved_mode = find_kv(saved, saved_count, "MODE");

    kv_t new_saved[256];
    size_t new_saved_count = 0;
    if (strcmp(mode, "test") == 0 || strcmp(mode, "launch") == 0) {
        kv_copy_orig_entries(saved, saved_count, new_saved, &new_saved_count);
    }
    const char *snapshot_path = NULL;
    if (strcmp(mode, "normal") == 0) {
        snapshot_path = mav_select_params_path();
        if (snapshot_path) {
            fprintf(stdout, "%sPARAM%s restore source: %s\n", CLR_CYAN, CLR_RESET, snapshot_path);
        } else {
            fprintf(stdout, "%sPARAM%s restore source not found; fallback ORIG_* in mode file\n",
                    CLR_YELLOW, CLR_RESET);
        }
    }

    fprintf(stdout, "%sMODE%s %s -> target %s:%d (tsys=%d tcomp=%d)\n",
            CLR_CYAN, CLR_RESET, mode, target_ip, target_port, target_sys, target_comp);

    if (strcmp(mode, "launch") != 0) {
        const char *orig_base = find_kv(saved, saved_count, "ORIG_FLIGHT_BASE");
        const char *orig_main = find_kv(saved, saved_count, "ORIG_FLIGHT_MAIN");
        const char *orig_sub = find_kv(saved, saved_count, "ORIG_FLIGHT_SUB");
        if (orig_base && orig_main && orig_sub && saved_mode && strcmp(saved_mode, "LAUNCH") == 0) {
            uint8_t base = (uint8_t)strtoul(orig_base, NULL, 10);
            uint8_t main_mode = (uint8_t)strtoul(orig_main, NULL, 10);
            uint8_t sub_mode = (uint8_t)strtoul(orig_sub, NULL, 10);
            int restored = 0;
            for (int attempt = 1; attempt <= retries; ++attempt) {
                send_set_mode_once(fd, &dest, sysid, compid, target_sys, target_comp, base, main_mode, sub_mode);
                uint8_t ack = MAV_RESULT_FAILED;
                if (wait_command_ack_once(fd, MAV_CMD_DO_SET_MODE, timeout_sec, &ack) && ack == MAV_RESULT_ACCEPTED) {
                    restored = 1;
                    break;
                }
            }
            if (!restored) {
                fprintf(stderr, "%sWarn%s failed to restore previous flight mode\n", CLR_YELLOW, CLR_RESET);
            } else {
                fprintf(stdout, "%sRESTORE%s FLIGHT_MODE main=%u sub=%u\n",
                        CLR_GREEN, CLR_RESET, (unsigned)main_mode, (unsigned)sub_mode);
            }
        }
    }

    if (strcmp(mode, "launch") == 0) {
        mavlink_heartbeat_t hb;
        int got_hb = 0;
        for (int attempt = 1; attempt <= retries; ++attempt) {
            request_message_once(fd, &dest, sysid, compid, target_sys, target_comp, MAVLINK_MSG_ID_HEARTBEAT);
            if (request_heartbeat_once(fd, target_sys, target_comp, timeout_sec, &hb)) {
                got_hb = 1;
                break;
            }
        }
        if (!got_hb) {
            fprintf(stderr, "%sFailed%s to read heartbeat for launch mode\n", CLR_RED, CLR_RESET);
            close(fd);
            return 1;
        }

        uint8_t orig_main = 0;
        uint8_t orig_sub = 0;
        custom_mode_to_main_sub(hb.custom_mode, &orig_main, &orig_sub);
        char vbuf[32];
        snprintf(vbuf, sizeof(vbuf), "%u", (unsigned)hb.base_mode);
        kv_set(new_saved, &new_saved_count, "ORIG_FLIGHT_BASE", vbuf);
        snprintf(vbuf, sizeof(vbuf), "%u", (unsigned)orig_main);
        kv_set(new_saved, &new_saved_count, "ORIG_FLIGHT_MAIN", vbuf);
        snprintf(vbuf, sizeof(vbuf), "%u", (unsigned)orig_sub);
        kv_set(new_saved, &new_saved_count, "ORIG_FLIGHT_SUB", vbuf);

        int set_ok = 0;
        for (int attempt = 1; attempt <= retries; ++attempt) {
            send_set_mode_once(fd, &dest, sysid, compid, target_sys, target_comp,
                               hb.base_mode, PX4_CUSTOM_MAIN_MODE_MANUAL, 0);
            uint8_t ack = MAV_RESULT_FAILED;
            if (wait_command_ack_once(fd, MAV_CMD_DO_SET_MODE, timeout_sec, &ack) && ack == MAV_RESULT_ACCEPTED) {
                set_ok = 1;
                break;
            }
        }
        if (!set_ok) {
            fprintf(stderr, "%sFailed%s to switch to MANUAL for launch mode\n", CLR_RED, CLR_RESET);
            close(fd);
            return 1;
        }
        fprintf(stdout, "%sSET%s FLIGHT_MODE MANUAL\n", CLR_GREEN, CLR_RESET);

        /* Set current location/altitude as HOME, with roll/pitch/yaw explicitly unset (NaN). */
        int home_ok = 0;
        mavlink_global_position_int_t gp;
        for (int attempt = 1; attempt <= retries; ++attempt) {
            request_message_once(fd, &dest, sysid, compid, target_sys, target_comp, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
            if (!request_global_position_int_once(fd, target_sys, target_comp, timeout_sec, &gp)) {
                continue;
            }

            double lat_deg = (double)gp.lat / 1e7;
            double lon_deg = (double)gp.lon / 1e7;
            double alt_m = (double)gp.alt / 1000.0;

            send_set_home_once(fd, &dest, sysid, compid, target_sys, target_comp, 0, lat_deg, lon_deg, alt_m);
            uint8_t ack = MAV_RESULT_FAILED;
            if (wait_command_ack_once(fd, MAV_CMD_DO_SET_HOME, timeout_sec, &ack) && ack == MAV_RESULT_ACCEPTED) {
                fprintf(stdout, "%sSET%s HOME lat=%.7f lon=%.7f alt=%.3f (roll/pitch/yaw unset)\n",
                        CLR_GREEN, CLR_RESET, lat_deg, lon_deg, alt_m);
                home_ok = 1;
                break;
            }
        }
        if (!home_ok) {
            fprintf(stderr, "%sWarn%s failed to set HOME via GLOBAL_POSITION_INT + DO_SET_HOME\n",
                    CLR_YELLOW, CLR_RESET);
        }

        if (save_mode_file(mode_path, "LAUNCH", new_saved, new_saved_count) != 0) {
            fprintf(stderr, "%sFailed%s to write %s\n", CLR_RED, CLR_RESET, mode_path);
            close(fd);
            return 1;
        }
        close(fd);
        return 0;
    }

    for (size_t i = 0; i < sizeof(k_mode_params) / sizeof(k_mode_params[0]); ++i) {
        const char *name = k_mode_params[i].name;
        char orig_key[96];
        snprintf(orig_key, sizeof(orig_key), "ORIG_%s", name);

        const char *orig_val = find_kv(saved, saved_count, orig_key);

        mavlink_param_value_t pv;
        int got = 0;
        for (int attempt = 1; attempt <= retries; ++attempt) {
            if (request_param_value_once(fd, &dest, sysid, compid, target_sys, target_comp, name, timeout_sec, &pv)) {
                got = 1;
                break;
            }
        }
        if (!got) {
            if (k_mode_params[i].optional) {
                fprintf(stderr, "%sSkip%s %s (not available)\n", CLR_YELLOW, CLR_RESET, name);
                continue;
            }
            fprintf(stderr, "%sFailed%s to read %s\n", CLR_RED, CLR_RESET, name);
            close(fd);
            return 1;
        }

        double cur = rocket_decode_param_number(pv.param_type, pv.param_value);

        if (strcmp(mode, "test") == 0) {
            // Always refresh original values from FC at test entry.
            char cur_buf[64];
            snprintf(cur_buf, sizeof(cur_buf), "%.9g", cur);
            kv_set(new_saved, &new_saved_count, orig_key, cur_buf);
            double target_val = k_mode_params[i].test_value;
            if (rocket_param_value_matches(pv.param_type, pv.param_value, target_val, 1e-5)) {
                fprintf(stdout, "%sKEEP%s %s = %.9g\n", CLR_CYAN, CLR_RESET, name, target_val);
                continue;
            }
            int set_ok = 0;
            for (int attempt = 1; attempt <= retries; ++attempt) {
                set_param_value_once(fd, &dest, sysid, compid, target_sys, target_comp, name, pv.param_type, target_val);
                mavlink_param_value_t verify;
                if (request_param_value_once(fd, &dest, sysid, compid, target_sys, target_comp, name, timeout_sec, &verify) &&
                    rocket_param_value_matches(verify.param_type, verify.param_value, target_val, 1e-5)) {
                    set_ok = 1;
                    break;
                }
            }
            if (!set_ok) {
                if (k_mode_params[i].optional) {
                    fprintf(stderr, "%sSkip%s %s (set failed)\n", CLR_YELLOW, CLR_RESET, name);
                    continue;
                }
                fprintf(stderr, "%sFailed%s to set %s\n", CLR_RED, CLR_RESET, name);
                close(fd);
                return 1;
            }
            fprintf(stdout, "%sSET%s %s = %.9g\n", CLR_GREEN, CLR_RESET, name, target_val);
        } else {
            double snapshot_val = 0.0;
            int has_snapshot = (snapshot_path && read_param_from_snapshot(snapshot_path, name, &snapshot_val));

            if (!has_snapshot && !orig_val) {
                if (k_mode_params[i].optional) {
                    fprintf(stderr, "%sSkip%s %s (no original saved)\n", CLR_YELLOW, CLR_RESET, name);
                    continue;
                }
                if (saved_mode && strcmp(saved_mode, "TEST") == 0) {
                    fprintf(stderr, "%sMissing%s original for %s (run test first)\n", CLR_RED, CLR_RESET, name);
                    close(fd);
                    return 1;
                }
                fprintf(stderr, "%sSkip%s %s (no original saved)\n", CLR_YELLOW, CLR_RESET, name);
                continue;
            }

            double target_val = has_snapshot ? snapshot_val : strtod(orig_val, NULL);
            if (has_snapshot) {
                fprintf(stdout, "%sUSE%s %s from snapshot = %.9g\n", CLR_CYAN, CLR_RESET, name, target_val);
            }
            if (rocket_param_value_matches(pv.param_type, pv.param_value, target_val, 1e-5)) {
                fprintf(stdout, "%sKEEP%s %s = %.9g\n", CLR_CYAN, CLR_RESET, name, target_val);
                continue;
            }
            int set_ok = 0;
            for (int attempt = 1; attempt <= retries; ++attempt) {
                set_param_value_once(fd, &dest, sysid, compid, target_sys, target_comp, name, pv.param_type, target_val);
                mavlink_param_value_t verify;
                if (request_param_value_once(fd, &dest, sysid, compid, target_sys, target_comp, name, timeout_sec, &verify) &&
                    rocket_param_value_matches(verify.param_type, verify.param_value, target_val, 1e-5)) {
                    set_ok = 1;
                    break;
                }
            }
            if (!set_ok) {
                if (k_mode_params[i].optional) {
                    fprintf(stderr, "%sSkip%s %s (restore failed)\n", CLR_YELLOW, CLR_RESET, name);
                    continue;
                }
                fprintf(stderr, "%sFailed%s to restore %s\n", CLR_RED, CLR_RESET, name);
                close(fd);
                return 1;
            }
            fprintf(stdout, "%sRESTORE%s %s = %.9g\n", CLR_GREEN, CLR_RESET, name, target_val);
        }
    }

    if (strcmp(mode, "test") == 0) {
        if (save_mode_file(mode_path, "TEST", new_saved, new_saved_count) != 0) {
            fprintf(stderr, "%sFailed%s to write %s\n", CLR_RED, CLR_RESET, mode_path);
            close(fd);
            return 1;
        }
    } else {
        kv_t normal_saved[256];
        size_t normal_saved_count = 0;
        kv_copy_orig_entries(saved, saved_count, normal_saved, &normal_saved_count);
        if (save_mode_file(mode_path, "NORMAL", normal_saved, normal_saved_count) != 0) {
            fprintf(stderr, "%sFailed%s to write %s\n", CLR_RED, CLR_RESET, mode_path);
            close(fd);
            return 1;
        }
    }

    close(fd);
    return 0;
}
