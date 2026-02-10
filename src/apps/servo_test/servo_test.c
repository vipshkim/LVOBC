#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/socket.h>
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
#define CLR_CYAN    "\033[36m"
#define CLR_MAGENTA "\033[35m"

#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_PORT 14654
#define DEFAULT_SYSID MAV_DEFAULT_SYSID
#define DEFAULT_COMPID MAV_DEFAULT_COMPID
#define DEFAULT_TARGET_SYS MAV_DEFAULT_TARGET_SYS
#define DEFAULT_TARGET_COMP MAV_DEFAULT_TARGET_COMP

#define DEFAULT_SERVO_COUNT 8
#define MAIN_OUTPUTS 8
#define DEFAULT_MIN_PWM 1000
#define DEFAULT_MAX_PWM 2000

#define WAVE_PERIOD_SEC 1.0
#define DEFAULT_LOOP_HZ 50.0
#define PX4_CUSTOM_MAIN_MODE_MANUAL 1
#define STREAM_FLOAT_COUNT 8
#define STREAM_PACKET_SIZE (1 + 1 + STREAM_FLOAT_COUNT * (int)sizeof(float) + 1)
#define DEFAULT_SERVO_TEST_STREAM_MAP "5,6,7,8"
#define DEFAULT_ACTUATOR_TEST_TIMEOUT_SEC 0.25

static volatile sig_atomic_t g_running = 1;
static char g_last_event[128] = "none";
static int g_show_all_rx = 0;

typedef struct {
    int valid;
    uint8_t base_mode;
    uint8_t main_mode;
    uint8_t sub_mode;
} flight_mode_snapshot_t;

typedef struct {
    char name[64];
    double value;
} param_t;

typedef struct {
    param_t *items;
    size_t count;
} param_list_t;

static int params_get(const param_list_t *list, const char *name, double *out);

static int parse_stream_slot_map(const char *spec, int *slots, int max_slots) {
    if (!spec || !spec[0] || !slots || max_slots <= 0) return 0;
    char buf[128];
    snprintf(buf, sizeof(buf), "%s", spec);
    int n = 0;
    char *save = NULL;
    for (char *tok = strtok_r(buf, ", ", &save); tok; tok = strtok_r(NULL, ", ", &save)) {
        if (n >= max_slots) break;
        char *end = NULL;
        long v = strtol(tok, &end, 10);
        if (end == tok || (end && *end != '\0')) continue;
        if (v < 1 || v > STREAM_FLOAT_COUNT) continue;
        slots[n++] = (int)v;
    }
    return n;
}

static int send_stream_packet(int sock, const struct sockaddr_in *dest, uint8_t arm, const float controls[STREAM_FLOAT_COUNT]) {
    uint8_t pkt[STREAM_PACKET_SIZE];
    size_t off = 0;
    pkt[off++] = '$';
    pkt[off++] = arm ? 1u : 0u;
    for (size_t i = 0; i < STREAM_FLOAT_COUNT; ++i) {
        memcpy(pkt + off, &controls[i], sizeof(float));
        off += sizeof(float);
    }
    pkt[off++] = '\n';
    ssize_t n = sendto(sock, pkt, sizeof(pkt), 0, (const struct sockaddr *)dest, sizeof(*dest));
    return (n == (ssize_t)sizeof(pkt)) ? 1 : 0;
}

static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static int open_serial_port(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    speed_t speed = B115200;
    switch (baud) {
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: speed = B115200; break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static ssize_t send_mavlink_packet(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                                   const uint8_t *buf, size_t len) {
    if (serial_mode) {
        return write(io_fd, buf, len);
    }
    return sendto(io_fd, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void handle_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static void print_banner(const char *label, const char *color) {
    printf("%s%s%s%s\n", CLR_BOLD, color, label, CLR_RESET);
}

static const char *map_output_function_name(int func_id, char *buf, size_t buf_len) {
    if (func_id <= 0) return NULL;

    if (func_id >= 101 && func_id <= 112) {
        snprintf(buf, buf_len, "JetThrottle%d", func_id - 100);
        return buf;
    }
    if (func_id >= 201 && func_id <= 208) {
        snprintf(buf, buf_len, "Servo%d", func_id - 200);
        return buf;
    }
    if (func_id >= 301 && func_id <= 306) {
        snprintf(buf, buf_len, "PeripheralActuatorSet%d", func_id - 300);
        return buf;
    }
    if (func_id >= 407 && func_id <= 412) {
        snprintf(buf, buf_len, "RC_AUX%d", func_id - 406);
        return buf;
    }

    switch (func_id) {
        case 1: return "Constant_Min";
        case 2: return "Constant_Max";
        case 400: return "Landing_Gear";
        case 401: return "Parachute";
        case 402: return "RC_Roll";
        case 403: return "RC_Pitch";
        case 404: return "RC_Throttle";
        case 405: return "RC_Yaw";
        case 406: return "RC_Flaps";
        case 420: return "Gimbal_Roll";
        case 421: return "Gimbal_Pitch";
        case 422: return "Gimbal_Yaw";
        case 430: return "Gripper";
        case 440: return "Landing_Gear_Wheel";
        case 450: return "IC_Engine_Ignition";
        case 451: return "IC_Engine_Throttle";
        case 452: return "IC_Engine_Choke";
        case 453: return "IC_Engine_Starter";
        case 2000: return "Camera_Trigger";
        case 2032: return "Camera_Capture";
        case 2064: return "PPS_Input";
        case 2070: return "RPM_Input";
        default:
            snprintf(buf, buf_len, "Func%d", func_id);
            return buf;
    }
}

static const char *get_servo_role_title(const param_list_t *list, int servo_index, char *buf, size_t buf_len) {
    char name[64];
    double v;

    if (servo_index <= MAIN_OUTPUTS) {
        snprintf(name, sizeof(name), "PWM_MAIN_FUNC%d", servo_index);
        if (params_get(list, name, &v)) return map_output_function_name((int)v, buf, buf_len);
    } else {
        int aux_index = servo_index - MAIN_OUTPUTS;
        snprintf(name, sizeof(name), "PWM_AUX_FUNC%d", aux_index);
        if (params_get(list, name, &v)) return map_output_function_name((int)v, buf, buf_len);
    }
    return NULL;
}

static int get_servo_function_id(const param_list_t *list, int servo_index) {
    char name[64];
    double v;

    snprintf(name, sizeof(name), "UAVCAN_SV_FUNC%d", servo_index);
    if (params_get(list, name, &v)) return (int)v;

    if (servo_index <= MAIN_OUTPUTS) {
        snprintf(name, sizeof(name), "PWM_MAIN_FUNC%d", servo_index);
        if (params_get(list, name, &v)) return (int)v;
    } else {
        int aux_index = servo_index - MAIN_OUTPUTS;
        snprintf(name, sizeof(name), "PWM_AUX_FUNC%d", aux_index);
        if (params_get(list, name, &v)) return (int)v;
    }
    return 0;
}

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

static int parse_double(const char *s, double *out) {
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno != 0 || end == s) return 0;
    *out = v;
    return 1;
}

static const char *select_default_params_path(void) {
    return mav_select_params_path();
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

static void params_set(param_list_t *list, const char *name, double value) {
    for (size_t i = 0; i < list->count; ++i) {
        if (strcmp(list->items[i].name, name) == 0) {
            list->items[i].value = value;
            return;
        }
    }
    param_t *next = realloc(list->items, (list->count + 1) * sizeof(param_t));
    if (!next) return;
    list->items = next;
    snprintf(list->items[list->count].name, sizeof(list->items[list->count].name), "%s", name);
    list->items[list->count].value = value;
    list->count++;
}

static int params_get(const param_list_t *list, const char *name, double *out) {
    for (size_t i = 0; i < list->count; ++i) {
        if (strcmp(list->items[i].name, name) == 0) {
            *out = list->items[i].value;
            return 1;
        }
    }
    return 0;
}

static void params_load_file(param_list_t *list, const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) return;

    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        trim(line);
        if (line[0] == '\0' || line[0] == '#') continue;

        // Format 1: param set-default NAME VALUE
        if (strncmp(line, "param set-default ", 18) == 0) {
            char name[64] = {0};
            char value_str[64] = {0};
            if (sscanf(line + 18, "%63s %63s", name, value_str) == 2) {
                double v;
                if (parse_double(value_str, &v)) params_set(list, name, v);
            }
            continue;
        }

        // Format 2: NAME=VALUE
        char *eq = strchr(line, '=');
        if (eq) {
            *eq = '\0';
            char *name = line;
            char *value_str = eq + 1;
            trim(name);
            trim(value_str);
            double v;
            if (name[0] != '\0' && parse_double(value_str, &v)) {
                params_set(list, name, v);
            }
            continue;
        }

        // Format 3: NAME VALUE
        char name[64] = {0};
        char value_str[64] = {0};
        if (sscanf(line, "%63s %63s", name, value_str) == 2) {
            double v;
            if (parse_double(value_str, &v)) params_set(list, name, v);
        }
    }

    fclose(fp);
}

static int detect_servo_count(const param_list_t *list) {
    double v;
    if (params_get(list, "CA_SV_COUNT", &v)) return (int)v;
    if (params_get(list, "CA_SV_CS_COUNT", &v)) return (int)v;

    int max_idx = 0;
    for (int i = 1; i <= 16; ++i) {
        char name[64];
        double tmp;
        snprintf(name, sizeof(name), "PWM_MAIN_MIN%d", i);
        if (params_get(list, name, &tmp) && tmp > 0 && i > max_idx) max_idx = i;
        snprintf(name, sizeof(name), "PWM_MAIN_MAX%d", i);
        if (params_get(list, name, &tmp) && tmp > 0 && i > max_idx) max_idx = i;
    }
    for (int i = 1; i <= 8; ++i) {
        char name[64];
        double tmp;
        snprintf(name, sizeof(name), "PWM_AUX_MIN%d", i);
        if (params_get(list, name, &tmp) && tmp > 0 && (i + MAIN_OUTPUTS) > max_idx) max_idx = i + MAIN_OUTPUTS;
        snprintf(name, sizeof(name), "PWM_AUX_MAX%d", i);
        if (params_get(list, name, &tmp) && tmp > 0 && (i + MAIN_OUTPUTS) > max_idx) max_idx = i + MAIN_OUTPUTS;
    }
    return max_idx;
}

static void get_min_max(const param_list_t *list, int servo_index, int *out_min, int *out_max) {
    int min_pwm = DEFAULT_MIN_PWM;
    int max_pwm = DEFAULT_MAX_PWM;

    char name[64];
    double v;

    if (servo_index <= MAIN_OUTPUTS) {
        snprintf(name, sizeof(name), "PWM_MAIN_MIN%d", servo_index);
        if (params_get(list, name, &v)) min_pwm = (int)v;
        snprintf(name, sizeof(name), "PWM_MAIN_MAX%d", servo_index);
        if (params_get(list, name, &v)) max_pwm = (int)v;
    } else {
        int aux_index = servo_index - MAIN_OUTPUTS;
        snprintf(name, sizeof(name), "PWM_AUX_MIN%d", aux_index);
        if (params_get(list, name, &v)) min_pwm = (int)v;
        snprintf(name, sizeof(name), "PWM_AUX_MAX%d", aux_index);
        if (params_get(list, name, &v)) max_pwm = (int)v;
    }

    if (min_pwm > max_pwm) {
        int tmp = min_pwm;
        min_pwm = max_pwm;
        max_pwm = tmp;
    }

    *out_min = min_pwm;
    *out_max = max_pwm;
}

static void send_arm_disarm(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                            int sysid, int compid, int target_sys, int target_comp,
                            int arm, int force) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm ? 1.0f : 0.0f,
        force ? 21196.0f : 0.0f,
        0, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static void send_set_message_interval(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                                      int sysid, int compid, int target_sys, int target_comp,
                                      uint32_t msg_id, int32_t interval_us) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        (float)msg_id,
        (float)interval_us,
        0, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static void send_actuator_test_cmd(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                                   int sysid, int compid, int target_sys, int target_comp,
                                   int func_id, float value, float timeout_sec) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    if (func_id <= 0) return;
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_ACTUATOR_TEST,
        0,
        (float)func_id,
        value,
        timeout_sec,
        0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static void send_heartbeat(int io_fd, bool serial_mode, const struct sockaddr_in *dest, int sysid, int compid) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        sysid, compid, &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static uint64_t now_usec(void) {
    return (uint64_t)(now_seconds() * 1000000.0);
}

static void custom_mode_to_main_sub(uint32_t custom_mode, uint8_t *main_mode, uint8_t *sub_mode) {
    if (main_mode) *main_mode = (uint8_t)((custom_mode >> 16) & 0xFFu);
    if (sub_mode) *sub_mode = (uint8_t)((custom_mode >> 24) & 0xFFu);
}

static void send_set_mode(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                          int sysid, int compid, int target_sys, int target_comp,
                          uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t use_base = (uint8_t)(base_mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);

    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_DO_SET_MODE,
        0,
        (float)use_base,
        (float)main_mode,
        (float)sub_mode,
        0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static int wait_for_target_heartbeat(int io_fd, bool serial_mode,
                                     int target_sys, int target_comp,
                                     double timeout_sec, flight_mode_snapshot_t *out_mode) {
    unsigned char buf[2048];
    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double deadline = now_seconds() + timeout_sec;
    while (now_seconds() < deadline) {
        double remaining = deadline - now_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(io_fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(io_fd + 1, &rfds, NULL, NULL, &tv);
        if (ready <= 0) continue;
        if (!FD_ISSET(io_fd, &rfds)) continue;

        ssize_t n = serial_mode ? read(io_fd, buf, sizeof(buf)) : recvfrom(io_fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) continue;
            if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) continue;
            if (message.sysid != (uint8_t)target_sys) continue;
            if (target_comp > 0 && message.compid != (uint8_t)target_comp) continue;

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&message, &hb);
            if (out_mode) {
                out_mode->valid = 1;
                out_mode->base_mode = hb.base_mode;
                custom_mode_to_main_sub(hb.custom_mode, &out_mode->main_mode, &out_mode->sub_mode);
            }
            return 1;
        }
    }
    return 0;
}

static void send_actuator(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                          int sysid, int compid,
                          int target_sys, int target_comp, int servo_index, float value) {
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    int group = (servo_index - 1) / 8;
    int within = (servo_index - 1) % 8;
    if (group < 0) group = 0;
    if (group > 3) group = 3;

    float controls[8] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
    controls[within] = value;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_set_actuator_control_target_pack(
        sysid, compid, &msg,
        now_usec(),
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        (uint8_t)group,
        controls);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, dest, buf, len);
}

static const char *ack_result_str(uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMP_REJECTED";
        case MAV_RESULT_DENIED: return "DENIED";
        case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
        case MAV_RESULT_FAILED: return "FAILED";
        case MAV_RESULT_IN_PROGRESS: return "IN_PROGRESS";
        case MAV_RESULT_CANCELLED: return "CANCELLED";
        default: return "UNKNOWN";
    }
}

static int wait_for_command_ack(int io_fd, bool serial_mode, uint16_t cmd,
                                double timeout_sec, char *out, size_t out_len) {
    unsigned char buf[2048];
    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double deadline = now_seconds() + timeout_sec;
    while (now_seconds() < deadline) {
        double remaining = deadline - now_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(io_fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(io_fd + 1, &rfds, NULL, NULL, &tv);
        if (ready <= 0) continue;
        if (!FD_ISSET(io_fd, &rfds)) continue;

        ssize_t n = serial_mode ? read(io_fd, buf, sizeof(buf)) : recvfrom(io_fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) continue;
            if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&message, &ack);
                if (ack.command == cmd) {
                    snprintf(out, out_len, "ACK cmd=%u %s (%u) rp2=%d prog=%u tsys=%u tcomp=%u",
                             (unsigned)ack.command,
                             ack_result_str(ack.result),
                             (unsigned)ack.result,
                             ack.result_param2,
                             (unsigned)ack.progress,
                             message.sysid,
                             message.compid);
                    return 1;
                }
            } else if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                mavlink_statustext_t st;
                mavlink_msg_statustext_decode(&message, &st);
                char text[51];
                memcpy(text, st.text, 50);
                text[50] = '\0';
                snprintf(g_last_event, sizeof(g_last_event),
                         "STATUSTEXT sev=%u tsys=%u tcomp=%u %s",
                         (unsigned)st.severity,
                         message.sysid,
                         message.compid,
                         text);
            }
        }
    }
    return 0;
}

static void drain_command_acks(int io_fd, bool serial_mode) {
    unsigned char buf[2048];
    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    for (;;) {
        ssize_t n = serial_mode ? read(io_fd, buf, sizeof(buf)) : recvfrom(io_fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
            return;
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) continue;
            if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&message, &ack);
                snprintf(g_last_event, sizeof(g_last_event),
                         "ACK cmd=%u %s (%u) rp2=%d prog=%u tsys=%u tcomp=%u",
                         (unsigned)ack.command,
                         ack_result_str(ack.result),
                         (unsigned)ack.result,
                         ack.result_param2,
                         (unsigned)ack.progress,
                         message.sysid,
                         message.compid);
            } else if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                mavlink_statustext_t st;
                mavlink_msg_statustext_decode(&message, &st);
                char text[51];
                memcpy(text, st.text, 50);
                text[50] = '\0';
                snprintf(g_last_event, sizeof(g_last_event),
                         "STATUSTEXT sev=%u tsys=%u tcomp=%u %s",
                         (unsigned)st.severity,
                         message.sysid,
                         message.compid,
                         text);
            } else if (g_show_all_rx) {
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
                snprintf(g_last_event, sizeof(g_last_event),
                         "RX %s(%u) tsys=%u tcomp=%u",
                         name, message.msgid, message.sysid, message.compid);
            }
        }
    }
}

static void usage(const char *prog, const char *ip_default, int port_default,
                  int sysid_default, int compid_default, int target_sys_default, int target_comp_default) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  -c <config>     : config.params (선택, 기본 /tmp -> ~/mavlink_projects/scan)\n"
        "  -a <airframe>   : airframe 파일(추가 파라미터용, 선택)\n"
        "  -n <count>      : 서보 개수 강제 지정\n"
        "  -u <ip>         : stream_commander 입력 IP (기본 %s)\n"
        "  -p <port>       : stream_commander 입력 포트 (기본 %d)\n"
        "  -s <sysid>      : 송신 sysid (기본 %d)\n"
        "  -c2 <compid>    : 송신 compid (기본 %d)\n"
        "  -t <tsys>       : 대상 sysid (기본 %d)\n"
        "  -k <tcomp>      : 대상 compid (기본 %d)\n"
        "  --act-test      : MAV_CMD_ACTUATOR_TEST 직접 전송(테스트)\n"
        "  --show-rx       : 이벤트 문자열 표시\n",
        prog, ip_default, port_default, sysid_default, compid_default,
        target_sys_default, target_comp_default);
}

int main(int argc, char **argv) {
    const char *config_path = NULL;
    const char *airframe_path = NULL;
    const char *ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_LISTEN_IP, DEFAULT_IP);
    int port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT, DEFAULT_PORT);
    const char *arm_ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, MAV_DEFAULT_IP);
    int arm_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, MAV_DEFAULT_PORT);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_SERVO_COMPID,
                                 mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, DEFAULT_COMPID));
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, DEFAULT_TARGET_COMP);
    const char *stream_out_mode = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_OUTPUT_MODE, "ACTUATOR");
    int act_test_mode = (stream_out_mode &&
                        (strcasecmp(stream_out_mode, "ACTUATOR_TEST") == 0 ||
                         strcasecmp(stream_out_mode, "TEST") == 0));
    int direct_act_test = mav_cfg_get_int(MAV_CFG_KEY_SERVO_TEST_DIRECT_ACT_TEST, 0);
    float act_test_timeout = (float)mav_cfg_get_double(MAV_CFG_KEY_SERVO_TEST_ACT_TIMEOUT_SEC,
                                                       DEFAULT_ACTUATOR_TEST_TIMEOUT_SEC);
    int force_count = 0;
    double loop_hz = mav_cfg_get_double(MAV_CFG_KEY_SERVO_LOOP_HZ, DEFAULT_LOOP_HZ);
    int force_act_test = 0;
    g_show_all_rx = 0;

    if (!(loop_hz > 0.0) || !isfinite(loop_hz)) {
        loop_hz = DEFAULT_LOOP_HZ;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            airframe_path = argv[++i];
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            force_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c2") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--act-test") == 0) {
            force_act_test = 1;
        } else if (strcmp(argv[i], "--show-rx") == 0) {
            g_show_all_rx = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0], ip, port, sysid, compid, target_sys, target_comp);
            return 0;
        } else {
            usage(argv[0], ip, port, sysid, compid, target_sys, target_comp);
            return 1;
        }
    }

    if (force_act_test) {
        act_test_mode = 1;
        direct_act_test = 1;
    }
    if (direct_act_test && !act_test_mode) {
        act_test_mode = 1;
    }

    if (!config_path) {
        config_path = select_default_params_path();
        if (config_path) {
            printf("%sPARAM%s Using default params file: %s\n", CLR_CYAN, CLR_RESET, config_path);
        }
    }

    param_list_t params;
    params_init(&params);
    if (config_path) params_load_file(&params, config_path);
    if (airframe_path) params_load_file(&params, airframe_path);

    int count = 0;
    if (force_count > 0) {
        count = force_count;
    } else {
        count = detect_servo_count(&params);
        if (count <= 0) {
            count = DEFAULT_SERVO_COUNT;
            fprintf(stderr, "No servo count detected. Using default count %d.\n", count);
        }
    }

    int io_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (io_fd < 0) {
        perror("socket");
        params_free(&params);
        return 1;
    }
    int flags = fcntl(io_fd, F_GETFL, 0);
    if (flags >= 0) fcntl(io_fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP: %s\n", ip);
        close(io_fd);
        params_free(&params);
        return 1;
    }

    print_banner("MAV SERVO TEST", CLR_BLUE);
    printf("%sTARGET%s stream_commander %s:%d (tsys=%d tcomp=%d)\n", CLR_YELLOW, CLR_RESET, ip, port, target_sys, target_comp);
    printf("%sARM PATH%s direct MAVLink %s:%d (force arm/disarm)\n", CLR_YELLOW, CLR_RESET, arm_ip, arm_port);
    if (act_test_mode) {
        printf("%sMODE%s stream_commander output is ACTUATOR_TEST -> run DISARMED test path\n",
               CLR_YELLOW, CLR_RESET);
    }
    if (direct_act_test) {
        printf("%sACTUATOR_TEST%s direct MAV_CMD_ACTUATOR_TEST enabled\n", CLR_YELLOW, CLR_RESET);
    }
    printf("%sCOUNT%s %d\n", CLR_YELLOW, CLR_RESET, count);
    printf("%sINPUT%s Enter = next servo, q + Enter = quit\n", CLR_YELLOW, CLR_RESET);
    printf("%sTX PATH%s via stream_commander packet interface ($ arm + 8 floats + newline)\n",
           CLR_GREEN, CLR_RESET);

    int stream_slots[STREAM_FLOAT_COUNT] = {0};
    const char *slot_map_spec = mav_cfg_get_str(
        MAV_CFG_KEY_SERVO_TEST_STREAM_MAP,
        DEFAULT_SERVO_TEST_STREAM_MAP);
    int stream_servo_count = parse_stream_slot_map(slot_map_spec, stream_slots, STREAM_FLOAT_COUNT);
    if (stream_servo_count <= 0) {
        stream_slots[0] = 5;
        stream_slots[1] = 6;
        stream_slots[2] = 7;
        stream_slots[3] = 8;
        stream_servo_count = 4;
    }
    if (stream_servo_count > count) {
        stream_servo_count = count;
    }
    printf("%sMAP%s stream slots:", CLR_YELLOW, CLR_RESET);
    for (int i = 0; i < stream_servo_count; ++i) {
        printf(" %d", stream_slots[i]);
    }
    printf("\n");

    int arm_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (arm_fd < 0) {
        perror("socket");
        close(io_fd);
        params_free(&params);
        return 1;
    }
    int arm_flags = fcntl(arm_fd, F_GETFL, 0);
    if (arm_flags >= 0) fcntl(arm_fd, F_SETFL, arm_flags | O_NONBLOCK);

    struct sockaddr_in arm_dest;
    memset(&arm_dest, 0, sizeof(arm_dest));
    arm_dest.sin_family = AF_INET;
    arm_dest.sin_port = htons(arm_port);
    if (inet_pton(AF_INET, arm_ip, &arm_dest.sin_addr) != 1) {
        fprintf(stderr, "Invalid ARM IP: %s\n", arm_ip);
        close(arm_fd);
        close(io_fd);
        params_free(&params);
        return 1;
    }

    if (!act_test_mode) {
        // Separate channel force-arm command in addition to stream_commander packet arm.
        for (int i = 0; i < 2; ++i) {
            send_arm_disarm(arm_fd, false, &arm_dest, sysid, compid, target_sys, target_comp, 1, 1);
            usleep(20000);
        }
        // Arm via stream_commander input packet.
        float controls[STREAM_FLOAT_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            (void)send_stream_packet(io_fd, &dest, 1, controls);
            usleep(20000);
        }
    } else {
        // ACTUATOR_TEST is expected to operate in disarmed state.
        float controls[STREAM_FLOAT_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 2; ++i) {
            send_arm_disarm(arm_fd, false, &arm_dest, sysid, compid, target_sys, target_comp, 0, 1);
            (void)send_stream_packet(io_fd, &dest, 0, controls);
            usleep(20000);
        }
    }

    int servo = 1;
    char input_buf[64];
    while (g_running && servo <= stream_servo_count) {
        char role_buf[64];
        const char *role = get_servo_role_title(&params, servo, role_buf, sizeof(role_buf));

        print_banner("SERVO STEP", CLR_BLUE);
        if (role) {
            printf("%sSERVO%s %d | %sROLE%s %s | %sVALUE%s -1..1\n",
                   CLR_GREEN, CLR_RESET, servo,
                   CLR_YELLOW, CLR_RESET, role,
                   CLR_GREEN, CLR_RESET);
        } else {
            printf("%sSERVO%s %d | %sVALUE%s -1..1\n",
                   CLR_GREEN, CLR_RESET, servo,
                   CLR_GREEN, CLR_RESET);
        }

        double start = now_seconds();
        for (;;) {
            double t = now_seconds() - start;
            double phase = (2.0 * M_PI) * (t / WAVE_PERIOD_SEC);
            float u = (float)sin(phase);
            if (act_test_mode && direct_act_test) {
                int func_id = get_servo_function_id(&params, servo);
                if (func_id > 0) {
                    send_actuator_test_cmd(arm_fd, false, &arm_dest, sysid, compid,
                                           target_sys, target_comp, func_id, u,
                                           act_test_timeout);
                    drain_command_acks(arm_fd, false);
                } else {
                    float controls[STREAM_FLOAT_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
                    int stream_slot = stream_slots[servo - 1] - 1; // 1..8 -> index 0..7
                    if (stream_slot >= 0 && stream_slot < STREAM_FLOAT_COUNT) {
                        controls[stream_slot] = u;
                    }
                    (void)send_stream_packet(io_fd, &dest, 1, controls);
                    drain_command_acks(arm_fd, false);
                }
            } else {
                float controls[STREAM_FLOAT_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
                int stream_slot = stream_slots[servo - 1] - 1; // 1..8 -> index 0..7
                if (stream_slot >= 0 && stream_slot < STREAM_FLOAT_COUNT) {
                    controls[stream_slot] = u;
                }
                (void)send_stream_packet(io_fd, &dest, 1, controls);
                drain_command_acks(arm_fd, false);
            }
            printf("\r%sVALUE%s %+0.3f | %sEVT%s %s\033[K",
                   CLR_CYAN, CLR_RESET, u,
                   CLR_MAGENTA, CLR_RESET, g_last_event);
            fflush(stdout);

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = (suseconds_t)(1e6 / loop_hz);

            int ready = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
            if (ready > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
                if (!fgets(input_buf, sizeof(input_buf), stdin)) {
                    // stdin may be non-interactive/EOF; keep running unless explicitly quit.
                    clearerr(stdin);
                    continue;
                }
                trim(input_buf);
                if (input_buf[0] == 'q' || input_buf[0] == 'Q') {
                    g_running = 0;
                    break;
                }
                if (input_buf[0] == '\0' && stream_servo_count <= 1) {
                    // Single-slot test: ignore empty input to keep running.
                    continue;
                }
                servo++;
                printf("\n");
                break;
            }
            if (!g_running) break;
        }
        if (!g_running) {
            printf("\n");
        }
    }

    print_banner("RESET OUTPUTS", CLR_BLUE);
    {
        float controls[STREAM_FLOAT_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            (void)send_stream_packet(io_fd, &dest, 0, controls);
            usleep(20000);
        }
    }
    if (!act_test_mode) {
        for (int i = 0; i < 2; ++i) {
            send_arm_disarm(arm_fd, false, &arm_dest, sysid, compid, target_sys, target_comp, 0, 1);
            usleep(20000);
        }
    }

    close(arm_fd);
    close(io_fd);
    params_free(&params);
    return 0;
}
