#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
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
#define CLR_CYAN    "\033[36m"

#define DEFAULT_IP MAV_DEFAULT_IP
#define DEFAULT_PORT MAV_DEFAULT_PORT
#define DEFAULT_SYSID MAV_DEFAULT_SYSID
#define DEFAULT_COMPID MAV_DEFAULT_COMPID
#define DEFAULT_TARGET_SYS MAV_DEFAULT_TARGET_SYS
#define DEFAULT_TARGET_COMP MAV_DEFAULT_TARGET_COMP

#define DEFAULT_MOTOR_COUNT 4
#define MAIN_OUTPUTS 8

#define ACTUATOR_RAW_MAX 8191
#define STAGE_PRE_IDLE_PERCENT 20.0
#define STAGE_ADJUST_MIN_PERCENT 5.0
#define STAGE_ADJUST_MAX_PERCENT 30.0
#define DEFAULT_LOOP_HZ 50.0

static volatile sig_atomic_t g_running = 1;

typedef struct {
    char name[64];
    double value;
} param_t;

typedef struct {
    param_t *items;
    size_t count;
} param_list_t;

typedef struct {
    int motor_index;
    char role[96];
    int raw_max;
    int raw_min;
    int requested_idle_raw;
    int idle_raw;
} motor_cal_result_t;

static int params_get(const param_list_t *list, const char *name, double *out);

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

static int is_motor_function_id(int func_id) {
    if (func_id >= 101 && func_id <= 112) return 1;
    if (func_id == 451) return 1; // IC engine throttle
    return 0;
}

static int detect_motor_count_from_functions(const param_list_t *list) {
    int detected = 0;
    for (int i = 1; i <= 16; ++i) {
        char name[64];
        double v = 0.0;
        snprintf(name, sizeof(name), "PWM_MAIN_FUNC%d", i);
        if (!params_get(list, name, &v)) continue;
        int func_id = (int)v;
        if (!is_motor_function_id(func_id)) continue;
        if (func_id >= 101 && func_id <= 112 && func_id - 100 > detected) detected = func_id - 100;
        if (func_id == 451 && detected < 1) detected = 1;
    }
    for (int i = 1; i <= 8; ++i) {
        char name[64];
        double v = 0.0;
        snprintf(name, sizeof(name), "PWM_AUX_FUNC%d", i);
        if (!params_get(list, name, &v)) continue;
        int func_id = (int)v;
        if (!is_motor_function_id(func_id)) continue;
        if (func_id >= 101 && func_id <= 112 && func_id - 100 > detected) detected = func_id - 100;
        if (func_id == 451 && detected < 1) detected = 1;
    }
    for (int i = 1; i <= 8; ++i) {
        char name[64];
        double v = 0.0;
        snprintf(name, sizeof(name), "UAVCAN_EC_FUNC%d", i);
        if (!params_get(list, name, &v)) continue;
        int func_id = (int)v;
        if (!is_motor_function_id(func_id)) continue;
        if (func_id >= 101 && func_id <= 112 && func_id - 100 > detected) detected = func_id - 100;
        if (func_id == 451 && detected < 1) detected = 1;
    }
    return detected;
}

static int detect_motor_count_from_rotor_geometry(const param_list_t *list) {
    int detected = 0;
    for (int i = 0; i < 16; ++i) {
        char px_name[64];
        char py_name[64];
        double px = 0.0;
        double py = 0.0;
        snprintf(px_name, sizeof(px_name), "CA_ROTOR%d_PX", i);
        snprintf(py_name, sizeof(py_name), "CA_ROTOR%d_PY", i);
        int has_px = params_get(list, px_name, &px);
        int has_py = params_get(list, py_name, &py);
        if (!has_px && !has_py) continue;
        if (fabs(px) > 1e-6 || fabs(py) > 1e-6) {
            if (i + 1 > detected) detected = i + 1;
        }
    }
    return detected;
}

static int detect_motor_count(const param_list_t *list, const char **source) {
    double v = 0.0;
    if (params_get(list, "CA_ROTOR_COUNT", &v) && (int)v > 0) {
        if (source) *source = "CA_ROTOR_COUNT";
        return (int)v;
    }
    if (params_get(list, "CA_MC_R_COUNT", &v) && (int)v > 0) {
        if (source) *source = "CA_MC_R_COUNT";
        return (int)v;
    }

    int by_func = detect_motor_count_from_functions(list);
    if (by_func > 0) {
        if (source) *source = "output functions";
        return by_func;
    }

    int by_geom = detect_motor_count_from_rotor_geometry(list);
    if (by_geom > 0) {
        if (source) *source = "CA_ROTORx_PX/PY";
        return by_geom;
    }

    if (source) *source = "default";
    return 0;
}

static const char *find_motor_role(const param_list_t *list, int motor_index, char *buf, size_t buf_len) {
    int expected = 100 + motor_index;
    for (int i = 1; i <= 16; ++i) {
        char key[64];
        double v = 0.0;
        snprintf(key, sizeof(key), "PWM_MAIN_FUNC%d", i);
        if (!params_get(list, key, &v)) continue;
        int func_id = (int)v;
        if (func_id == expected || (motor_index == 1 && func_id == 451)) {
            const char *name = map_output_function_name(func_id, buf, buf_len);
            char role_name[64];
            snprintf(role_name, sizeof(role_name), "%s", name ? name : "Motor");
            snprintf(buf, buf_len, "%.40s (%.24s)", role_name, key);
            return buf;
        }
    }
    for (int i = 1; i <= 8; ++i) {
        char key[64];
        double v = 0.0;
        snprintf(key, sizeof(key), "PWM_AUX_FUNC%d", i);
        if (!params_get(list, key, &v)) continue;
        int func_id = (int)v;
        if (func_id == expected || (motor_index == 1 && func_id == 451)) {
            const char *name = map_output_function_name(func_id, buf, buf_len);
            char role_name[64];
            snprintf(role_name, sizeof(role_name), "%s", name ? name : "Motor");
            snprintf(buf, buf_len, "%.40s (%.24s)", role_name, key);
            return buf;
        }
    }
    for (int i = 1; i <= 8; ++i) {
        char key[64];
        double v = 0.0;
        snprintf(key, sizeof(key), "UAVCAN_EC_FUNC%d", i);
        if (!params_get(list, key, &v)) continue;
        int func_id = (int)v;
        if (func_id == expected || (motor_index == 1 && func_id == 451)) {
            const char *name = map_output_function_name(func_id, buf, buf_len);
            char role_name[64];
            snprintf(role_name, sizeof(role_name), "%s", name ? name : "Motor");
            snprintf(buf, buf_len, "%.40s (%.24s)", role_name, key);
            return buf;
        }
    }
    return NULL;
}

static void send_arm_disarm(int sock, const struct sockaddr_in *dest, int sysid, int compid, int target_sys, int target_comp, int arm) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm ? 1.0f : 0.0f,
        21196.0f,
        0, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void send_actuator(int sock, const struct sockaddr_in *dest, int sysid, int compid, int target_sys, int target_comp, int servo_index, float value) {
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    int set_index = (servo_index - 1) / 6;
    int within = (servo_index - 1) % 6;

    float params[6] = {NAN, NAN, NAN, NAN, NAN, NAN};
    params[within] = value;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        sysid, compid, &msg,
        target_sys, target_comp,
        MAV_CMD_DO_SET_ACTUATOR,
        0,
        params[0],
        params[1],
        params[2],
        params[3],
        params[4],
        params[5],
        (float)set_index);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static int raw_from_percent(double percent) {
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    return (int)lround((percent / 100.0) * (double)ACTUATOR_RAW_MAX);
}

static double percent_from_raw(int raw) {
    if (raw < 0) raw = 0;
    if (raw > ACTUATOR_RAW_MAX) raw = ACTUATOR_RAW_MAX;
    return ((double)raw * 100.0) / (double)ACTUATOR_RAW_MAX;
}

static double throttle_from_raw(int raw, int idle_raw) {
    if (raw <= idle_raw) return 0.0;
    if (idle_raw >= ACTUATOR_RAW_MAX) return 100.0;
    return ((double)(raw - idle_raw) * 100.0) / (double)(ACTUATOR_RAW_MAX - idle_raw);
}

static int set_stdin_raw_mode(struct termios *saved, int *changed) {
    *changed = 0;
    if (!isatty(STDIN_FILENO)) return 1;
    if (tcgetattr(STDIN_FILENO, saved) != 0) return 0;
    struct termios raw = *saved;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_iflag &= (tcflag_t) ~(ICRNL | IXON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return 0;
    *changed = 1;
    return 1;
}

static void restore_stdin_mode(const struct termios *saved, int changed) {
    if (!changed) return;
    tcsetattr(STDIN_FILENO, TCSANOW, saved);
}

typedef enum {
    KEY_NONE = 0,
    KEY_ENTER,
    KEY_QUIT,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT
} key_event_t;

static key_event_t read_key_event(double timeout_sec) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    struct timeval tv;
    tv.tv_sec = (time_t)timeout_sec;
    tv.tv_usec = (suseconds_t)((timeout_sec - (double)tv.tv_sec) * 1e6);

    int ready = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &rfds)) return KEY_NONE;

    unsigned char seq[8] = {0};
    ssize_t n = read(STDIN_FILENO, seq, sizeof(seq));
    if (n <= 0) return KEY_NONE;

    if (seq[0] == '\n' || seq[0] == '\r') return KEY_ENTER;
    if (seq[0] == 'q' || seq[0] == 'Q' || seq[0] == 3) return KEY_QUIT;

    if (seq[0] == 0x1b && n >= 3 && seq[1] == '[') {
        if (seq[2] == 'A') return KEY_UP;
        if (seq[2] == 'B') return KEY_DOWN;
        if (seq[2] == 'C') return KEY_RIGHT;
        if (seq[2] == 'D') return KEY_LEFT;
    }
    return KEY_NONE;
}

static int run_hold_stage(int sock, const struct sockaddr_in *dest, int sysid, int compid,
                          int target_sys, int target_comp, int motor_index, int raw_value,
                          const char *title, const char *prompt, double loop_hz) {
    print_banner(title, CLR_BLUE);
    printf("%s%s%s\n", CLR_YELLOW, prompt, CLR_RESET);
    while (g_running) {
        send_actuator(sock, dest, sysid, compid, target_sys, target_comp, motor_index,
                      (float)raw_value / (float)ACTUATOR_RAW_MAX);
        printf("\r%sOUTPUT%s raw=%4d/%d (%5.2f%%)  [Enter=next, q=quit]   ",
               CLR_CYAN, CLR_RESET, raw_value, ACTUATOR_RAW_MAX, percent_from_raw(raw_value));
        fflush(stdout);

        key_event_t ev = read_key_event(1.0 / loop_hz);
        if (ev == KEY_ENTER) {
            printf("\n");
            return 1;
        }
        if (ev == KEY_QUIT) {
            g_running = 0;
            break;
        }
    }
    printf("\n");
    return 0;
}

static int run_adjust_stage(int sock, const struct sockaddr_in *dest, int sysid, int compid,
                            int target_sys, int target_comp, int motor_index, int start_raw,
                            int min_raw, int max_raw, int *out_raw, const char *title, double loop_hz) {
    int raw = start_raw;
    if (raw < min_raw) raw = min_raw;
    if (raw > max_raw) raw = max_raw;

    print_banner(title ? title : "STAGE: ADJUST IDLE POINT", CLR_BLUE);
    printf("%sUse Arrow keys to tune (1/%d step), Enter=save, q=quit%s\n",
           CLR_YELLOW, ACTUATOR_RAW_MAX, CLR_RESET);

    while (g_running) {
        send_actuator(sock, dest, sysid, compid, target_sys, target_comp, motor_index,
                      (float)raw / (float)ACTUATOR_RAW_MAX);
        printf("\r%sIDLE%s raw=%4d/%d (%5.2f%%) range[%4d..%4d] (%0.1f..%0.1f%%)   ",
               CLR_GREEN, CLR_RESET,
               raw, ACTUATOR_RAW_MAX, percent_from_raw(raw),
               min_raw, max_raw, percent_from_raw(min_raw), percent_from_raw(max_raw));
        fflush(stdout);

        key_event_t ev = read_key_event(1.0 / loop_hz);
        if (ev == KEY_ENTER) {
            printf("\n");
            *out_raw = raw;
            return 1;
        }
        if (ev == KEY_QUIT) {
            g_running = 0;
            break;
        }
        if (ev == KEY_UP || ev == KEY_RIGHT) raw++;
        if (ev == KEY_DOWN || ev == KEY_LEFT) raw--;
        if (raw < min_raw) raw = min_raw;
        if (raw > max_raw) raw = max_raw;
    }
    printf("\n");
    return 0;
}

static const char *default_motor_config_path(void) {
    static char path[512];
    const char *home = getenv("HOME");
    if (!home || !home[0]) home = "/home/rocket";
    snprintf(path, sizeof(path), "%s/mavlink_projects/motor_init/motor.config", home);
    return path;
}

static int write_motor_config(const char *path, int count,
                              const motor_cal_result_t *results, size_t result_count) {
    FILE *fp = fopen(path, "w");
    if (!fp) return 0;
    fprintf(fp, "# motor_init calibration result\n");
    fprintf(fp, "MOTOR_COUNT=%d\n", count);
    fprintf(fp, "CALIBRATED_MOTORS=%zu\n", result_count);
    for (size_t i = 0; i < result_count; ++i) {
        const motor_cal_result_t *r = &results[i];
        fprintf(fp, "\n");
        fprintf(fp, "MOTOR%d_ROLE=%s\n", r->motor_index, r->role[0] ? r->role : "Motor");
        fprintf(fp, "MOTOR%d_RAW_MAX=%d\n", r->motor_index, r->raw_max);
        fprintf(fp, "MOTOR%d_RAW_MIN=%d\n", r->motor_index, r->raw_min);
        fprintf(fp, "MOTOR%d_REQUESTED_IDLE_RAW=%d\n", r->motor_index, r->requested_idle_raw);
        fprintf(fp, "MOTOR%d_IDLE_RAW=%d\n", r->motor_index, r->idle_raw);
        fprintf(fp, "MOTOR%d_IDLE_PERCENT=%.6f\n", r->motor_index, percent_from_raw(r->idle_raw));
        fprintf(fp, "MOTOR%d_THROTTLE_MAP=if raw<=IDLE_RAW then 0 else (raw-IDLE_RAW)*100/(%d-IDLE_RAW)\n",
                r->motor_index, ACTUATOR_RAW_MAX);
    }
    fclose(fp);
    return 1;
}

static void usage(const char *prog, const char *ip_default, int port_default,
                  int sysid_default, int compid_default, int target_sys_default, int target_comp_default) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  -c <config>     : config.params (선택, 기본 /tmp -> ~/mavlink_projects/scan)\n"
        "  -a <airframe>   : airframe 파일(추가 파라미터용, 선택)\n"
        "  -n <count>      : 모터 개수 강제 지정\n"
        "  -m <index>      : 특정 모터만 캘리브레이션 (기본: 전체)\n"
        "  -o <path>       : motor.config 저장 경로\n"
        "  -u <ip>         : 대상 IP (기본 %s)\n"
        "  -p <port>       : 대상 포트 (기본 %d)\n"
        "  -s <sysid>      : 송신 sysid (기본 %d)\n"
        "  -c2 <compid>    : 송신 compid (기본 %d)\n"
        "  -t <tsys>       : 대상 sysid (기본 %d)\n"
        "  -k <tcomp>      : 대상 compid (기본 %d)\n",
        prog, ip_default, port_default, sysid_default, compid_default, target_sys_default, target_comp_default);
}

int main(int argc, char **argv) {
    const char *config_path = NULL;
    const char *airframe_path = NULL;
    const char *output_path = default_motor_config_path();
    const char *ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, DEFAULT_IP);
    int port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, DEFAULT_PORT);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_MOTOR_COMPID,
                                 mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, DEFAULT_COMPID));
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, DEFAULT_TARGET_COMP);
    double loop_hz = mav_cfg_get_double(MAV_CFG_KEY_MOTOR_LOOP_HZ, DEFAULT_LOOP_HZ);
    int force_count = 0;
    int motor_index = 1;
    int single_motor_mode = 0;
    int exit_code = 1;
    struct termios stdin_saved;
    int stdin_raw_changed = 0;
    motor_cal_result_t *results = NULL;
    size_t result_count = 0;
    int motor_start = 1;
    int motor_end = 1;
    int expected_motor_runs = 0;

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
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            motor_index = atoi(argv[++i]);
            single_motor_mode = 1;
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_path = argv[++i];
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
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0], ip, port, sysid, compid, target_sys, target_comp);
            return 0;
        } else {
            usage(argv[0], ip, port, sysid, compid, target_sys, target_comp);
            return 1;
        }
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
    const char *count_source = "default";
    if (force_count > 0) {
        count = force_count;
        count_source = "CLI -n";
    } else {
        count = detect_motor_count(&params, &count_source);
        if (count <= 0) {
            count = DEFAULT_MOTOR_COUNT;
            count_source = "fallback default";
            fprintf(stderr, "%sCOUNT%s No motor count detected, using default %d\n",
                    CLR_YELLOW, CLR_RESET, count);
        }
    }
    if (single_motor_mode) {
        if (motor_index < 1) motor_index = 1;
        if (motor_index > count) motor_index = count;
        motor_start = motor_index;
        motor_end = motor_index;
    } else {
        motor_start = 1;
        motor_end = count;
    }
    expected_motor_runs = motor_end - motor_start + 1;
    if (expected_motor_runs <= 0) {
        fprintf(stderr, "%sInvalid motor range%s\n", CLR_RED, CLR_RESET);
        params_free(&params);
        return 1;
    }
    results = calloc((size_t)expected_motor_runs, sizeof(*results));
    if (!results) {
        fprintf(stderr, "%sFailed to allocate result buffer%s\n", CLR_RED, CLR_RESET);
        params_free(&params);
        return 1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        params_free(&params);
        return 1;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP: %s\n", ip);
        close(sock);
        params_free(&params);
        return 1;
    }

    print_banner("MAV MOTOR INIT", CLR_BLUE);
    printf("%sTARGET%s %s:%d (tsys=%d tcomp=%d)\n", CLR_YELLOW, CLR_RESET, ip, port, target_sys, target_comp);
    printf("%sCOUNT%s %d (%s)\n", CLR_YELLOW, CLR_RESET, count, count_source);
    if (single_motor_mode) {
        printf("%sMODE%s single motor #%d\n", CLR_YELLOW, CLR_RESET, motor_index);
    } else {
        printf("%sMODE%s full sequence motors %d..%d (%d stages)\n",
               CLR_YELLOW, CLR_RESET, motor_start, motor_end, expected_motor_runs * 4);
    }

    send_arm_disarm(sock, &dest, sysid, compid, target_sys, target_comp, 1);
    usleep(200000);

    if (!set_stdin_raw_mode(&stdin_saved, &stdin_raw_changed)) {
        fprintf(stderr, "%sWARN%s Failed to set raw input mode, arrow keys may not work.\n", CLR_YELLOW, CLR_RESET);
    }

    int raw_max = ACTUATOR_RAW_MAX;
    int raw_min = 0;
    for (int m = motor_start; m <= motor_end && g_running; ++m) {
        char role_buf[96] = {0};
        const char *role = find_motor_role(&params, m, role_buf, sizeof(role_buf));
        print_banner("MOTOR TARGET", CLR_BLUE);
        if (role) {
            printf("%sMOTOR%s #%d | %sROLE%s %s\n", CLR_GREEN, CLR_RESET, m, CLR_YELLOW, CLR_RESET, role);
        } else {
            printf("%sMOTOR%s #%d\n", CLR_GREEN, CLR_RESET, m);
        }

        double requested_idle_percent = STAGE_PRE_IDLE_PERCENT;
        double fd_idle = 0.0;
        if (params_get(&params, "FD_ACT_MOT_THR", &fd_idle) && fd_idle > 0.0 && fd_idle < 1.0) {
            requested_idle_percent = fd_idle * 100.0;
        }
        if (requested_idle_percent < 15.0) requested_idle_percent = 15.0;
        if (requested_idle_percent > 20.0) requested_idle_percent = 20.0;
        int raw_requested_idle = raw_from_percent(requested_idle_percent);

        int step_base = (m - motor_start) * 4;
        int total_steps = expected_motor_runs * 4;
        char stage_title[128];

        snprintf(stage_title, sizeof(stage_title), "STAGE %d/%d: MOTOR %d MAX (100%%)", step_base + 1, total_steps, m);
        if (!run_hold_stage(sock, &dest, sysid, compid, target_sys, target_comp, m, raw_max,
                            stage_title,
                            "ECU RC calibration start -> keep MAX. Press Enter when ECU asks MIN.",
                            loop_hz)) {
            goto cleanup;
        }

        snprintf(stage_title, sizeof(stage_title), "STAGE %d/%d: MOTOR %d MIN (0%%)", step_base + 2, total_steps, m);
        if (!run_hold_stage(sock, &dest, sysid, compid, target_sys, target_comp, m, raw_min,
                            stage_title,
                            "Now sending MIN. Press Enter when ECU asks 15~20% idle point.",
                            loop_hz)) {
            goto cleanup;
        }

        snprintf(stage_title, sizeof(stage_title), "STAGE %d/%d: MOTOR %d REQUESTED IDLE", step_base + 3, total_steps, m);
        if (!run_hold_stage(sock, &dest, sysid, compid, target_sys, target_comp, m, raw_requested_idle,
                            stage_title,
                            "Requested idle level is active. Press Enter to start fine tuning (5~30%).",
                            loop_hz)) {
            goto cleanup;
        }

        int raw_idle = raw_requested_idle;
        int raw_adj_min = raw_from_percent(STAGE_ADJUST_MIN_PERCENT);
        int raw_adj_max = raw_from_percent(STAGE_ADJUST_MAX_PERCENT);
        snprintf(stage_title, sizeof(stage_title), "STAGE %d/%d: MOTOR %d ADJUST IDLE", step_base + 4, total_steps, m);
        if (!run_adjust_stage(sock, &dest, sysid, compid, target_sys, target_comp, m,
                              raw_requested_idle, raw_adj_min, raw_adj_max, &raw_idle, stage_title, loop_hz)) {
            goto cleanup;
        }

        print_banner("MOTOR SUMMARY", CLR_BLUE);
        printf("%sMOTOR%s #%d\n", CLR_GREEN, CLR_RESET, m);
        if (role) printf("%sROLE%s %s\n", CLR_GREEN, CLR_RESET, role);
        printf("%sMAX%s %d (100.00%%)\n", CLR_GREEN, CLR_RESET, raw_max);
        printf("%sMIN%s %d (0.00%%)\n", CLR_GREEN, CLR_RESET, raw_min);
        printf("%sREQ_IDLE%s %d (%0.2f%%)\n", CLR_GREEN, CLR_RESET, raw_requested_idle, percent_from_raw(raw_requested_idle));
        printf("%sIDLE%s %d (%0.2f%%)\n", CLR_GREEN, CLR_RESET, raw_idle, percent_from_raw(raw_idle));
        printf("%sMAP%s raw<=IDLE => throttle 0%%, raw>IDLE => (raw-IDLE)*100/(%d-IDLE)\n",
               CLR_GREEN, CLR_RESET, ACTUATOR_RAW_MAX);
        printf("%sEXAMPLE%s raw %d => throttle %0.2f%%\n",
               CLR_GREEN, CLR_RESET, raw_requested_idle, throttle_from_raw(raw_requested_idle, raw_idle));

        motor_cal_result_t *r = &results[result_count++];
        r->motor_index = m;
        r->raw_max = raw_max;
        r->raw_min = raw_min;
        r->requested_idle_raw = raw_requested_idle;
        r->idle_raw = raw_idle;
        if (role) {
            snprintf(r->role, sizeof(r->role), "%s", role);
        } else {
            snprintf(r->role, sizeof(r->role), "Motor%d", m);
        }
    }

    if (g_running && result_count == (size_t)expected_motor_runs) {
        print_banner("CALIBRATION RESULT", CLR_BLUE);
        for (size_t i = 0; i < result_count; ++i) {
            const motor_cal_result_t *r = &results[i];
            printf("%sMOTOR%s #%d | idle %d (%0.2f%%) | role %s\n",
                   CLR_GREEN, CLR_RESET, r->motor_index, r->idle_raw, percent_from_raw(r->idle_raw), r->role);
        }

        if (!write_motor_config(output_path, count, results, result_count)) {
            fprintf(stderr, "%sFailed to write %s: %s%s\n", CLR_RED, output_path, strerror(errno), CLR_RESET);
            goto cleanup;
        }
        printf("%sSaved motor config to %s%s\n", CLR_GREEN, output_path, CLR_RESET);
        exit_code = 0;
    }

cleanup:
    restore_stdin_mode(&stdin_saved, stdin_raw_changed);
    print_banner("RESET OUTPUTS", CLR_BLUE);
    for (int i = 1; i <= count; ++i) {
        send_actuator(sock, &dest, sysid, compid, target_sys, target_comp, i, 0.0f);
    }
    usleep(100000);
    send_arm_disarm(sock, &dest, sysid, compid, target_sys, target_comp, 0);
    usleep(100000);
    close(sock);
    params_free(&params);
    free(results);
    return exit_code;
}
