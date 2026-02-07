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

#define DEFAULT_MOTOR_COUNT 8
#define MAIN_OUTPUTS 8
#define DEFAULT_MIN_PWM 1000
#define DEFAULT_MAX_PWM 2000

#define WAVE_PERIOD_SEC 1.0
#define LOOP_HZ 50.0

static volatile sig_atomic_t g_running = 1;

typedef struct {
    char name[64];
    double value;
} param_t;

typedef struct {
    param_t *items;
    size_t count;
} param_list_t;

static int params_get(const param_list_t *list, const char *name, double *out);

static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
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

static void usage(const char *prog, const char *ip_default, int port_default,
                  int sysid_default, int compid_default, int target_sys_default, int target_comp_default) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  -c <config>     : config.params (선택, 기본 /tmp -> ~/mavlink_projects/scan)\n"
        "  -a <airframe>   : airframe 파일(추가 파라미터용, 선택)\n"
        "  -n <count>      : 모터 개수 강제 지정\n"
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
    const char *ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, DEFAULT_IP);
    int port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, DEFAULT_PORT);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, DEFAULT_COMPID);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, DEFAULT_TARGET_COMP);
    int force_count = 0;

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
    if (force_count > 0) {
        count = force_count;
    } else {
        count = detect_servo_count(&params);
        if (count <= 0) {
            count = DEFAULT_MOTOR_COUNT;
            fprintf(stderr, "No motor count detected. Using default count %d.\n", count);
        }
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
    printf("%sCOUNT%s %d\n", CLR_YELLOW, CLR_RESET, count);
    printf("%sINPUT%s Enter = next motor, q + Enter = quit\n", CLR_YELLOW, CLR_RESET);

    send_arm_disarm(sock, &dest, sysid, compid, target_sys, target_comp, 1);
    usleep(200000);

    int servo = 1;
    char input_buf[64];

    while (g_running && servo <= count) {
        char role_buf[64];
        const char *role = get_servo_role_title(&params, servo, role_buf, sizeof(role_buf));

        print_banner("MOTOR STEP", CLR_BLUE);
        if (role) {
            printf("%sMOTOR%s %d | %sROLE%s %s | %sVALUE%s -1..1\n",
                   CLR_GREEN, CLR_RESET, servo,
                   CLR_YELLOW, CLR_RESET, role,
                   CLR_GREEN, CLR_RESET);
        } else {
            printf("%sMOTOR%s %d | %sVALUE%s -1..1\n",
                   CLR_GREEN, CLR_RESET, servo,
                   CLR_GREEN, CLR_RESET);
        }

        double start = now_seconds();
        for (;;) {
            double t = now_seconds() - start;
            double phase = (2.0 * M_PI) * (t / WAVE_PERIOD_SEC);
            float u = (float)sin(phase);

            send_actuator(sock, &dest, sysid, compid, target_sys, target_comp, servo, u);
            printf("\r%sVALUE%s %+0.3f   ", CLR_CYAN, CLR_RESET, u);
            fflush(stdout);

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = (suseconds_t)(1e6 / LOOP_HZ);

            int ready = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
            if (ready > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
                if (!fgets(input_buf, sizeof(input_buf), stdin)) {
                    g_running = 0;
                    break;
                }
                trim(input_buf);
                if (input_buf[0] == 'q' || input_buf[0] == 'Q') {
                    g_running = 0;
                    break;
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
    for (int i = 1; i <= count; ++i) {
        send_actuator(sock, &dest, sysid, compid, target_sys, target_comp, i, 0.0f);
    }
    usleep(100000);
    send_arm_disarm(sock, &dest, sysid, compid, target_sys, target_comp, 0);
    usleep(100000);
    close(sock);
    params_free(&params);
    return 0;
}
