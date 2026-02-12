#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
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

#define DEFAULT_TIMEOUT_SEC 2.0

typedef enum {
    CMD_ARM = 0,
    CMD_DISARM,
    CMD_TAKEOFF,
    CMD_LAND,
    CMD_RTL,
    CMD_MODE,
    CMD_SETHOME,
    CMD_REBOOT,
    CMD_SHUTDOWN,
    CMD_MAVCMD
} commander_cmd_t;

typedef struct {
    const char *ip;
    int port;
    int listen_port;
    int sysid;
    int compid;
    int target_sys;
    int target_comp;
    double timeout_sec;
} commander_cfg_t;

typedef struct {
    const char *name;
    uint8_t base_mode;
    uint8_t custom_main;
    uint8_t custom_sub;
} px4_mode_map_t;

enum {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2,
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3,
    PX4_CUSTOM_MAIN_MODE_AUTO = 4,
    PX4_CUSTOM_MAIN_MODE_ACRO = 5,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6,
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
};

enum {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6,
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7,
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8,
};

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static const char *ack_result_str(uint8_t r) {
    switch (r) {
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

static void to_upper_copy(char *dst, size_t dst_len, const char *src) {
    if (!dst || dst_len == 0) return;
    size_t i = 0;
    for (; src && src[i] && i + 1 < dst_len; ++i) {
        unsigned char c = (unsigned char)src[i];
        if (c == '-' || c == ' ') c = '_';
        dst[i] = (char)toupper(c);
    }
    dst[i] = '\0';
}

static int parse_common_opt(int argc, char **argv, int *i, commander_cfg_t *cfg) {
    if (strcmp(argv[*i], "-u") == 0 && *i + 1 < argc) {
        cfg->ip = argv[++(*i)];
        return 1;
    }
    if (strcmp(argv[*i], "-p") == 0 && *i + 1 < argc) {
        cfg->port = atoi(argv[++(*i)]);
        return 1;
    }
    if (strcmp(argv[*i], "-l") == 0 && *i + 1 < argc) {
        cfg->listen_port = atoi(argv[++(*i)]);
        return 1;
    }
    if (strcmp(argv[*i], "-s") == 0 && *i + 1 < argc) {
        cfg->sysid = atoi(argv[++(*i)]);
        return 1;
    }
    if (strcmp(argv[*i], "-c") == 0 && *i + 1 < argc) {
        cfg->compid = atoi(argv[++(*i)]);
        return 1;
    }
    if (strcmp(argv[*i], "-t") == 0 && *i + 1 < argc) {
        cfg->target_sys = atoi(argv[++(*i)]);
        return 1;
    }
    if (strcmp(argv[*i], "-k") == 0 && *i + 1 < argc) {
        cfg->target_comp = atoi(argv[++(*i)]);
        return 1;
    }
    if (strcmp(argv[*i], "-w") == 0 && *i + 1 < argc) {
        cfg->timeout_sec = atof(argv[++(*i)]);
        return 1;
    }
    return 0;
}

static int wait_command_ack(int fd, uint16_t command, double timeout_sec, uint8_t *out_result) {
    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t rx_msg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0.0) remaining = 0.0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (time_t)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            return -1;
        }
        if (ready == 0) continue;

        uint8_t rx[2048];
        ssize_t n = recvfrom(fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &rx_msg, &status)) continue;
            if (rx_msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) continue;

            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&rx_msg, &ack);
            if (ack.command != command) continue;

            if (out_result) *out_result = ack.result;
            printf("ACK cmd=%u %s(%u) rp2=%d progress=%u tsys=%u tcomp=%u\n",
                   (unsigned)ack.command,
                   ack_result_str(ack.result), (unsigned)ack.result,
                   ack.result_param2, (unsigned)ack.progress,
                   (unsigned)rx_msg.sysid, (unsigned)rx_msg.compid);
            return 0;
        }
    }

    return 1;
}

static int send_command_long(int fd, const struct sockaddr_in *dest,
                             int sysid, int compid, int target_sys, int target_comp,
                             uint16_t command,
                             float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
        (uint8_t)sysid, (uint8_t)compid, &msg,
        (uint8_t)target_sys, (uint8_t)target_comp,
        command, 0,
        p1, p2, p3, p4, p5, p6, p7);
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)sendto(fd, tx, tx_len, 0, (const struct sockaddr *)dest, sizeof(*dest));
    return 0;
}

static int send_set_mode(int fd, const struct sockaddr_in *dest,
                         int sysid, int compid, int target_sys, int target_comp,
                         uint8_t base_mode, uint8_t custom_main, uint8_t custom_sub) {
    return send_command_long(fd, dest,
                             sysid, compid, target_sys, target_comp,
                             MAV_CMD_DO_SET_MODE,
                             (float)(base_mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                             (float)custom_main,
                             (float)custom_sub,
                             0.f, 0.f, 0.f, 0.f);
}

static int send_set_home(int fd, const struct sockaddr_in *dest,
                         int sysid, int compid, int target_sys, int target_comp,
                         int use_current, double lat, double lon, double alt) {
    return send_command_long(fd, dest,
                             sysid, compid, target_sys, target_comp,
                             MAV_CMD_DO_SET_HOME,
                             use_current ? 1.0f : 0.0f,
                             NAN, NAN, NAN,
                             (float)lat, (float)lon, (float)alt);
}

static const px4_mode_map_t *find_px4_mode(const char *name) {
    static const uint8_t auto_flags = (uint8_t)(MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG_GUIDED_ENABLED);
    static const px4_mode_map_t modes[] = {
        {"MANUAL", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                             MAV_MODE_FLAG_STABILIZE_ENABLED |
                             MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
         PX4_CUSTOM_MAIN_MODE_MANUAL, 0},
        {"STABILIZED", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                 MAV_MODE_FLAG_STABILIZE_ENABLED |
                                 MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
         PX4_CUSTOM_MAIN_MODE_STABILIZED, 0},
        {"ACRO", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                           MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
         PX4_CUSTOM_MAIN_MODE_ACRO, 0},
        {"RATTITUDE", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
         PX4_CUSTOM_MAIN_MODE_RATTITUDE, 0},
        {"ALTCTL", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                             MAV_MODE_FLAG_STABILIZE_ENABLED |
                             MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
         PX4_CUSTOM_MAIN_MODE_ALTCTL, 0},
        {"POSCTL", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                             MAV_MODE_FLAG_STABILIZE_ENABLED |
                             MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
         PX4_CUSTOM_MAIN_MODE_POSCTL, 0},
        {"LOITER", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER},
        {"HOLD", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER},
        {"MISSION", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION},
        {"RTL", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL},
        {"LAND", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND},
        {"RTGS", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTGS},
        {"FOLLOW", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET},
        {"FOLLOWME", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET},
        {"OFFBOARD", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0},
        {"TAKEOFF", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF},
        {"AUTO", (uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_flags),
         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_READY},
    };
    char key[64];
    to_upper_copy(key, sizeof(key), name);
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
        if (strcmp(key, modes[i].name) == 0) return &modes[i];
    }
    return NULL;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s <command> [options]\n"
            "Commands:\n"
            "  arm|disarm           arm/disarm (use -f for force)\n"
            "  takeoff              NAV_TAKEOFF (use --alt/--yaw/--lat/--lon)\n"
            "  land                 NAV_LAND (use --alt/--yaw/--lat/--lon)\n"
            "  rtl                  NAV_RETURN_TO_LAUNCH\n"
            "  mode <NAME>          PX4 mode (MANUAL, POSCTL, OFFBOARD, MISSION, ...)\n"
            "  sethome              set home (use --current or --home-lat/--home-lon/--home-alt)\n"
            "  reboot|shutdown      PREFLIGHT_REBOOT_SHUTDOWN\n"
            "  mavcmd <id> [p1..p7]  generic MAV_CMD\n"
            "Options:\n"
            "  -f           force (arm/disarm param2=21196)\n"
            "  -u <ip>      target IP (default from ROCKET_MAV_TOOLS_TARGET_IP)\n"
            "  -p <port>    target ingress port (default from ROCKET_MAV_TOOLS_TARGET_PORT)\n"
            "  -l <port>    local listen port for ACK (default from ROCKET_MAV_TOOLS_LISTEN_PORT)\n"
            "  -s <sysid>   sender sysid (default ROCKET_MAV_TOOLS_SYSID)\n"
            "  -c <compid>  sender compid (default ROCKET_MAV_TOOLS_COMPID)\n"
            "  -t <sysid>   target sysid (default ROCKET_MAV_TOOLS_TARGET_SYS)\n"
            "  -k <compid>  target compid (default ROCKET_MAV_TOOLS_TARGET_COMP)\n"
            "  -w <sec>     ACK wait timeout (default %.1f)\n"
            "  -h           help\n"
            "Examples:\n"
            "  %s arm\n"
            "  %s takeoff --alt 5\n"
            "  %s mode POSCTL\n",
            prog, DEFAULT_TIMEOUT_SEC, prog, prog, prog);
}

int main(int argc, char **argv) {
    commander_cfg_t cfg;
    cfg.ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, MAV_DEFAULT_IP);
    cfg.port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, 15651);
    cfg.listen_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_LISTEN_PORT, 15551);
    cfg.sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    cfg.compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    cfg.target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    cfg.target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    cfg.timeout_sec = DEFAULT_TIMEOUT_SEC;

    commander_cmd_t cmd = CMD_ARM;
    const char *cmd_arg = NULL;
    const char *mode_name = NULL;
    uint16_t mavcmd_id = 0;
    float mavcmd_params[7] = {0};
    int mavcmd_param_count = 0;
    int use_current_home = 0;
    double home_lat = NAN, home_lon = NAN, home_alt = NAN;
    double nav_alt = NAN, nav_yaw = NAN, nav_lat = NAN, nav_lon = NAN;
    int force = 0;

    int argi = 1;
    if (argc > 1 && argv[1][0] != '-') {
        cmd_arg = argv[1];
        argi = 2;
    }

    if (!cmd_arg) {
        cmd_arg = "arm"; /* backward compatibility */
    }

    if (strcmp(cmd_arg, "arm") == 0) {
        cmd = CMD_ARM;
    } else if (strcmp(cmd_arg, "disarm") == 0) {
        cmd = CMD_DISARM;
    } else if (strcmp(cmd_arg, "takeoff") == 0) {
        cmd = CMD_TAKEOFF;
    } else if (strcmp(cmd_arg, "land") == 0) {
        cmd = CMD_LAND;
    } else if (strcmp(cmd_arg, "rtl") == 0 || strcmp(cmd_arg, "return") == 0) {
        cmd = CMD_RTL;
    } else if (strcmp(cmd_arg, "mode") == 0) {
        cmd = CMD_MODE;
    } else if (strcmp(cmd_arg, "sethome") == 0 || strcmp(cmd_arg, "set_home") == 0) {
        cmd = CMD_SETHOME;
    } else if (strcmp(cmd_arg, "reboot") == 0) {
        cmd = CMD_REBOOT;
    } else if (strcmp(cmd_arg, "shutdown") == 0) {
        cmd = CMD_SHUTDOWN;
    } else if (strcmp(cmd_arg, "mavcmd") == 0) {
        cmd = CMD_MAVCMD;
    } else if (strcmp(cmd_arg, "help") == 0) {
        usage(argv[0]);
        return 0;
    } else {
        fprintf(stderr, "invalid command: %s\n", cmd_arg);
        usage(argv[0]);
        return 1;
    }

    for (int i = argi; i < argc; ++i) {
        if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--force") == 0) {
            force = 1;
            continue;
        }
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        if (parse_common_opt(argc, argv, &i, &cfg)) continue;

        if (cmd == CMD_TAKEOFF || cmd == CMD_LAND) {
            if (strcmp(argv[i], "--alt") == 0 || strcmp(argv[i], "-a") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                nav_alt = atof(argv[++i]);
                continue;
            }
            if (strcmp(argv[i], "--yaw") == 0 || strcmp(argv[i], "-y") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                nav_yaw = atof(argv[++i]);
                continue;
            }
            if (strcmp(argv[i], "--lat") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                nav_lat = atof(argv[++i]);
                continue;
            }
            if (strcmp(argv[i], "--lon") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                nav_lon = atof(argv[++i]);
                continue;
            }
            if (argv[i][0] != '-' && isnan(nav_alt)) {
                nav_alt = atof(argv[i]);
                continue;
            }
        }

        if (cmd == CMD_MODE) {
            if (!mode_name && argv[i][0] != '-') {
                mode_name = argv[i];
                continue;
            }
        }

        if (cmd == CMD_SETHOME) {
            if (strcmp(argv[i], "--current") == 0) {
                use_current_home = 1;
                continue;
            }
            if (strcmp(argv[i], "--home-lat") == 0 || strcmp(argv[i], "--lat") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                home_lat = atof(argv[++i]);
                continue;
            }
            if (strcmp(argv[i], "--home-lon") == 0 || strcmp(argv[i], "--lon") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                home_lon = atof(argv[++i]);
                continue;
            }
            if (strcmp(argv[i], "--home-alt") == 0 || strcmp(argv[i], "--alt") == 0) {
                if (i + 1 >= argc) { usage(argv[0]); return 1; }
                home_alt = atof(argv[++i]);
                continue;
            }
        }

        if (cmd == CMD_MAVCMD) {
            if (mavcmd_id == 0 && argv[i][0] != '-') {
                mavcmd_id = (uint16_t)atoi(argv[i]);
                continue;
            }
            if (argv[i][0] != '-' && mavcmd_param_count < 7) {
                mavcmd_params[mavcmd_param_count++] = (float)atof(argv[i]);
                continue;
            }
        }

        fprintf(stderr, "unknown option: %s\n", argv[i]);
        usage(argv[0]);
        return 1;
    }

    if (cmd == CMD_MODE && !mode_name) {
        fprintf(stderr, "mode name required\n");
        usage(argv[0]);
        return 1;
    }
    if (cmd == CMD_MAVCMD && mavcmd_id == 0) {
        fprintf(stderr, "mavcmd id required\n");
        usage(argv[0]);
        return 1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons((uint16_t)cfg.listen_port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (const struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        fprintf(stderr, "bind(%d) failed: %s\n", cfg.listen_port, strerror(errno));
        close(fd);
        return 1;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)cfg.port);
    if (inet_pton(AF_INET, cfg.ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "invalid ip: %s\n", cfg.ip);
        close(fd);
        return 1;
    }

    uint16_t cmd_id = 0;
    const char *cmd_name = NULL;

    if (cmd == CMD_ARM || cmd == CMD_DISARM) {
        cmd_id = MAV_CMD_COMPONENT_ARM_DISARM;
        send_command_long(fd, &dest,
                          cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                          cmd_id,
                          cmd == CMD_ARM ? 1.0f : 0.0f,
                          force ? 21196.0f : 0.0f,
                          0.f, 0.f, 0.f, 0.f, 0.f);
        cmd_name = (cmd == CMD_ARM) ? "ARM" : "DISARM";
    } else if (cmd == CMD_TAKEOFF) {
        cmd_id = MAV_CMD_NAV_TAKEOFF;
        send_command_long(fd, &dest,
                          cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                          cmd_id,
                          0.f, 0.f, 0.f,
                          isnan(nav_yaw) ? NAN : (float)nav_yaw,
                          isnan(nav_lat) ? NAN : (float)nav_lat,
                          isnan(nav_lon) ? NAN : (float)nav_lon,
                          isnan(nav_alt) ? NAN : (float)nav_alt);
        cmd_name = "TAKEOFF";
    } else if (cmd == CMD_LAND) {
        cmd_id = MAV_CMD_NAV_LAND;
        send_command_long(fd, &dest,
                          cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                          cmd_id,
                          0.f, 0.f, 0.f,
                          isnan(nav_yaw) ? NAN : (float)nav_yaw,
                          isnan(nav_lat) ? NAN : (float)nav_lat,
                          isnan(nav_lon) ? NAN : (float)nav_lon,
                          isnan(nav_alt) ? NAN : (float)nav_alt);
        cmd_name = "LAND";
    } else if (cmd == CMD_RTL) {
        cmd_id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
        send_command_long(fd, &dest,
                          cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                          cmd_id,
                          0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
        cmd_name = "RTL";
    } else if (cmd == CMD_MODE) {
        const px4_mode_map_t *mode = find_px4_mode(mode_name);
        if (!mode) {
            fprintf(stderr, "unknown mode: %s\n", mode_name);
            close(fd);
            return 1;
        }
        cmd_id = MAV_CMD_DO_SET_MODE;
        send_set_mode(fd, &dest,
                      cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                      mode->base_mode, mode->custom_main, mode->custom_sub);
        cmd_name = "SET_MODE";
    } else if (cmd == CMD_SETHOME) {
        cmd_id = MAV_CMD_DO_SET_HOME;
        if (!use_current_home && (isnan(home_lat) || isnan(home_lon))) {
            fprintf(stderr, "sethome requires --current or --home-lat/--home-lon\n");
            close(fd);
            return 1;
        }
        send_set_home(fd, &dest,
                      cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                      use_current_home,
                      isnan(home_lat) ? NAN : home_lat,
                      isnan(home_lon) ? NAN : home_lon,
                      isnan(home_alt) ? NAN : home_alt);
        cmd_name = "SET_HOME";
    } else if (cmd == CMD_REBOOT || cmd == CMD_SHUTDOWN) {
        cmd_id = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        float p1 = (cmd == CMD_REBOOT) ? 1.0f : 2.0f;
        send_command_long(fd, &dest,
                          cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                          cmd_id,
                          p1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
        cmd_name = (cmd == CMD_REBOOT) ? "REBOOT" : "SHUTDOWN";
    } else if (cmd == CMD_MAVCMD) {
        cmd_id = mavcmd_id;
        send_command_long(fd, &dest,
                          cfg.sysid, cfg.compid, cfg.target_sys, cfg.target_comp,
                          cmd_id,
                          mavcmd_params[0], mavcmd_params[1], mavcmd_params[2],
                          mavcmd_params[3], mavcmd_params[4], mavcmd_params[5],
                          mavcmd_params[6]);
        cmd_name = "MAVCMD";
    }

    printf("TX %s -> %s:%d (tsys=%d tcomp=%d, sysid=%d compid=%d)\n",
           cmd_name ? cmd_name : "CMD",
           cfg.ip, cfg.port, cfg.target_sys, cfg.target_comp, cfg.sysid, cfg.compid);

    uint8_t ack_result = MAV_RESULT_FAILED;
    int ack_rc = wait_command_ack(fd, cmd_id, cfg.timeout_sec, &ack_result);
    if (ack_rc == 0) {
        close(fd);
        return (ack_result == MAV_RESULT_ACCEPTED) ? 0 : 2;
    }
    if (ack_rc == 1) {
        printf("ACK timeout %.1fs\n", cfg.timeout_sec);
    }

    close(fd);
    return 3;
}
