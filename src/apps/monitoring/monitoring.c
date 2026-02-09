#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <netinet/in.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>
#include "rocket_mav_common.h"

#define DEFAULT_LISTEN_IP   "127.0.0.1"
#define DEFAULT_LISTEN_PORT 14550
#define MAX_WAIT_SEC 0.25
#define SELECT_TIMEOUT 0.05
#define DEFAULT_MODE_STATE_PATH "/tmp/rocket-mav-mode.env"
#define HEARTBEAT_STALE_SEC 2.0

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void mode_state_path(char *out, size_t out_len) {
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
    snprintf(out, out_len, "%s", DEFAULT_MODE_STATE_PATH);
}

static int load_mode_state(void) {
    char path[PATH_MAX];
    mode_state_path(path, sizeof(path));

    FILE *fp = fopen(path, "r");
    if (!fp) return 0;

    char line[256];
    int state = 0;
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
        if (strcmp(key, "MODE") == 0) {
            if (strcasecmp(val, "TEST") == 0) state = 1;
            else if (strcasecmp(val, "LAUNCH") == 0) state = 2;
            break;
        }
    }
    fclose(fp);
    return state;
}

static void format_arm_state(int have_arm, int armed, char *out, size_t out_len) {
    if (have_arm && armed) {
        snprintf(out, out_len, "#[fg=black,bg=green,bold] 🚀 #[default]");
    } else {
        snprintf(out, out_len, "#[fg=white,bg=colour240,bold] ⏸ #[default]");
    }
}

static int compute_ready_level(int mode_state,
                               int hb_fresh, uint8_t hb_system_status,
                               int sys_status_fresh, int ready_to_fly_available,
                               int ready_to_fly,
                               int all_sensors_healthy_available,
                               int all_sensors_healthy) {
    if (!hb_fresh) return 0;

    /* HEARTBEAT state has absolute priority over any cached/derived fields. */
    if (hb_system_status == MAV_STATE_CRITICAL ||
        hb_system_status == MAV_STATE_EMERGENCY ||
        hb_system_status == MAV_STATE_POWEROFF ||
        hb_system_status == MAV_STATE_FLIGHT_TERMINATION) {
        return 0;
    }
    if (hb_system_status == MAV_STATE_UNINIT) {
        return 0;
    }
    int level = 1;
    if (hb_system_status == MAV_STATE_STANDBY || hb_system_status == MAV_STATE_ACTIVE) {
        level = 2;
    } else if (hb_system_status == MAV_STATE_BOOT || hb_system_status == MAV_STATE_CALIBRATING) {
        level = 1;
    }

    if (sys_status_fresh) {
        if (ready_to_fly_available) {
            if (!ready_to_fly) return 1;
            if (level > 1) return 2;
            return level;
        }
        if (mode_state != 1 && all_sensors_healthy_available && !all_sensors_healthy && level > 1) {
            level = 1;
        }
    }

    return level;
}

static void format_ready_state(int ready_level, char *out, size_t out_len) {
    if (ready_level >= 2) {
        snprintf(out, out_len, "#[fg=black,bg=green,bold] RDY #[default]");
    } else if (ready_level == 1) {
        snprintf(out, out_len, "#[fg=black,bg=yellow,bold] RDY #[default]");
    } else {
        snprintf(out, out_len, "#[fg=white,bg=red,bold] RDY #[default]");
    }
}

static void format_mode_state(int mode_state, char *out, size_t out_len) {
    if (mode_state == 1) {
        snprintf(out, out_len, "#[fg=white,bg=blue,bold] 🧪 #[default]");
    } else if (mode_state == 2) {
        snprintf(out, out_len, "#[fg=black,bg=yellow,bold] 🚀M #[default]");
    } else {
        snprintf(out, out_len, "#[fg=black,bg=green,bold] ⚙ #[default]");
    }
}

static void format_monitoring(int mode_state, int have_arm, int armed, int ready_level,
                              int have_percent, int percent, int have_voltage, double voltage,
                              int have_gps_sats, int gps_sats, char *out, size_t out_len) {
    char mode[96];
    char arm[96];
    char rdy[96];
    char batt[32] = "🔋NA";
    char gps[32] = "📡NA";
    format_mode_state(mode_state, mode, sizeof(mode));
    format_arm_state(have_arm, armed, arm, sizeof(arm));
    format_ready_state(ready_level, rdy, sizeof(rdy));

    if (have_percent) {
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        snprintf(batt, sizeof(batt), "🔋%d%%", percent);
    } else if (have_voltage) {
        snprintf(batt, sizeof(batt), "🔋%.1fV", voltage);
    }

    if (have_gps_sats) {
        if (gps_sats < 0) gps_sats = 0;
        snprintf(gps, sizeof(gps), "📡%d", gps_sats);
    }

    snprintf(out, out_len, "%s %s %s %s %s", mode, arm, rdy, batt, gps);
}

static void print_monitoring(int mode_state, int have_arm, int armed, int ready_level,
                             int have_percent, int percent, int have_voltage, double voltage,
                             int have_gps_sats, int gps_sats) {
    char out[160];
    format_monitoring(mode_state, have_arm, armed, ready_level, have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats, out, sizeof(out));
    printf("%s", out);
}

static int print_cached_monitoring(void) {
    const char *output_file = mav_cfg_get_str(MAV_CFG_KEY_MONITORING_OUTPUT_FILE, "/tmp/monitoring_last");
    FILE *fp = fopen(output_file, "r");
    if (!fp) return 0;

    char buf[256];
    size_t n = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    if (n == 0) return 0;

    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) n--;
    buf[n] = '\0';
    if (n == 0) return 0;
    printf("%s", buf);
    return 1;
}

int main(int argc, char **argv) {
    int live_mode = 0;
    if (argc > 1) {
        if (strcmp(argv[1], "--live") == 0) {
            live_mode = 1;
        } else if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            printf("Usage: monitoring [--live]\n");
            printf("  default : print cached /tmp/monitoring_last (same as tmux)\n");
            printf("  --live  : query MAVLink directly once\n");
            return 0;
        }
    }
    if (!live_mode) {
        if (print_cached_monitoring()) return 0;
        print_monitoring(0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0);
        return 0;
    }

    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_MONITORING_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_MONITORING_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    double max_wait_sec = mav_cfg_get_double(MAV_CFG_KEY_MONITORING_QUERY_MAX_WAIT_SEC, MAX_WAIT_SEC);
    double select_timeout = mav_cfg_get_double(MAV_CFG_KEY_MONITORING_QUERY_SELECT_TIMEOUT_SEC, SELECT_TIMEOUT);
    if (!(max_wait_sec > 0.0)) max_wait_sec = MAX_WAIT_SEC;
    if (!(select_timeout > 0.0)) select_timeout = SELECT_TIMEOUT;
    if (select_timeout > max_wait_sec) select_timeout = max_wait_sec;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        print_monitoring(0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0);
        return 0;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)listen_port);
    if (inet_pton(AF_INET, listen_ip, &addr.sin_addr) != 1) {
        close(sock);
        print_monitoring(0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0);
        return 0;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(sock);
        print_monitoring(0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0);
        return 0;
    }

    int have_percent = 0;
    int percent = -1;
    int have_voltage = 0;
    double voltage = 0.0;
    int have_gps_sats = 0;
    int gps_sats = -1;
    int have_arm = 0;
    int armed = 0;
    int have_hb = 0;
    uint8_t hb_system_status = MAV_STATE_UNINIT;
    double last_hb_update = 0.0;
    int have_sys_status = 0;
    int ready_to_fly_available = 0;
    int ready_to_fly = 0;
    int all_sensors_healthy_available = 0;
    int all_sensors_healthy = 0;
    double last_sys_status_update = 0.0;
    int mode_state = load_mode_state();

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double start = monotonic_seconds();

    while ((monotonic_seconds() - start) < max_wait_sec) {
        struct timeval tv;
        tv.tv_sec = (time_t)select_timeout;
        tv.tv_usec = (suseconds_t)((select_timeout - tv.tv_sec) * 1e6);

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);

        int ready = select(sock + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ready == 0) continue;

        unsigned char buf[2048];
        ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) {
                if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
                    mavlink_sys_status_t sys;
                    mavlink_msg_sys_status_decode(&message, &sys);
                    if (sys.battery_remaining >= 0 && sys.battery_remaining <= 100) {
                        have_percent = 1;
                        percent = (int)sys.battery_remaining;
                    }
                    if (sys.voltage_battery > 0 && sys.voltage_battery != UINT16_MAX) {
                        have_voltage = 1;
                        voltage = (double)sys.voltage_battery / 1000.0;
                    }
                        if ((int)message.sysid == target_sys) {
                            have_sys_status = 1;
                            ready_to_fly_available = ((sys.onboard_control_sensors_enabled & MAV_SYS_STATUS_PREARM_CHECK) != 0U) ? 1 : 0;
                            ready_to_fly = ((sys.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0U) ? 1 : 0;
                            uint32_t enabled_present = sys.onboard_control_sensors_enabled & sys.onboard_control_sensors_present;
                            if (enabled_present != 0U) {
                                all_sensors_healthy_available = 1;
                                all_sensors_healthy = ((enabled_present & sys.onboard_control_sensors_health) == enabled_present) ? 1 : 0;
                            } else {
                                all_sensors_healthy_available = 0;
                                all_sensors_healthy = 0;
                            }
                            last_sys_status_update = monotonic_seconds();
                        }
                } else if (message.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                    mavlink_battery_status_t batt;
                    mavlink_msg_battery_status_decode(&message, &batt);
                    if (batt.battery_remaining >= 0 && batt.battery_remaining <= 100) {
                        have_percent = 1;
                        percent = (int)batt.battery_remaining;
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&message, &gps);
                    if (gps.satellites_visible != UINT8_MAX) {
                        have_gps_sats = 1;
                        gps_sats = (int)gps.satellites_visible;
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&message, &hb);
                    if ((int)message.sysid == target_sys &&
                        hb.autopilot != MAV_AUTOPILOT_INVALID) {
                        have_arm = 1;
                        armed = ((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0) ? 1 : 0;
                        have_hb = 1;
                        hb_system_status = hb.system_status;
                        last_hb_update = monotonic_seconds();
                    }
                }

                if (have_percent || have_voltage || have_gps_sats || have_arm) {
                    double now = monotonic_seconds();
                    int arm_fresh = (have_arm && (now - last_hb_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
                    int hb_fresh = (have_hb && (now - last_hb_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
                    int sys_status_fresh = (have_sys_status && (now - last_sys_status_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
                    int ready_level = compute_ready_level(mode_state,
                                                          hb_fresh, hb_system_status,
                                                          sys_status_fresh, ready_to_fly_available,
                                                          ready_to_fly,
                                                          all_sensors_healthy_available,
                                                          all_sensors_healthy);
                    print_monitoring(mode_state, arm_fresh, armed, ready_level, have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats);
                    close(sock);
                    return 0;
                }
            }
        }
    }

    {
        double now = monotonic_seconds();
        int arm_fresh = (have_arm && (now - last_hb_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
        int hb_fresh = (have_hb && (now - last_hb_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
        int sys_status_fresh = (have_sys_status && (now - last_sys_status_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
        int ready_level = compute_ready_level(mode_state,
                                              hb_fresh, hb_system_status,
                                              sys_status_fresh, ready_to_fly_available,
                                              ready_to_fly,
                                              all_sensors_healthy_available,
                                              all_sensors_healthy);
        print_monitoring(mode_state, arm_fresh, armed, ready_level, have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats);
    }
    close(sock);
    return 0;
}
