#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <netinet/in.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>
#include "rocket_common.h"
#include "rocket_mav_common.h"

#define DEFAULT_LISTEN_IP   "127.0.0.1"
#define DEFAULT_LISTEN_PORT 14511
#define DEFAULT_SELECT_TIMEOUT 0.2
#define DEFAULT_OUTPUT_FILE "/tmp/monitoring_last"
#define DEFAULT_MODE_STATE_PATH "/tmp/rocket-mav-mode.env"
#define TIME_SYNC_THRESHOLD_SEC 60
#define TIME_SYNC_RETRY_INTERVAL_SEC 10.0
#define MIN_VALID_UNIX_USEC 1577836800000000ULL /* 2020-01-01T00:00:00Z */
#define HEARTBEAT_STALE_SEC 2.0
#define STREAM_REQ_RETRY_SEC 5.0
#define STREAM_REQ_INTERVAL_US 500000.0f /* 2 Hz */

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

static void send_message_interval_request(int sock, const struct sockaddr_in *dest,
                                          int sysid, int compid, int target_sys, int target_comp,
                                          uint16_t msgid, float interval_us) {
    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                                  (uint8_t)target_sys, (uint8_t)target_comp,
                                  MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                  (float)msgid, interval_us, 0, 0, 0, 0, 0);
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)sendto(sock, tx, tx_len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void write_value(const char *value) {
    const char *output_file = mav_cfg_get_str(MAV_CFG_KEY_MONITORING_OUTPUT_FILE, DEFAULT_OUTPUT_FILE);
    char tmp_path[PATH_MAX];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp.%ld", output_file, (long)getpid());

    FILE *fp = fopen(tmp_path, "w");
    if (!fp) return;
    fprintf(fp, "%s", value);
    fclose(fp);
    if (rename(tmp_path, output_file) != 0) {
        unlink(tmp_path);
    }
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

static int load_mode_state_cached(void) {
    static struct timespec last_mtim = {0, 0};
    static int last_mode = 0;
    char path[PATH_MAX];
    mode_state_path(path, sizeof(path));

    struct stat st;
    if (stat(path, &st) != 0) {
        last_mtim.tv_sec = 0;
        last_mtim.tv_nsec = 0;
        last_mode = 0;
        return last_mode;
    }
    if (st.st_mtim.tv_sec == last_mtim.tv_sec &&
        st.st_mtim.tv_nsec == last_mtim.tv_nsec) {
        return last_mode;
    }

    FILE *fp = fopen(path, "r");
    if (!fp) return last_mode;

    int mode_state = 0;
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
        if (strcmp(key, "MODE") == 0) {
            if (strcasecmp(val, "TEST") == 0) mode_state = 1;
            else if (strcasecmp(val, "LAUNCH") == 0) mode_state = 2;
            break;
        }
    }
    fclose(fp);
    last_mtim = st.st_mtim;
    last_mode = mode_state;
    return last_mode;
}

/*
 * Try to align system time from MAVLink SYSTEM_TIME.time_unix_usec.
 * Return 1 on successful set, 0 when no update needed/invalid input, <0 on error (-errno).
 */
static int maybe_sync_realtime_from_mavlink(uint64_t time_unix_usec, long *out_diff_sec) {
    long threshold_sec = mav_cfg_get_int(MAV_CFG_KEY_MONITORINGD_TIME_SYNC_THRESHOLD_SEC, TIME_SYNC_THRESHOLD_SEC);
    if (threshold_sec < 1) threshold_sec = TIME_SYNC_THRESHOLD_SEC;
    if (time_unix_usec < MIN_VALID_UNIX_USEC) return 0;

    time_t local_sec = time(NULL);
    if (local_sec <= 0) return 0;

    time_t remote_sec = (time_t)(time_unix_usec / 1000000ULL);
    long diff_sec = (long)llabs((long long)remote_sec - (long long)local_sec);
    if (out_diff_sec) *out_diff_sec = diff_sec;
    if (diff_sec < threshold_sec) return 0;

    struct timespec ts;
    ts.tv_sec = remote_sec;
    ts.tv_nsec = (long)((time_unix_usec % 1000000ULL) * 1000ULL);
    if (clock_settime(CLOCK_REALTIME, &ts) == 0) return 1;

    return -errno;
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

int main(void) {
    char lock_err[160];
    int lock_fd = rocket_single_instance_acquire("monitoringd", lock_err, sizeof(lock_err));
    if (lock_fd < 0) {
        fprintf(stderr, "monitoringd: %s\n", lock_err[0] ? lock_err : "single-instance lock failed");
        return 1;
    }

    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_MONITORING_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_MONITORING_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    int local_sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int local_compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    const char *tx_ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, "127.0.0.1");
    int tx_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, 15651);
    double select_timeout = mav_cfg_get_double(MAV_CFG_KEY_MONITORINGD_SELECT_TIMEOUT_SEC, DEFAULT_SELECT_TIMEOUT);
    double time_sync_retry_interval = mav_cfg_get_double(
        MAV_CFG_KEY_MONITORINGD_TIME_SYNC_RETRY_INTERVAL_SEC, TIME_SYNC_RETRY_INTERVAL_SEC);
    double write_interval = mav_cfg_get_double(MAV_CFG_KEY_MONITORINGD_WRITE_INTERVAL_SEC, 0.5);
    if (!(select_timeout > 0.0)) select_timeout = DEFAULT_SELECT_TIMEOUT;
    if (!(time_sync_retry_interval > 0.0)) time_sync_retry_interval = TIME_SYNC_RETRY_INTERVAL_SEC;
    if (!(write_interval > 0.0)) write_interval = 0.5;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        write_value("#[fg=white,bg=colour240,bold] ⏸ #[default] #[fg=white,bg=red,bold] RDY #[default] 🔋NA 📡NA");
        rocket_single_instance_release(lock_fd);
        return 1;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)listen_port);
    if (inet_pton(AF_INET, listen_ip, &addr.sin_addr) != 1) {
        close(sock);
        write_value("#[fg=white,bg=colour240,bold] ⏸ #[default] #[fg=white,bg=red,bold] RDY #[default] 🔋NA 📡NA");
        rocket_single_instance_release(lock_fd);
        return 1;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(sock);
        write_value("#[fg=white,bg=colour240,bold] ⏸ #[default] #[fg=white,bg=red,bold] RDY #[default] 🔋NA 📡NA");
        rocket_single_instance_release(lock_fd);
        return 1;
    }

    struct sockaddr_in tx_addr;
    memset(&tx_addr, 0, sizeof(tx_addr));
    tx_addr.sin_family = AF_INET;
    tx_addr.sin_port = htons((uint16_t)tx_port);
    if (inet_pton(AF_INET, tx_ip, &tx_addr.sin_addr) != 1) {
        tx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        tx_addr.sin_port = htons((uint16_t)tx_port);
    }

    int have_percent = 0;
    int percent = -1;
    int have_voltage = 0;
    double voltage = 0.0;
    int have_gps_sats = 0;
    int gps_sats = -1;
    int have_arm = 0;
    int armed = 0;
    double last_arm_update = 0.0;
    int have_hb = 0;
    uint8_t hb_system_status = MAV_STATE_UNINIT;
    double last_hb_update = 0.0;
    int have_sys_status = 0;
    int ready_to_fly_available = 0;
    int ready_to_fly = 0;
    int all_sensors_healthy_available = 0;
    int all_sensors_healthy = 0;
    double last_sys_status_update = 0.0;

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double last_write = 0.0;
    double last_time_sync_try = 0.0;
    double last_time_sync_warn = 0.0;
    double last_stream_req = 0.0;
    char out[160];
    int mode_state = load_mode_state_cached();
    format_monitoring(mode_state, have_arm, armed, 0, have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats, out, sizeof(out));
    write_value(out);

    while (g_running) {
        double now = monotonic_seconds();
        if ((now - last_stream_req) >= STREAM_REQ_RETRY_SEC) {
            send_message_interval_request(sock, &tx_addr, local_sysid, local_compid,
                                          target_sys, target_comp,
                                          MAVLINK_MSG_ID_SYS_STATUS, STREAM_REQ_INTERVAL_US);
            send_message_interval_request(sock, &tx_addr, local_sysid, local_compid,
                                          target_sys, target_comp,
                                          MAVLINK_MSG_ID_GPS_RAW_INT, STREAM_REQ_INTERVAL_US);
            last_stream_req = now;
        }

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

        if (ready > 0) {
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
                            last_arm_update = monotonic_seconds();
                            have_hb = 1;
                            hb_system_status = hb.system_status;
                            last_hb_update = last_arm_update;
                        }
                    } else if (message.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
                        double now_mono = monotonic_seconds();
                        if ((now_mono - last_time_sync_try) >= time_sync_retry_interval) {
                            mavlink_system_time_t st;
                            long diff_sec = 0;
                            mavlink_msg_system_time_decode(&message, &st);
                            int sync_res = maybe_sync_realtime_from_mavlink(st.time_unix_usec, &diff_sec);
                            last_time_sync_try = now_mono;

                            if (sync_res == 1) {
                                fprintf(stderr, "[monitoringd] system time updated from MAVLink (diff=%lds)\n", diff_sec);
                            } else if (sync_res < 0 && (now_mono - last_time_sync_warn) >= 60.0) {
                                if (sync_res == -EPERM) {
                                    fprintf(stderr, "[monitoringd] no permission to set system time (need CAP_SYS_TIME/root)\n");
                                } else {
                                    fprintf(stderr, "[monitoringd] failed to set system time: %s\n", strerror(-sync_res));
                                }
                                last_time_sync_warn = now_mono;
                            }
                        }
                    }
                }
            }
        }

        now = monotonic_seconds();
        if ((now - last_write) >= write_interval) {
            mode_state = load_mode_state_cached();
            int arm_fresh = (have_arm && (now - last_arm_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
            int hb_fresh = (have_hb && (now - last_hb_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
            int sys_status_fresh = (have_sys_status && (now - last_sys_status_update) <= HEARTBEAT_STALE_SEC) ? 1 : 0;
            int ready_level = compute_ready_level(mode_state,
                                                  hb_fresh, hb_system_status,
                                                  sys_status_fresh, ready_to_fly_available,
                                                  ready_to_fly,
                                                  all_sensors_healthy_available,
                                                  all_sensors_healthy);
            format_monitoring(mode_state, arm_fresh, armed, ready_level, have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats, out, sizeof(out));
            write_value(out);
            last_write = now;
        }
    }

    close(sock);
    rocket_single_instance_release(lock_fd);
    return 0;
}
