#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
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

#define LISTEN_IP   "127.0.0.1"
#define LISTEN_PORT 14550
#define SELECT_TIMEOUT 0.2
#define OUTPUT_FILE "/tmp/monitoring_last"
#define TIME_SYNC_THRESHOLD_SEC 60
#define TIME_SYNC_RETRY_INTERVAL_SEC 10.0
#define MIN_VALID_UNIX_USEC 1577836800000000ULL /* 2020-01-01T00:00:00Z */

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

static void write_value(const char *value) {
    char tmp_path[256];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", OUTPUT_FILE);

    FILE *fp = fopen(tmp_path, "w");
    if (!fp) return;
    fprintf(fp, "%s", value);
    fclose(fp);
    rename(tmp_path, OUTPUT_FILE);
}

/*
 * Try to align system time from MAVLink SYSTEM_TIME.time_unix_usec.
 * Return 1 on successful set, 0 when no update needed/invalid input, <0 on error (-errno).
 */
static int maybe_sync_realtime_from_mavlink(uint64_t time_unix_usec, long *out_diff_sec) {
    if (time_unix_usec < MIN_VALID_UNIX_USEC) return 0;

    time_t local_sec = time(NULL);
    if (local_sec <= 0) return 0;

    time_t remote_sec = (time_t)(time_unix_usec / 1000000ULL);
    long diff_sec = (long)llabs((long long)remote_sec - (long long)local_sec);
    if (out_diff_sec) *out_diff_sec = diff_sec;
    if (diff_sec < TIME_SYNC_THRESHOLD_SEC) return 0;

    struct timespec ts;
    ts.tv_sec = remote_sec;
    ts.tv_nsec = (long)((time_unix_usec % 1000000ULL) * 1000ULL);
    if (clock_settime(CLOCK_REALTIME, &ts) == 0) return 1;

    return -errno;
}

static void format_monitoring(int have_percent, int percent, int have_voltage, double voltage,
                              int have_gps_sats, int gps_sats, char *out, size_t out_len) {
    char batt[32] = "BATT=NA";
    if (have_percent) {
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        snprintf(batt, sizeof(batt), "BATT=%d%%", percent);
    } else if (have_voltage) {
        snprintf(batt, sizeof(batt), "BATT=%.1fV", voltage);
    }

    if (have_gps_sats) {
        if (gps_sats < 0) gps_sats = 0;
        snprintf(out, out_len, "%s GPS_SATS=%d", batt, gps_sats);
    } else {
        snprintf(out, out_len, "%s GPS_SATS=NA", batt);
    }
}

int main(void) {
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        write_value("BATT=NA GPS_SATS=NA");
        return 1;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LISTEN_PORT);
    if (inet_pton(AF_INET, LISTEN_IP, &addr.sin_addr) != 1) {
        close(sock);
        write_value("BATT=NA GPS_SATS=NA");
        return 1;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(sock);
        write_value("BATT=NA GPS_SATS=NA");
        return 1;
    }

    int have_percent = 0;
    int percent = -1;
    int have_voltage = 0;
    double voltage = 0.0;
    int have_gps_sats = 0;
    int gps_sats = -1;

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double last_write = 0.0;
    double last_time_sync_try = 0.0;
    double last_time_sync_warn = 0.0;
    char out[64];
    format_monitoring(have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats, out, sizeof(out));
    write_value(out);

    while (g_running) {
        struct timeval tv;
        tv.tv_sec = (time_t)SELECT_TIMEOUT;
        tv.tv_usec = (suseconds_t)((SELECT_TIMEOUT - tv.tv_sec) * 1e6);

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
                    } else if (message.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
                        double now_mono = monotonic_seconds();
                        if ((now_mono - last_time_sync_try) >= TIME_SYNC_RETRY_INTERVAL_SEC) {
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

        double now = monotonic_seconds();
        if ((now - last_write) >= 0.5) {
            format_monitoring(have_percent, percent, have_voltage, voltage, have_gps_sats, gps_sats, out, sizeof(out));
            write_value(out);
            last_write = now;
        }
    }

    close(sock);
    return 0;
}
