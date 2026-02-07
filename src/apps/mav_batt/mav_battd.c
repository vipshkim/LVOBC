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
#define OUTPUT_FILE "/tmp/mav_batt_last"

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

static void format_battery(int have_percent, int percent, int have_voltage, double voltage, char *out, size_t out_len) {
    if (have_percent) {
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        snprintf(out, out_len, "BATT:%d%%", percent);
        return;
    }
    if (have_voltage) {
        snprintf(out, out_len, "BATT:%.1fV", voltage);
        return;
    }
    snprintf(out, out_len, "BATT:--");
}

int main(void) {
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        write_value("BATT:--");
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
        write_value("BATT:--");
        return 1;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(sock);
        write_value("BATT:--");
        return 1;
    }

    int have_percent = 0;
    int percent = -1;
    int have_voltage = 0;
    double voltage = 0.0;

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    double last_write = 0.0;
    char out[64];
    format_battery(have_percent, percent, have_voltage, voltage, out, sizeof(out));
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
                    }
                }
            }
        }

        double now = monotonic_seconds();
        if ((now - last_write) >= 0.5) {
            format_battery(have_percent, percent, have_voltage, voltage, out, sizeof(out));
            write_value(out);
            last_write = now;
        }
    }

    close(sock);
    return 0;
}
