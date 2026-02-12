#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stddef.h>
#include <signal.h>
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

#define DEFAULT_LISTEN_IP MAV_DEFAULT_IP
#define DEFAULT_LISTEN_PORT 15552
#define SELECT_TIMEOUT_SEC 0.25

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

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Options:\n"
            "  -l <ip>    listen ip (default %s)\n"
            "  -p <port>  listen port (default %d)\n"
            "  -h         help\n",
            prog, DEFAULT_LISTEN_IP, DEFAULT_LISTEN_PORT);
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_HEARTBEAT_WATCH_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_HEARTBEAT_WATCH_LISTEN_PORT, DEFAULT_LISTEN_PORT);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)listen_port);
    if (inet_pton(AF_INET, listen_ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP: %s\n", listen_ip);
        close(sock);
        return 1;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "Failed to bind %s:%d: %s\n", listen_ip, listen_port, strerror(errno));
        close(sock);
        return 1;
    }

    printf("heartbeat_watch: listening %s:%d\n", listen_ip, listen_port);

    mavlink_message_t message;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (g_running) {
        struct timeval tv;
        tv.tv_sec = (time_t)SELECT_TIMEOUT_SEC;
        tv.tv_usec = (suseconds_t)((SELECT_TIMEOUT_SEC - (double)tv.tv_sec) * 1e6);

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);

        int ready = select(sock + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (ready == 0) continue;

        unsigned char buf[2048];
        ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recvfrom");
            break;
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &status)) {
                if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&message, &hb);
                    printf("t=%.3f sysid=%u compid=%u type=%u autopilot=%u base_mode=0x%02X custom_mode=0x%08X status=%u\n",
                           monotonic_seconds(),
                           message.sysid,
                           message.compid,
                           (unsigned)hb.type,
                           (unsigned)hb.autopilot,
                           (unsigned)hb.base_mode,
                           (unsigned)hb.custom_mode,
                           (unsigned)hb.system_status);
                    fflush(stdout);
                }
            }
        }
    }

    close(sock);
    return 0;
}
