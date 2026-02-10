#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
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

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [arm|disarm] [options]\n"
            "Options:\n"
            "  -f           force (param2=21196)\n"
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
            "  %s disarm -f\n",
            prog, DEFAULT_TIMEOUT_SEC, prog, prog);
}

int main(int argc, char **argv) {
    const char *ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, MAV_DEFAULT_IP);
    int port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, MAV_DEFAULT_PORT);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_LISTEN_PORT, 14552);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    int arm = 1;
    int force = 0;
    double timeout_sec = DEFAULT_TIMEOUT_SEC;

    int argi = 1;
    if (argc > 1 && argv[1][0] != '-') {
        if (strcmp(argv[1], "arm") == 0) {
            arm = 1;
        } else if (strcmp(argv[1], "disarm") == 0) {
            arm = 0;
        } else if (strcmp(argv[1], "help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "invalid action: %s\n", argv[1]);
            usage(argv[0]);
            return 1;
        }
        argi = 2;
    }

    for (int i = argi; i < argc; ++i) {
        if (strcmp(argv[i], "-f") == 0) {
            force = 1;
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            timeout_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
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

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons((uint16_t)listen_port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (const struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        fprintf(stderr, "bind(%d) failed: %s\n", listen_port, strerror(errno));
        close(fd);
        return 1;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "invalid ip: %s\n", ip);
        close(fd);
        return 1;
    }

    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
        (uint8_t)sysid, (uint8_t)compid, &msg,
        (uint8_t)target_sys, (uint8_t)target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        arm ? 1.0f : 0.0f,
        force ? 21196.0f : 0.0f,
        0.f, 0.f, 0.f, 0.f, 0.f);
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)sendto(fd, tx, tx_len, 0, (const struct sockaddr *)&dest, sizeof(dest));

    printf("TX %s%s -> %s:%d (tsys=%d tcomp=%d, sysid=%d compid=%d)\n",
           arm ? "ARM" : "DISARM",
           force ? " FORCE" : "",
           ip, port, target_sys, target_comp, sysid, compid);

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
            close(fd);
            return 1;
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
            if (ack.command != MAV_CMD_COMPONENT_ARM_DISARM) continue;

            printf("ACK cmd=%u %s(%u) rp2=%d progress=%u tsys=%u tcomp=%u\n",
                   (unsigned)ack.command,
                   ack_result_str(ack.result), (unsigned)ack.result,
                   ack.result_param2, (unsigned)ack.progress,
                   (unsigned)rx_msg.sysid, (unsigned)rx_msg.compid);
            close(fd);
            return (ack.result == MAV_RESULT_ACCEPTED) ? 0 : 2;
        }
    }

    printf("ACK timeout %.1fs\n", timeout_sec);
    close(fd);
    return 3;
}
