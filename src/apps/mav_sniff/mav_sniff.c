#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 0
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

#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_PORT 14651

static volatile sig_atomic_t g_running = 1;

static void handle_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void print_command_long(const mavlink_message_t *msg) {
    mavlink_command_long_t cl;
    mavlink_msg_command_long_decode(msg, &cl);
    printf("CMD_LONG sys=%u comp=%u cmd=%u p1=%.3f p2=%.3f p3=%.3f p4=%.3f p5=%.3f p6=%.3f p7=%.3f\n",
           msg->sysid, msg->compid, (unsigned)cl.command,
           cl.param1, cl.param2, cl.param3, cl.param4, cl.param5, cl.param6, cl.param7);
}

static void print_rc_override(const mavlink_message_t *msg) {
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(msg, &rc);
    printf("RC_CHANNELS_OVERRIDE sys=%u comp=%u ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u ch7=%u ch8=%u ch9=%u ch10=%u ch11=%u ch12=%u ch13=%u ch14=%u ch15=%u ch16=%u ch17=%u ch18=%u\n",
           msg->sysid, msg->compid,
           rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
           rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
           rc.chan9_raw, rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
           rc.chan13_raw, rc.chan14_raw, rc.chan15_raw, rc.chan16_raw,
           rc.chan17_raw, rc.chan18_raw);
}

static void print_message(const mavlink_message_t *msg) {
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        print_command_long(msg);
        return;
    }
    if (msg->msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
        print_rc_override(msg);
        return;
    }
#if (MAVLINK_USE_MESSAGE_INFO) && !defined(MAVLINK_NO_HELPERS)
    const mavlink_message_info_t *info = mavlink_get_message_info_by_id(msg->msgid);
    if (info && info->name) {
        printf("MSG %s(%u) sys=%u comp=%u\n", info->name, msg->msgid, msg->sysid, msg->compid);
        return;
    }
#endif
    printf("MSG ID_%u sys=%u comp=%u\n", msg->msgid, msg->sysid, msg->compid);
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [-l ip] [-p port] [-n count] [-t sec]\n"
            "  -l <ip>   listen ip (default %s)\n"
            "  -p <port> listen port (default %d)\n"
            "  -n <cnt>  stop after cnt messages (default 0 = run)\n"
            "  -t <sec>  stop after seconds (default 0 = run)\n",
            prog, DEFAULT_IP, DEFAULT_PORT);
}

int main(int argc, char **argv) {
    const char *ip = DEFAULT_IP;
    int port = DEFAULT_PORT;
    int max_msgs = 0;
    double max_sec = 0.0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            max_msgs = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            max_sec = atof(argv[++i]);
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

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid ip: %s\n", ip);
        close(fd);
        return 1;
    }
    if (bind(fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("bind");
        close(fd);
        return 1;
    }

    printf("mav_sniff: listening %s:%d\n", ip, port);
    int msg_count = 0;
    double start = now_sec();

    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (g_running) {
        if (max_msgs > 0 && msg_count >= max_msgs) break;
        if (max_sec > 0.0 && (now_sec() - start) >= max_sec) break;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (ready == 0) continue;

        uint8_t buf[2048];
        ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                print_message(&msg);
                msg_count++;
                if (max_msgs > 0 && msg_count >= max_msgs) break;
            }
        }
    }

    close(fd);
    return 0;
}
