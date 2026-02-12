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
#include "rocket_mav_common.h"

#define DEFAULT_LISTEN_IP "127.0.0.1"
#define DEFAULT_LISTEN_PORT 15561
#define DEFAULT_TIMEOUT_SEC 2.0

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

static const char *severity_str(uint8_t sev) {
    switch (sev) {
        case MAV_SEVERITY_EMERGENCY: return "EMERG";
        case MAV_SEVERITY_ALERT: return "ALERT";
        case MAV_SEVERITY_CRITICAL: return "CRIT";
        case MAV_SEVERITY_ERROR: return "ERROR";
        case MAV_SEVERITY_WARNING: return "WARN";
        case MAV_SEVERITY_NOTICE: return "NOTICE";
        case MAV_SEVERITY_INFO: return "INFO";
        case MAV_SEVERITY_DEBUG: return "DEBUG";
        default: return "UNK";
    }
}

typedef struct {
    char text[96];
    uint8_t sev;
    double ts;
} statustext_cache_t;

static void to_lower_copy(char *dst, size_t dst_len, const char *src) {
    if (!dst || dst_len == 0) return;
    size_t i = 0;
    for (; src && src[i] && i + 1 < dst_len; ++i) {
        char c = src[i];
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        dst[i] = c;
    }
    dst[i] = '\0';
}

static int is_arm_related_text(const char *text) {
    char lower[96];
    to_lower_copy(lower, sizeof(lower), text);
    return (strstr(lower, "arm") != NULL) || (strstr(lower, "prearm") != NULL);
}

static int wait_for_ack(int rx_fd,
                        double timeout_sec,
                        int show_all,
                        const char *label,
                        int command,
                        mavlink_message_t *msg,
                        mavlink_status_t *status,
                        statustext_cache_t *cache) {
    double deadline = monotonic_seconds() + timeout_sec;
    while (g_running && monotonic_seconds() < deadline) {
        double now = monotonic_seconds();
        double remain = deadline - now;
        if (remain < 0.0) remain = 0.0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(rx_fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (time_t)remain;
        tv.tv_usec = (suseconds_t)((remain - (double)tv.tv_sec) * 1e6);

        int r = select(rx_fd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            return -1;
        }
        if (r == 0 || !FD_ISSET(rx_fd, &rfds)) continue;

        uint8_t buf[2048];
        ssize_t n = recvfrom(rx_fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, status)) continue;

            if (msg->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                mavlink_statustext_t st;
                mavlink_msg_statustext_decode(msg, &st);
                char text[51];
                memcpy(text, st.text, 50);
                text[50] = '\0';

                int is_arm = is_arm_related_text(text);
                if (show_all || is_arm) {
                    printf("STATUSTEXT sev=%s(%u) tsys=%u tcomp=%u %s\n",
                           severity_str(st.severity),
                           (unsigned)st.severity,
                           msg->sysid, msg->compid,
                           text);
                    fflush(stdout);
                }

                if (is_arm || st.severity <= MAV_SEVERITY_WARNING) {
                    snprintf(cache->text, sizeof(cache->text), "%s", text);
                    cache->sev = st.severity;
                    cache->ts = now;
                }
            } else if (msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(msg, &ack);
                if (ack.command != command) continue;

                printf("ACK %s %s(%u) rp2=%d prog=%u tsys=%u tcomp=%u\n",
                       label,
                       ack_result_str(ack.result),
                       (unsigned)ack.result,
                       ack.result_param2,
                       (unsigned)ack.progress,
                       msg->sysid, msg->compid);
                fflush(stdout);
                return (int)ack.result;
            }
        }
    }

    return -1;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Options:\n"
            "  -u <ip>    target IP (default from ROCKET_MAV_TOOLS_TARGET_IP)\n"
            "  -p <port>  target ingress port (default from ROCKET_MAV_TOOLS_TARGET_PORT)\n"
            "  -L <ip>    listen ip (default from ROCKET_MAV_ARM_REASON_LISTEN_IP)\n"
            "  -P <port>  listen port (default from ROCKET_MAV_ARM_REASON_LISTEN_PORT)\n"
            "  -s <sysid> sender sysid (default ROCKET_MAV_TOOLS_SYSID)\n"
            "  -c <comp>  sender compid (default ROCKET_MAV_TOOLS_COMPID)\n"
            "  -t <sysid> target sysid (default ROCKET_MAV_TOOLS_TARGET_SYS)\n"
            "  -k <comp>  target compid (default ROCKET_MAV_TOOLS_TARGET_COMP)\n"
            "  -f         force arm (param2=21196)\n"
            "  -w <sec>   ACK wait timeout (default %.1f)\n"
            "  -A         show all STATUSTEXT (default: arm-related only)\n"
            "  -h         help\n",
            prog, DEFAULT_TIMEOUT_SEC);
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_ARM_REASON_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_ARM_REASON_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    const char *target_ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, MAV_DEFAULT_IP);
    int target_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, 15651);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    double timeout_sec = DEFAULT_TIMEOUT_SEC;
    int show_all = 0;
    int force = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            target_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-L") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-P") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-f") == 0) {
            force = 1;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            timeout_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-A") == 0) {
            show_all = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (listen_port <= 0) {
        fprintf(stderr, "invalid listen port: %d\n", listen_port);
        return 1;
    }
    if (target_port <= 0) {
        fprintf(stderr, "invalid target port: %d\n", target_port);
        return 1;
    }

    int rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_fd < 0) {
        perror("socket");
        return 1;
    }

    int reuse = 1;
    setsockopt(rx_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)listen_port);
    if (inet_pton(AF_INET, listen_ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid listen ip: %s\n", listen_ip);
        close(rx_fd);
        return 1;
    }
    if (bind(rx_fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "bind %s:%d failed: %s\n", listen_ip, listen_port, strerror(errno));
        close(rx_fd);
        return 1;
    }

    int tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_fd < 0) {
        perror("socket");
        close(rx_fd);
        return 1;
    }

    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons((uint16_t)target_port);
    if (inet_pton(AF_INET, target_ip, &target_addr.sin_addr) != 1) {
        fprintf(stderr, "invalid target ip: %s\n", target_ip);
        close(tx_fd);
        close(rx_fd);
        return 1;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("arm_reason: listen=%s:%d target=%s:%d sys=%d comp=%d tsys=%d tcomp=%d force=%d\n",
           listen_ip, listen_port, target_ip, target_port, sysid, compid, target_sys, target_comp, force);
    printf("filter: %s\n", show_all ? "all" : "arm");

    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));
    statustext_cache_t cache;
    memset(&cache, 0, sizeof(cache));
    cache.sev = 255;

    mavlink_msg_command_long_pack(
        (uint8_t)sysid, (uint8_t)compid, &msg,
        (uint8_t)target_sys, (uint8_t)target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0f,
        force ? 21196.0f : 0.0f,
        0.f, 0.f, 0.f, 0.f, 0.f);
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buf, &msg);
    (void)sendto(tx_fd, tx_buf, tx_len, 0, (const struct sockaddr *)&target_addr, sizeof(target_addr));
    printf("TX ARM -> %s:%d\n", target_ip, target_port);
    int arm_result = wait_for_ack(rx_fd, timeout_sec, show_all, "ARM",
                                  MAV_CMD_COMPONENT_ARM_DISARM, &msg, &status, &cache);
    if (arm_result < 0) {
        printf("ACK ARM timeout %.1fs\n", timeout_sec);
    }

    if ((arm_result < 0 || arm_result != MAV_RESULT_ACCEPTED) && cache.text[0]) {
        double age = monotonic_seconds() - cache.ts;
        if (age < 10.0) {
            printf("LAST_STATUSTEXT sev=%s(%u) age=%.1fs %s\n",
                   severity_str(cache.sev), (unsigned)cache.sev, age, cache.text);
        }
    }

    if (arm_result == MAV_RESULT_ACCEPTED) {
        mavlink_msg_command_long_pack(
            (uint8_t)sysid, (uint8_t)compid, &msg,
            (uint8_t)target_sys, (uint8_t)target_comp,
            MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0.0f,
            0.0f,
            0.f, 0.f, 0.f, 0.f, 0.f);
        tx_len = mavlink_msg_to_send_buffer(tx_buf, &msg);
        (void)sendto(tx_fd, tx_buf, tx_len, 0, (const struct sockaddr *)&target_addr, sizeof(target_addr));
        printf("TX DISARM -> %s:%d\n", target_ip, target_port);

        memset(&status, 0, sizeof(status));
        int disarm_result = wait_for_ack(rx_fd, timeout_sec, show_all, "DISARM",
                                         MAV_CMD_COMPONENT_ARM_DISARM, &msg, &status, &cache);
        if (disarm_result < 0) {
            printf("ACK DISARM timeout %.1fs\n", timeout_sec);
        }
    }

    close(tx_fd);
    close(rx_fd);
    return 0;
}
