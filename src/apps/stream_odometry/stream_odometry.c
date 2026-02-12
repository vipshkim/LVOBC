#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
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
#include "rocket_common.h"
#include "rocket_mav_common.h"

#define DEFAULT_MAV_IP MAV_DEFAULT_IP
#define DEFAULT_MAV_PORT MAV_DEFAULT_PORT
#define DEFAULT_OUT_IP "127.0.0.1"
#define DEFAULT_RATE_HZ 100.0
#define DEFAULT_STATE_PATH "/tmp/stream_odometry.latest"
#define ODOM_REQUEST_RETRY_SEC 1.0
#define ODOM_REQUEST_INTERVAL_US 10000.0f /* 100 Hz */

#define ODOM_FLOAT_COUNT 13
#define ODOM_PACKET_SIZE (ODOM_FLOAT_COUNT * (int)sizeof(float))

static volatile sig_atomic_t g_running = 1;

typedef struct {
    float q[4];
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    uint8_t quality;
    uint64_t time_usec;
    int valid;
} odom_state_t;

static void handle_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void write_f32_le(uint8_t *dst, float value) {
    uint8_t raw[sizeof(float)];
    memcpy(raw, &value, sizeof(raw));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    dst[0] = raw[0];
    dst[1] = raw[1];
    dst[2] = raw[2];
    dst[3] = raw[3];
#else
    dst[0] = raw[3];
    dst[1] = raw[2];
    dst[2] = raw[1];
    dst[3] = raw[0];
#endif
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  -u <ip>        MAVLink input listen IP (default %s)\n"
        "  -p <port>      MAVLink input listen port (default %d)\n"
        "  -o <ip>        UDP output IP (default %s)\n"
        "  -q <port>      UDP output port (default from %s)\n"
        "  -r <hz>        Stream rate in Hz (default %.1f)\n"
        "  -f <path>      /tmp state file path (default %s)\n"
        "  -h             Show this help\n",
        prog,
        DEFAULT_MAV_IP,
        DEFAULT_MAV_PORT,
        DEFAULT_OUT_IP,
        MAV_CFG_KEY_STREAM_ODOM_OUT_PORT,
        DEFAULT_RATE_HZ,
        DEFAULT_STATE_PATH);
}

static int setup_udp_listener(const char *ip, int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        close(fd);
        return -1;
    }
    if (bind(fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int setup_udp_sender(struct sockaddr_in *dest, const char *ip, int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    memset(dest, 0, sizeof(*dest));
    dest->sin_family = AF_INET;
    dest->sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &dest->sin_addr) != 1) {
        close(fd);
        return -1;
    }
    return fd;
}

static void save_state_file(const char *path, const odom_state_t *s) {
    FILE *fp = fopen(path, "w");
    if (!fp) return;

    fprintf(fp, "time_usec=%llu\n", (unsigned long long)s->time_usec);
    fprintf(fp, "q0=%.9g\n", (double)s->q[0]);
    fprintf(fp, "q1=%.9g\n", (double)s->q[1]);
    fprintf(fp, "q2=%.9g\n", (double)s->q[2]);
    fprintf(fp, "q3=%.9g\n", (double)s->q[3]);
    fprintf(fp, "rollspeed=%.9g\n", (double)s->rollspeed);
    fprintf(fp, "pitchspeed=%.9g\n", (double)s->pitchspeed);
    fprintf(fp, "yawspeed=%.9g\n", (double)s->yawspeed);
    fprintf(fp, "x=%.9g\n", (double)s->x);
    fprintf(fp, "y=%.9g\n", (double)s->y);
    fprintf(fp, "z=%.9g\n", (double)s->z);
    fprintf(fp, "vx=%.9g\n", (double)s->vx);
    fprintf(fp, "vy=%.9g\n", (double)s->vy);
    fprintf(fp, "vz=%.9g\n", (double)s->vz);
    fprintf(fp, "quality=%u\n", (unsigned)s->quality);
    fclose(fp);
}

static void fill_odom_packet(const odom_state_t *s, uint8_t *pkt) {
    size_t off = 0;

    const float fields[ODOM_FLOAT_COUNT] = {
        s->q[0], s->q[1], s->q[2], s->q[3],
        s->rollspeed, s->pitchspeed, s->yawspeed,
        s->x, s->y, s->z,
        s->vx, s->vy, s->vz
    };

    for (size_t i = 0; i < ODOM_FLOAT_COUNT; ++i) {
        write_f32_le(pkt + off, fields[i]);
        off += sizeof(float);
    }
}

static void update_from_odometry(odom_state_t *s, const mavlink_odometry_t *o) {
    memcpy(s->q, o->q, sizeof(s->q));
    s->rollspeed = o->rollspeed;
    s->pitchspeed = o->pitchspeed;
    s->yawspeed = o->yawspeed;
    s->x = o->x;
    s->y = o->y;
    s->z = o->z;
    s->vx = o->vx;
    s->vy = o->vy;
    s->vz = o->vz;
    s->time_usec = o->time_usec;

    if (o->quality < 0) {
        s->quality = 0;
    } else {
        s->quality = (uint8_t)o->quality;
    }
    s->valid = 1;
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

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(
        MAV_CFG_KEY_STREAM_ODOM_LISTEN_IP,
        mav_cfg_get_str(MAV_CFG_KEY_SCAN_LISTEN_IP, DEFAULT_MAV_IP));
    int listen_port = mav_cfg_get_int(
        MAV_CFG_KEY_STREAM_ODOM_LISTEN_PORT,
        mav_cfg_get_int(MAV_CFG_KEY_SCAN_LISTEN_PORT, DEFAULT_MAV_PORT));
    const char *out_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_ODOM_OUT_IP, DEFAULT_OUT_IP);
    int out_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_ODOM_OUT_PORT, -1);
    const char *test_out_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_ODOM_TEST_LISTEN_IP, out_ip);
    int test_out_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_ODOM_TEST_LISTEN_PORT, -1);
    const char *req_ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, "127.0.0.1");
    int req_port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, 15651);
    int req_sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int req_compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    int req_target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int req_target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    double rate_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_ODOM_RATE_HZ, DEFAULT_RATE_HZ);
    const char *state_path = mav_cfg_get_str(MAV_CFG_KEY_STREAM_ODOM_STATE_PATH, DEFAULT_STATE_PATH);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out_ip = argv[++i];
        } else if (strcmp(argv[i], "-q") == 0 && i + 1 < argc) {
            out_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            rate_hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            state_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (out_port <= 0) {
        fprintf(stderr, "Missing %s in ports.env\n", MAV_CFG_KEY_STREAM_ODOM_OUT_PORT);
        return 1;
    }

    if (rate_hz <= 0.0 || !isfinite(rate_hz)) {
        fprintf(stderr, "Invalid rate: %f\n", rate_hz);
        return 1;
    }

    char lock_err[160];
    int lock_fd = rocket_single_instance_acquire("stream_odometry", lock_err, sizeof(lock_err));
    if (lock_fd < 0) {
        fprintf(stderr, "stream_odometry: %s\n", lock_err[0] ? lock_err : "single-instance lock failed");
        return 1;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int rx_fd = setup_udp_listener(listen_ip, listen_port);
    if (rx_fd < 0) {
        fprintf(stderr, "Failed to bind MAVLink listener %s:%d: %s\n", listen_ip, listen_port, strerror(errno));
        return 1;
    }

    struct sockaddr_in tx_addr;
    int tx_fd = setup_udp_sender(&tx_addr, out_ip, out_port);
    if (tx_fd < 0) {
        fprintf(stderr, "Failed to setup UDP sender %s:%d\n", out_ip, out_port);
        close(rx_fd);
        return 1;
    }

    int test_tx_enabled = 0;
    int test_tx_fd = -1;
    struct sockaddr_in test_tx_addr;
    if (test_out_port > 0 && !(test_out_port == out_port && strcmp(test_out_ip, out_ip) == 0)) {
        test_tx_fd = setup_udp_sender(&test_tx_addr, test_out_ip, test_out_port);
        if (test_tx_fd < 0) {
            fprintf(stderr, "Failed to setup test UDP sender %s:%d\n", test_out_ip, test_out_port);
            close(tx_fd);
            close(rx_fd);
            return 1;
        }
        test_tx_enabled = 1;
    }

    if (test_tx_enabled) {
        printf("stream_odometry: in=%s:%d out=%s:%d test_out=%s:%d rate=%.2fHz state=%s\n",
               listen_ip, listen_port, out_ip, out_port, test_out_ip, test_out_port, rate_hz, state_path);
    } else {
        printf("stream_odometry: in=%s:%d out=%s:%d rate=%.2fHz state=%s\n",
               listen_ip, listen_port, out_ip, out_port, rate_hz, state_path);
    }

    const double period = 1.0 / rate_hz;
    double next_tx = monotonic_seconds() + period;
    double last_odom_rx = 0.0;
    double last_odom_req = 0.0;

    struct sockaddr_in req_addr;
    int req_fd = setup_udp_sender(&req_addr, req_ip, req_port);
    if (req_fd < 0) {
        fprintf(stderr, "stream_odometry: warning: failed to setup request sender %s:%d\n", req_ip, req_port);
    }

    odom_state_t state;
    memset(&state, 0, sizeof(state));

    mavlink_message_t msg;
    mavlink_status_t mstatus;
    memset(&mstatus, 0, sizeof(mstatus));

    while (g_running) {
        double now = monotonic_seconds();
        double timeout = next_tx - now;
        if (timeout < 0.0) timeout = 0.0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(rx_fd, &rfds);

        struct timeval tv;
        tv.tv_sec = (time_t)timeout;
        tv.tv_usec = (suseconds_t)((timeout - (double)tv.tv_sec) * 1e6);

        int ready = select(rx_fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        if (ready > 0 && FD_ISSET(rx_fd, &rfds)) {
            for (;;) {
                uint8_t buf[2048];
                ssize_t n = recvfrom(rx_fd, buf, sizeof(buf), MSG_DONTWAIT, NULL, NULL);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    if (errno == EINTR) continue;
                    break;
                }

                for (ssize_t i = 0; i < n; ++i) {
                    if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &mstatus)) continue;
                    if (msg.msgid != MAVLINK_MSG_ID_ODOMETRY) continue;

                    mavlink_odometry_t odom;
                    mavlink_msg_odometry_decode(&msg, &odom);
                    update_from_odometry(&state, &odom);
                    last_odom_rx = monotonic_seconds();
                    save_state_file(state_path, &state);
                }
            }
        }

        now = monotonic_seconds();
        if (req_fd >= 0 && (last_odom_rx <= 0.0 || (now - last_odom_rx) >= ODOM_REQUEST_RETRY_SEC)) {
            if (last_odom_req <= 0.0 || (now - last_odom_req) >= ODOM_REQUEST_RETRY_SEC) {
                send_message_interval_request(req_fd, &req_addr,
                                              req_sysid, req_compid,
                                              req_target_sys, req_target_comp,
                                              MAVLINK_MSG_ID_ODOMETRY,
                                              ODOM_REQUEST_INTERVAL_US);
                last_odom_req = now;
            }
        }

        if (now >= next_tx) {
            if (state.valid) {
                uint8_t pkt[ODOM_PACKET_SIZE];
                fill_odom_packet(&state, pkt);
                sendto(tx_fd, pkt, sizeof(pkt), 0, (const struct sockaddr *)&tx_addr, sizeof(tx_addr));
                if (test_tx_enabled) {
                    sendto(test_tx_fd, pkt, sizeof(pkt), 0, (const struct sockaddr *)&test_tx_addr, sizeof(test_tx_addr));
                }
            }

            double periods = floor((now - next_tx) / period) + 1.0;
            if (periods < 1.0) periods = 1.0;
            next_tx += periods * period;
        }
    }

    if (test_tx_fd >= 0) close(test_tx_fd);
    if (req_fd >= 0) close(req_fd);
    close(tx_fd);
    close(rx_fd);
    rocket_single_instance_release(lock_fd);
    return 0;
}
