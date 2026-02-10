#define _DEFAULT_SOURCE 1

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
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

#define DEFAULT_LISTEN_IP "127.0.0.1"
#define DEFAULT_LISTEN_PORT 15500
#define DEFAULT_TARGET_IP MAV_DEFAULT_IP
#define DEFAULT_TARGET_PORT 14651
#define DEFAULT_STATE_PATH "/tmp/stream_commander.latest"
#define DEFAULT_MANUAL_KEEPALIVE_HZ 10.0

#define START_TOKEN ((uint8_t)'$')
#define END_TOKEN ((uint8_t)'\n')

#define MAX_RC_CHANNELS 18
#define MAX_ACTUATOR_CHANNELS 12

#define STATE_MAGIC "RSCM"
#define STATE_VERSION 2
#define STATE_ACT_COUNT 8
#define STATE_MAX_RC_COUNT 16

typedef struct {
    int16_t x, y, z, r;
    uint16_t buttons;
    uint16_t buttons2;
    uint8_t enabled_extensions;
    int16_t s, t;
    int16_t aux1, aux2, aux3, aux4, aux5, aux6;
} manual_state_t;

typedef struct {
    uint8_t arm;
    uint8_t rc_count;
    uint16_t rc[MAX_RC_CHANNELS];
    uint8_t act_count;
    float act[MAX_ACTUATOR_CHANNELS];

    uint8_t manual_level; /* 0..15 */
    manual_state_t manual;
} command_packet_t;

static volatile sig_atomic_t g_running = 1;

static void handle_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static double mono_now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
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

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)read_u16_le(p);
}

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static float read_f32_le(const uint8_t *p) {
    union {
        uint32_t u;
        float f;
    } cvt;
    cvt.u = read_u32_le(p);
    return cvt.f;
}

static void write_u16_le(FILE *fp, uint16_t v) {
    uint8_t b[2];
    b[0] = (uint8_t)(v & 0xFFu);
    b[1] = (uint8_t)((v >> 8) & 0xFFu);
    fwrite(b, 1, sizeof(b), fp);
}

static void write_u32_le(FILE *fp, uint32_t v) {
    uint8_t b[4];
    b[0] = (uint8_t)(v & 0xFFu);
    b[1] = (uint8_t)((v >> 8) & 0xFFu);
    b[2] = (uint8_t)((v >> 16) & 0xFFu);
    b[3] = (uint8_t)((v >> 24) & 0xFFu);
    fwrite(b, 1, sizeof(b), fp);
}

static int parse_command_packet(const uint8_t *buf, size_t len, command_packet_t *out) {
    if (!buf || !out) return 0;
    if (len < 5) return 0;
    if (buf[0] != START_TOKEN || buf[len - 1] != END_TOKEN) return 0;

    memset(out, 0, sizeof(*out));
    out->manual.x = 0;
    out->manual.y = 0;
    out->manual.z = 0;
    out->manual.r = 0;

    size_t off = 1;
    out->arm = buf[off++] ? 1u : 0u;

    if (off >= len) return 0;
    uint8_t rc_count_raw = buf[off++];
    out->rc_count = (rc_count_raw > MAX_RC_CHANNELS) ? MAX_RC_CHANNELS : rc_count_raw;
    for (uint8_t i = 0; i < rc_count_raw; ++i) {
        if (off + 2 > len) return 0;
        uint16_t v = read_u16_le(buf + off);
        if (i < out->rc_count) out->rc[i] = v;
        off += 2;
    }

    if (off >= len) return 0;
    uint8_t act_count_raw = buf[off++];
    out->act_count = (act_count_raw > MAX_ACTUATOR_CHANNELS) ? MAX_ACTUATOR_CHANNELS : act_count_raw;
    for (uint8_t i = 0; i < act_count_raw; ++i) {
        if (off + 4 > len) return 0;
        float v = read_f32_le(buf + off);
        if (i < out->act_count) out->act[i] = v;
        off += 4;
    }

    /* backward compatibility: old format ends here with '\n' */
    if (off + 1 == len && buf[off] == END_TOKEN) {
        out->manual_level = 0;
        return 1;
    }

    if (off >= len) return 0;
    out->manual_level = buf[off++];
    if (out->manual_level > 15) return 0;

    if (out->manual_level >= 1) { if (off + 2 > len) return 0; out->manual.x = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 2) { if (off + 2 > len) return 0; out->manual.y = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 3) { if (off + 2 > len) return 0; out->manual.z = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 4) { if (off + 2 > len) return 0; out->manual.r = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 5) { if (off + 2 > len) return 0; out->manual.buttons = read_u16_le(buf + off); off += 2; }
    if (out->manual_level >= 6) { if (off + 2 > len) return 0; out->manual.buttons2 = read_u16_le(buf + off); off += 2; }
    if (out->manual_level >= 7) { if (off + 1 > len) return 0; out->manual.enabled_extensions = buf[off++]; }
    if (out->manual_level >= 8) { if (off + 2 > len) return 0; out->manual.s = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 9) { if (off + 2 > len) return 0; out->manual.t = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 10){ if (off + 2 > len) return 0; out->manual.aux1 = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 11){ if (off + 2 > len) return 0; out->manual.aux2 = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 12){ if (off + 2 > len) return 0; out->manual.aux3 = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 13){ if (off + 2 > len) return 0; out->manual.aux4 = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 14){ if (off + 2 > len) return 0; out->manual.aux5 = read_i16_le(buf + off); off += 2; }
    if (out->manual_level >= 15){ if (off + 2 > len) return 0; out->manual.aux6 = read_i16_le(buf + off); off += 2; }

    return (off + 1 == len && buf[off] == END_TOKEN);
}

static void send_mavlink_packet(int sock, const struct sockaddr_in *dest, const mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void send_arm_command(int tx_fd, const struct sockaddr_in *dest,
                             int sysid, int compid, int target_sys, int target_comp,
                             uint8_t arm) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm ? 1.0f : 0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f);
    send_mavlink_packet(tx_fd, dest, &msg);
}

static void send_manual_control(int tx_fd, const struct sockaddr_in *dest,
                                int sysid, int compid, int target_sys,
                                const manual_state_t *m) {
    mavlink_message_t msg;
    mavlink_msg_manual_control_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        m->x, m->y, m->z, m->r,
        m->buttons,
        m->buttons2,
        m->enabled_extensions,
        m->s, m->t,
        m->aux1, m->aux2, m->aux3, m->aux4, m->aux5, m->aux6);
    send_mavlink_packet(tx_fd, dest, &msg);
}

static void send_rc_channels_override(int tx_fd, const struct sockaddr_in *dest,
                                      int sysid, int compid, int target_sys, int target_comp,
                                      const command_packet_t *pkt) {
    if (!pkt || pkt->rc_count == 0) return;

    uint16_t raw[18];
    for (int i = 0; i < 18; ++i) raw[i] = UINT16_MAX;
    for (uint8_t i = 0; i < pkt->rc_count && i < 18; ++i) raw[i] = pkt->rc[i];

    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],
        raw[8], raw[9], raw[10], raw[11], raw[12], raw[13], raw[14], raw[15],
        raw[16], raw[17]);
    send_mavlink_packet(tx_fd, dest, &msg);
}

static void send_do_set_actuator_group(int tx_fd, const struct sockaddr_in *dest,
                                       int sysid, int compid, int target_sys, int target_comp,
                                       int set_index,
                                       float p1, float p2, float p3, float p4, float p5, float p6) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        MAV_CMD_DO_SET_ACTUATOR,
        0,
        p1, p2, p3, p4, p5, p6,
        (float)set_index);
    send_mavlink_packet(tx_fd, dest, &msg);
}

static void send_do_set_actuator(int tx_fd, const struct sockaddr_in *dest,
                                 int sysid, int compid, int target_sys, int target_comp,
                                 const command_packet_t *pkt) {
    if (!pkt || pkt->act_count == 0) return;

    float s0[6] = {NAN, NAN, NAN, NAN, NAN, NAN};
    float s1[6] = {NAN, NAN, NAN, NAN, NAN, NAN};

    uint8_t c0 = (pkt->act_count > 6) ? 6 : pkt->act_count;
    for (uint8_t i = 0; i < c0; ++i) s0[i] = pkt->act[i];

    send_do_set_actuator_group(tx_fd, dest, sysid, compid, target_sys, target_comp,
                               0, s0[0], s0[1], s0[2], s0[3], s0[4], s0[5]);

    if (pkt->act_count >= 7) {
        uint8_t c1 = pkt->act_count - 6;
        if (c1 > 6) c1 = 6;
        for (uint8_t i = 0; i < c1; ++i) s1[i] = pkt->act[i + 6];

        send_do_set_actuator_group(tx_fd, dest, sysid, compid, target_sys, target_comp,
                                   1, s1[0], s1[1], s1[2], s1[3], s1[4], s1[5]);
    }
}

static void save_state_file(const char *path, const command_packet_t *pkt) {
    if (!path || !path[0] || !pkt) return;

    FILE *fp = fopen(path, "wb");
    if (!fp) return;

    fwrite(STATE_MAGIC, 1, 4, fp);
    fputc(STATE_VERSION, fp);
    fputc(pkt->arm ? 1 : 0, fp);
    write_u16_le(fp, (uint16_t)STATE_ACT_COUNT);

    for (int i = 0; i < STATE_ACT_COUNT; ++i) {
        union {
            float f;
            uint32_t u;
        } cvt;
        cvt.f = (i < pkt->act_count) ? pkt->act[i] : 0.0f;
        write_u32_le(fp, cvt.u);
    }

    uint8_t rc_n = pkt->rc_count;
    if (rc_n > STATE_MAX_RC_COUNT) rc_n = STATE_MAX_RC_COUNT;
    fputc(rc_n, fp);
    for (uint8_t i = 0; i < rc_n; ++i) {
        union {
            float f;
            uint32_t u;
        } cvt;
        cvt.f = (float)pkt->rc[i];
        write_u32_le(fp, cvt.u);
    }

    fclose(fp);
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Options:\n"
            "  -l <ip>    listen ip (default %s)\n"
            "  -q <port>  listen port (default %d)\n"
            "  -u <ip>    target ip (default %s)\n"
            "  -p <port>  target port (default %d)\n"
            "  -s <id>    sender sysid\n"
            "  -c <id>    sender compid\n"
            "  -t <id>    target sysid\n"
            "  -k <id>    target compid\n"
            "  -f <path>  state file path (default %s)\n"
            "  -h         help\n",
            prog,
            DEFAULT_LISTEN_IP, DEFAULT_LISTEN_PORT,
            DEFAULT_TARGET_IP, DEFAULT_TARGET_PORT,
            DEFAULT_STATE_PATH);
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    const char *target_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_TARGET_IP, DEFAULT_TARGET_IP);
    int target_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_TARGET_PORT, DEFAULT_TARGET_PORT);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_COMPID, 192);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    const char *state_path = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_STATE_PATH, DEFAULT_STATE_PATH);
    double manual_keepalive_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_MANUAL_KEEPALIVE_HZ, DEFAULT_MANUAL_KEEPALIVE_HZ);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) listen_ip = argv[++i];
        else if (strcmp(argv[i], "-q") == 0 && i + 1 < argc) listen_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) target_ip = argv[++i];
        else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) target_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) sysid = atoi(argv[++i]);
        else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) compid = atoi(argv[++i]);
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) target_sys = atoi(argv[++i]);
        else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) target_comp = atoi(argv[++i]);
        else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) state_path = argv[++i];
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (manual_keepalive_hz <= 0.0) manual_keepalive_hz = DEFAULT_MANUAL_KEEPALIVE_HZ;
    double manual_period = 1.0 / manual_keepalive_hz;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int rx_fd = setup_udp_listener(listen_ip, listen_port);
    if (rx_fd < 0) {
        fprintf(stderr, "failed to bind %s:%d: %s\n", listen_ip, listen_port, strerror(errno));
        return 1;
    }

    struct sockaddr_in target_addr;
    int tx_fd = setup_udp_sender(&target_addr, target_ip, target_port);
    if (tx_fd < 0) {
        fprintf(stderr, "failed to setup sender to %s:%d: %s\n", target_ip, target_port, strerror(errno));
        close(rx_fd);
        return 1;
    }

    printf("stream_commander: listen=%s:%d -> target=%s:%d sys=%d comp=%d tsys=%d tcomp=%d manual_keepalive=%.2fHz\n",
           listen_ip, listen_port, target_ip, target_port, sysid, compid, target_sys, target_comp, manual_keepalive_hz);

    int last_arm = -1;
    unsigned long invalid_count = 0;
    manual_state_t manual_state;
    memset(&manual_state, 0, sizeof(manual_state));
    double next_manual_tx = mono_now_sec() + manual_period;

    while (g_running) {
        double now = mono_now_sec();
        double wait_sec = next_manual_tx - now;
        if (wait_sec < 0.0) wait_sec = 0.0;
        if (wait_sec > 1.0) wait_sec = 1.0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(rx_fd, &rfds);

        struct timeval tv;
        tv.tv_sec = (int)wait_sec;
        tv.tv_usec = (suseconds_t)((wait_sec - (double)tv.tv_sec) * 1e6);

        int r = select(rx_fd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        if (r > 0 && FD_ISSET(rx_fd, &rfds)) {
            uint8_t buf[1024];
            ssize_t n = recvfrom(rx_fd, buf, sizeof(buf), 0, NULL, NULL);
            if (n >= 0) {
                command_packet_t pkt;
                if (!parse_command_packet(buf, (size_t)n, &pkt)) {
                    invalid_count++;
                    if ((invalid_count % 50UL) == 1UL) {
                        fprintf(stderr, "stream_commander: ignored invalid packet len=%zd (count=%lu)\n", n, invalid_count);
                    }
                } else {
                    if (last_arm < 0 || pkt.arm != (uint8_t)last_arm) {
                        send_arm_command(tx_fd, &target_addr, sysid, compid, target_sys, target_comp, pkt.arm);
                        last_arm = pkt.arm;
                    }

                    if (pkt.rc_count > 0) {
                        send_rc_channels_override(tx_fd, &target_addr, sysid, compid, target_sys, target_comp, &pkt);
                    }
                    if (pkt.act_count > 0) {
                        send_do_set_actuator(tx_fd, &target_addr, sysid, compid, target_sys, target_comp, &pkt);
                    }

                    if (pkt.manual_level >= 1) manual_state.x = pkt.manual.x;
                    if (pkt.manual_level >= 2) manual_state.y = pkt.manual.y;
                    if (pkt.manual_level >= 3) manual_state.z = pkt.manual.z;
                    if (pkt.manual_level >= 4) manual_state.r = pkt.manual.r;
                    if (pkt.manual_level >= 5) manual_state.buttons = pkt.manual.buttons;
                    if (pkt.manual_level >= 6) manual_state.buttons2 = pkt.manual.buttons2;
                    if (pkt.manual_level >= 7) manual_state.enabled_extensions = pkt.manual.enabled_extensions;
                    if (pkt.manual_level >= 8) manual_state.s = pkt.manual.s;
                    if (pkt.manual_level >= 9) manual_state.t = pkt.manual.t;
                    if (pkt.manual_level >= 10) manual_state.aux1 = pkt.manual.aux1;
                    if (pkt.manual_level >= 11) manual_state.aux2 = pkt.manual.aux2;
                    if (pkt.manual_level >= 12) manual_state.aux3 = pkt.manual.aux3;
                    if (pkt.manual_level >= 13) manual_state.aux4 = pkt.manual.aux4;
                    if (pkt.manual_level >= 14) manual_state.aux5 = pkt.manual.aux5;
                    if (pkt.manual_level >= 15) manual_state.aux6 = pkt.manual.aux6;

                    save_state_file(state_path, &pkt);
                }
            }
        }

        now = mono_now_sec();
        if (now >= next_manual_tx) {
            send_manual_control(tx_fd, &target_addr, sysid, compid, target_sys, &manual_state);
            next_manual_tx += manual_period;
            if (now - next_manual_tx > 1.0) {
                next_manual_tx = now + manual_period;
            }
        }
    }

    close(tx_fd);
    close(rx_fd);
    return 0;
}
