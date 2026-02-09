#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
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
#include "rocket_mav_common.h"

#define DEFAULT_LISTEN_IP "127.0.0.1"
#define DEFAULT_LISTEN_PORT 14654
#define DEFAULT_MAV_IP MAV_DEFAULT_IP
#define DEFAULT_MAV_PORT MAV_DEFAULT_PORT
#define DEFAULT_RATE_HZ 100.0
#define DEFAULT_IDLE_RATE_HZ 20.0
#define DEFAULT_ACTIVE_RATE_HZ 200.0
#define DEFAULT_ACTIVE_HOLD_SEC 0.5
#define DEFAULT_MANUAL_KEEPALIVE_HZ 5.0
#define DEFAULT_RX_BURST_MAX 32
#define DEFAULT_RX_PARSE_HZ 200.0
#define DEFAULT_INPUT_TIMEOUT_SEC 1.0
#define DEFAULT_STATE_PATH "/tmp/stream_commander.latest"

#define START_TOKEN ((uint8_t)'$')
#define END_TOKEN ((uint8_t)'\n')
#define COMMAND_FLOAT_COUNT 8
#define COMMAND_PACKET_SIZE (1 + 1 + COMMAND_FLOAT_COUNT * (int)sizeof(float) + 1)
#define STATE_MAGIC "RSCM"
#define STATE_VERSION 1
#define PX4_CUSTOM_MAIN_MODE_MANUAL 1

static volatile sig_atomic_t g_running = 1;

typedef struct {
    uint8_t arm;
    float actuator[COMMAND_FLOAT_COUNT];
    int valid;
} command_state_t;

typedef struct {
    int valid;
    uint8_t base_mode;
    uint8_t main_mode;
    uint8_t sub_mode;
} flight_mode_snapshot_t;

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
        "  -l <ip>        Command input listen IP (default %s)\n"
        "  -q <port>      Command input listen port (default %d)\n"
        "  -u <ip>        MAVLink target IP (default %s)\n"
        "  -p <port>      MAVLink target port (default %d)\n"
        "  -r <hz>        Control output rate in Hz (default %.1f)\n"
        "  -i <hz>        Idle actuator TX rate in Hz (default %.1f)\n"
        "  -a <hz>        Active actuator TX rate cap in Hz (default %.1f)\n"
        "  -b <sec>       Active boost hold time after cmd packet (default %.1f)\n"
        "  -m <hz>        MANUAL_CONTROL keepalive rate in Hz (default %.1f)\n"
        "  -x <count>     Max RX packets handled per loop (default %d)\n"
        "  -y <hz>        Max RX parse cadence in Hz (default %.1f, max 200)\n"
        "  -s <sysid>     Sender system id (default %d)\n"
        "  -c <compid>    Sender component id (default %d)\n"
        "  -t <target>    Target system id (default %d)\n"
        "  -k <target>    Target component id (default %d)\n"
        "  -f <path>      /tmp state file path (default %s)\n"
        "  -h             Show this help\n",
        prog,
        DEFAULT_LISTEN_IP,
        DEFAULT_LISTEN_PORT,
        DEFAULT_MAV_IP,
        DEFAULT_MAV_PORT,
        DEFAULT_RATE_HZ,
        DEFAULT_IDLE_RATE_HZ,
        DEFAULT_ACTIVE_RATE_HZ,
        DEFAULT_ACTIVE_HOLD_SEC,
        DEFAULT_MANUAL_KEEPALIVE_HZ,
        DEFAULT_RX_BURST_MAX,
        DEFAULT_RX_PARSE_HZ,
        MAV_DEFAULT_SYSID,
        MAV_DEFAULT_COMPID,
        MAV_DEFAULT_TARGET_SYS,
        MAV_DEFAULT_TARGET_COMP,
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

static float clamp_unit(float v) {
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static float clamp_range(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float normalize_input_value(size_t idx, float raw, int use_legacy_01000) {
    if (!use_legacy_01000) {
        return clamp_unit(raw);
    }
    if (idx < 4) {
        // motor 1..4: legacy 0..1000 -> 0..1
        return clamp_range(raw / 1000.0f, 0.0f, 1.0f);
    }
    // servo 5..8: legacy 0..1000 (500 neutral) -> -1..1
    return clamp_unit((raw - 500.0f) / 500.0f);
}

static void send_arm_disarm(int sock, const struct sockaddr_in *dest,
                            int sysid, int compid, int target_sys, int target_comp,
                            int arm, int force) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm ? 1.0f : 0.0f,
        force ? 21196.0f : 0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void custom_mode_to_main_sub(uint32_t custom_mode, uint8_t *main_mode, uint8_t *sub_mode) {
    if (main_mode) *main_mode = (uint8_t)((custom_mode >> 16) & 0xFFu);
    if (sub_mode) *sub_mode = (uint8_t)((custom_mode >> 24) & 0xFFu);
}

static void send_set_mode(int sock, const struct sockaddr_in *dest,
                          int sysid, int compid, int target_sys, int target_comp,
                          uint8_t base_mode, uint8_t main_mode, uint8_t sub_mode) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t use_base = (uint8_t)(base_mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);

    mavlink_msg_command_long_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        MAV_CMD_DO_SET_MODE,
        0,
        (float)use_base,
        (float)main_mode,
        (float)sub_mode,
        0.f, 0.f, 0.f, 0.f);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void send_manual_control_keepalive(int sock, const struct sockaddr_in *dest,
                                          int sysid, int compid, int target_sys) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Keep throttle at minimum for arming safety gate (PX4 rejects high throttle).
    mavlink_msg_manual_control_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        0, 0, 0, 0,
        0,    // buttons
        0,    // buttons2
        0,    // enabled_extensions
        0, 0, 0, 0, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void send_prearm_low_throttle(int sock, const struct sockaddr_in *dest,
                                     int sysid, int compid, int target_sys) {
    /* Push a short burst of low-throttle manual control before ARM command. */
    for (int i = 0; i < 5; ++i) {
        send_manual_control_keepalive(sock, dest, sysid, compid, target_sys);
        usleep(20000);
    }
}

static void request_message(int sock, const struct sockaddr_in *dest,
                            int sysid, int compid, int target_sys, int target_comp,
                            uint32_t message_id) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(
        (uint8_t)sysid,
        (uint8_t)compid,
        &msg,
        (uint8_t)target_sys,
        (uint8_t)target_comp,
        MAV_CMD_REQUEST_MESSAGE,
        0,
        (float)message_id,
        0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void process_rx_message(const mavlink_message_t *msg,
                               int target_sys, int target_comp,
                               flight_mode_snapshot_t *mode_hint) {
    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        if (msg->sysid != (uint8_t)target_sys) return;
        if (target_comp > 0 && msg->compid != (uint8_t)target_comp) return;
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        if (mode_hint) {
            mode_hint->valid = 1;
            mode_hint->base_mode = hb.base_mode;
            custom_mode_to_main_sub(hb.custom_mode, &mode_hint->main_mode, &mode_hint->sub_mode);
        }
    }
}

static int wait_for_target_heartbeat(int fd, int target_sys, int target_comp,
                                     double timeout_sec, flight_mode_snapshot_t *out_mode) {
    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));
    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (time_t)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);
        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        uint8_t buf[2048];
        ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;
            process_rx_message(&msg, target_sys, target_comp, out_mode);
            if (out_mode && out_mode->valid) return 1;
        }
    }
    return 0;
}

static int wait_for_command_ack(int fd, uint16_t command, double timeout_sec, uint8_t *out_result) {
    double deadline = monotonic_seconds() + timeout_sec;
    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));

    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (time_t)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);
        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        uint8_t buf[2048];
        ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;
            if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
                continue;
            }
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            if (ack.command != command) continue;
            if (out_result) *out_result = ack.result;
            return 1;
        }
    }

    return 0;
}

static void send_set_actuator_group(int sock, const struct sockaddr_in *dest,
                                    int sysid, int compid, int target_sys, int target_comp,
                                    int set_index,
                                    float p1, float p2, float p3, float p4, float p5, float p6) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

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

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void send_actuators(int sock, const struct sockaddr_in *dest,
                           int sysid, int compid, int target_sys, int target_comp,
                           const command_state_t *st) {
    // Set 0 controls actuator 1..6, set 1 controls actuator 7..12.
    send_set_actuator_group(sock, dest, sysid, compid, target_sys, target_comp,
                            0,
                            clamp_unit(st->actuator[0]),
                            clamp_unit(st->actuator[1]),
                            clamp_unit(st->actuator[2]),
                            clamp_unit(st->actuator[3]),
                            clamp_unit(st->actuator[4]),
                            clamp_unit(st->actuator[5]));

    send_set_actuator_group(sock, dest, sysid, compid, target_sys, target_comp,
                            1,
                            clamp_unit(st->actuator[6]),
                            clamp_unit(st->actuator[7]),
                            NAN, NAN, NAN, NAN);
}

static int parse_command_packet(const uint8_t *buf, size_t n, command_state_t *st) {
    if (n != COMMAND_PACKET_SIZE) return 0;
    if (buf[0] != START_TOKEN) return 0;
    if (buf[COMMAND_PACKET_SIZE - 1] != END_TOKEN) return 0;

    st->arm = buf[1] ? 1 : 0;

    float raw_vals[COMMAND_FLOAT_COUNT];
    int use_legacy_01000 = 0;
    size_t off = 2;
    for (size_t i = 0; i < COMMAND_FLOAT_COUNT; ++i) {
        float v = 0.0f;
        memcpy(&v, buf + off, sizeof(float));
        raw_vals[i] = v;
        if (fabsf(v) > 1.2f) {
            use_legacy_01000 = 1;
        }
        off += sizeof(float);
    }
    for (size_t i = 0; i < COMMAND_FLOAT_COUNT; ++i) {
        st->actuator[i] = normalize_input_value(i, raw_vals[i], use_legacy_01000);
    }

    st->valid = 1;
    return 1;
}

static void put_le16(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_le32(uint8_t *dst, uint32_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void save_state_file(const char *path, const command_state_t *st) {
    if (!path || !path[0] || !st) return;

    char tmp_path[PATH_MAX];
    int n = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    if (n <= 0 || (size_t)n >= sizeof(tmp_path)) return;

    uint8_t blob[4 + 1 + 1 + 2 + COMMAND_FLOAT_COUNT * 4];
    size_t off = 0;
    memcpy(blob + off, STATE_MAGIC, 4);
    off += 4;
    blob[off++] = STATE_VERSION;
    blob[off++] = st->arm ? 1u : 0u;
    put_le16(blob + off, (uint16_t)COMMAND_FLOAT_COUNT);
    off += 2;
    for (size_t i = 0; i < COMMAND_FLOAT_COUNT; ++i) {
        union {
            float f;
            uint32_t u;
        } cvt;
        cvt.f = st->actuator[i];
        put_le32(blob + off, cvt.u);
        off += 4;
    }

    FILE *fp = fopen(tmp_path, "wb");
    if (!fp) return;

    if (fwrite(blob, 1, off, fp) != off) {
        fclose(fp);
        unlink(tmp_path);
        return;
    }
    if (fflush(fp) != 0) {
        fclose(fp);
        unlink(tmp_path);
        return;
    }
    int fd = fileno(fp);
    if (fd >= 0) {
        (void)fsync(fd);
    }
    if (fclose(fp) != 0) {
        unlink(tmp_path);
        return;
    }
    if (rename(tmp_path, path) != 0) {
        unlink(tmp_path);
    }
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    const char *mav_ip = mav_cfg_get_str(
        MAV_CFG_KEY_STREAM_CMD_TARGET_IP,
        mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, DEFAULT_MAV_IP));
    int mav_port = mav_cfg_get_int(
        MAV_CFG_KEY_STREAM_CMD_TARGET_PORT,
        mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, DEFAULT_MAV_PORT));
    double configured_rate_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_RATE_HZ, DEFAULT_RATE_HZ);
    const char *idle_rate_cfg = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_IDLE_RATE_HZ, NULL);
    int env_has_idle_rate = (idle_rate_cfg && idle_rate_cfg[0] != '\0');
    double idle_rate_hz = env_has_idle_rate
        ? mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_IDLE_RATE_HZ, configured_rate_hz)
        : configured_rate_hz;
    double active_rate_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_ACTIVE_RATE_HZ, DEFAULT_ACTIVE_RATE_HZ);
    double active_hold_sec = mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_ACTIVE_HOLD_SEC, DEFAULT_ACTIVE_HOLD_SEC);
    double manual_keepalive_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_MANUAL_KEEPALIVE_HZ, DEFAULT_MANUAL_KEEPALIVE_HZ);
    int rx_burst_max = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_RX_BURST_MAX, DEFAULT_RX_BURST_MAX);
    double rx_parse_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_CMD_RX_PARSE_HZ, DEFAULT_RX_PARSE_HZ);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_COMPID,
                                 mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID));
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);
    const char *state_path = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_STATE_PATH, DEFAULT_STATE_PATH);

    int cli_set_idle_rate = 0;
    int cli_set_rate = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-q") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            mav_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            mav_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            configured_rate_hz = atof(argv[++i]);
            cli_set_rate = 1;
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            idle_rate_hz = atof(argv[++i]);
            cli_set_idle_rate = 1;
        } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            active_rate_hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            active_hold_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            manual_keepalive_hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-x") == 0 && i + 1 < argc) {
            rx_burst_max = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-y") == 0 && i + 1 < argc) {
            rx_parse_hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
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

    if (cli_set_rate && !cli_set_idle_rate && !env_has_idle_rate) {
        idle_rate_hz = configured_rate_hz;
    }

    if (configured_rate_hz <= 0.0 || !isfinite(configured_rate_hz)) {
        fprintf(stderr, "Invalid -r rate: %f\n", configured_rate_hz);
        return 1;
    }
    if (idle_rate_hz <= 0.0 || !isfinite(idle_rate_hz)) {
        fprintf(stderr, "Invalid idle rate: %f\n", idle_rate_hz);
        return 1;
    }
    if (active_rate_hz <= 0.0 || !isfinite(active_rate_hz)) {
        fprintf(stderr, "Invalid active rate: %f\n", active_rate_hz);
        return 1;
    }
    if (manual_keepalive_hz <= 0.0 || !isfinite(manual_keepalive_hz)) {
        fprintf(stderr, "Invalid manual keepalive rate: %f\n", manual_keepalive_hz);
        return 1;
    }
    if (active_hold_sec < 0.0 || !isfinite(active_hold_sec)) {
        fprintf(stderr, "Invalid active hold sec: %f\n", active_hold_sec);
        return 1;
    }
    if (rx_burst_max < 1) rx_burst_max = 1;
    if (rx_burst_max > 1024) rx_burst_max = 1024;
    if (rx_parse_hz <= 0.0 || !isfinite(rx_parse_hz)) {
        fprintf(stderr, "Invalid RX parse rate: %f\n", rx_parse_hz);
        return 1;
    }
    if (rx_parse_hz > 200.0) rx_parse_hz = 200.0;

    // Keep saturation guard conservative by design.
    if (active_rate_hz > 200.0) active_rate_hz = 200.0;
    if (configured_rate_hz > active_rate_hz) configured_rate_hz = active_rate_hz;
    if (idle_rate_hz > active_rate_hz) idle_rate_hz = active_rate_hz;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int rx_fd = setup_udp_listener(listen_ip, listen_port);
    if (rx_fd < 0) {
        fprintf(stderr, "Failed to bind command listener %s:%d: %s\n", listen_ip, listen_port, strerror(errno));
        return 1;
    }

    struct sockaddr_in mav_addr;
    int tx_fd = setup_udp_sender(&mav_addr, mav_ip, mav_port);
    if (tx_fd < 0) {
        fprintf(stderr, "Failed to setup MAVLink sender %s:%d\n", mav_ip, mav_port);
        close(rx_fd);
        return 1;
    }

    printf("stream_commander: in=%s:%d out=%s:%d idle=%.2fHz active<=%.2fHz keepalive=%.2fHz hold=%.2fs timeout=%.2fs rx_burst=%d rx_parse=%.2fHz tsys=%d tcomp=%d\n",
           listen_ip, listen_port, mav_ip, mav_port,
           idle_rate_hz, active_rate_hz, manual_keepalive_hz, active_hold_sec, DEFAULT_INPUT_TIMEOUT_SEC,
           rx_burst_max, rx_parse_hz, target_sys, target_comp);

    double next_tx = monotonic_seconds() + (1.0 / idle_rate_hz);
    double next_manual_keepalive = monotonic_seconds() + (1.0 / manual_keepalive_hz);
    double next_rx = monotonic_seconds() + (1.0 / rx_parse_hz);
    double active_until = 0.0;
    double adaptive_rate_hz = idle_rate_hz;
    double last_cmd_rx_time = -1.0;
    int have_rx_period = 0;

    command_state_t state;
    memset(&state, 0, sizeof(state));
    flight_mode_snapshot_t original_mode;
    memset(&original_mode, 0, sizeof(original_mode));
    int mode_switch_attempted = 0;
    int mode_switched_to_manual = 0;

    int have_arm_state = 0;
    uint8_t armed = 0;

    request_message(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp, MAVLINK_MSG_ID_HEARTBEAT);
    (void)wait_for_target_heartbeat(tx_fd, target_sys, target_comp, 1.5, &original_mode);

    while (g_running) {
        double now = monotonic_seconds();
        double next_event = next_manual_keepalive;
        if (next_tx < next_event) next_event = next_tx;
        if (next_rx < next_event) next_event = next_rx;
        double timeout = next_event - now;
        if (timeout < 0.0) timeout = 0.0;

        fd_set rfds;
        FD_ZERO(&rfds);
        int poll_rx = (now >= next_rx);
        if (poll_rx) {
            FD_SET(rx_fd, &rfds);
        }

        struct timeval tv;
        tv.tv_sec = (time_t)timeout;
        tv.tv_usec = (suseconds_t)((timeout - (double)tv.tv_sec) * 1e6);

        int ready = select(poll_rx ? (rx_fd + 1) : 0, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        if (poll_rx && ready > 0 && FD_ISSET(rx_fd, &rfds)) {
            int rx_processed = 0;
            for (;;) {
                uint8_t buf[512];
                ssize_t n = recvfrom(rx_fd, buf, sizeof(buf), MSG_DONTWAIT, NULL, NULL);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    if (errno == EINTR) continue;
                    break;
                }

                command_state_t incoming;
                memset(&incoming, 0, sizeof(incoming));
                if (!parse_command_packet(buf, (size_t)n, &incoming)) {
                    continue;
                }

                state = incoming;
                save_state_file(state_path, &state);
                double rx_now = monotonic_seconds();
                active_until = rx_now + active_hold_sec;
                if (last_cmd_rx_time > 0.0) {
                    const double dt = rx_now - last_cmd_rx_time;
                    if (dt > 1e-6) {
                        double candidate_rate_hz = 1.0 / dt;
                        if (candidate_rate_hz < idle_rate_hz) candidate_rate_hz = idle_rate_hz;
                        if (candidate_rate_hz > active_rate_hz) candidate_rate_hz = active_rate_hz;
                        if (!have_rx_period) {
                            adaptive_rate_hz = candidate_rate_hz;
                            have_rx_period = 1;
                        } else {
                            // Smooth jitter while still following source cadence quickly.
                            adaptive_rate_hz = adaptive_rate_hz * 0.70 + candidate_rate_hz * 0.30;
                        }
                    }
                } else {
                    adaptive_rate_hz = idle_rate_hz;
                }
                last_cmd_rx_time = rx_now;
                if (next_tx > rx_now) next_tx = rx_now;

                if (!mode_switch_attempted && state.valid) {
                    mode_switch_attempted = 1;
                    if (!original_mode.valid) {
                        request_message(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp, MAVLINK_MSG_ID_HEARTBEAT);
                        (void)wait_for_target_heartbeat(tx_fd, target_sys, target_comp, 1.0, &original_mode);
                    }
                    if (original_mode.valid && original_mode.main_mode != PX4_CUSTOM_MAIN_MODE_MANUAL) {
                        send_set_mode(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp,
                                      original_mode.base_mode, PX4_CUSTOM_MAIN_MODE_MANUAL, 0);
                        uint8_t ack_result = MAV_RESULT_FAILED;
                        if (wait_for_command_ack(tx_fd, MAV_CMD_DO_SET_MODE, 1.5, &ack_result)
                            && ack_result == MAV_RESULT_ACCEPTED) {
                            mode_switched_to_manual = 1;
                            printf("mode switched -> MANUAL\n");
                        } else {
                            printf("mode switch -> MANUAL failed/timeout\n");
                        }
                    }
                }

                if (!have_arm_state || state.arm != armed) {
                    int force_arm = (mav_read_mode_state() == MAV_MODE_STATE_TEST);
                    if (state.arm && !force_arm) {
                        printf("safe mode: mode!=TEST, force-arm disabled (normal arm only)\n");
                    }
                    if (state.arm) {
                        send_prearm_low_throttle(tx_fd, &mav_addr, sysid, compid, target_sys);
                    }
                    send_arm_disarm(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp,
                                    state.arm != 0, force_arm);
                    armed = state.arm;
                    have_arm_state = 1;
                    printf("arm state changed -> %s (%s)\n",
                           armed ? "ARM" : "DISARM",
                           force_arm ? "force" : "normal");
                }

                rx_processed++;
                if (rx_processed >= rx_burst_max) {
                    break;
                }
            }
        }

        now = monotonic_seconds();
        if (now >= next_rx) {
            const double rx_period = 1.0 / rx_parse_hz;
            double periods = floor((now - next_rx) / rx_period) + 1.0;
            if (periods < 1.0) periods = 1.0;
            next_rx += periods * rx_period;
        }

        now = monotonic_seconds();
        if (now >= next_manual_keepalive) {
            send_manual_control_keepalive(tx_fd, &mav_addr, sysid, compid, target_sys);
            double keepalive_period = 1.0 / manual_keepalive_hz;
            double periods = floor((now - next_manual_keepalive) / keepalive_period) + 1.0;
            if (periods < 1.0) periods = 1.0;
            next_manual_keepalive += periods * keepalive_period;
        }

        if (now >= next_tx) {
            int stream_active =
                (state.valid &&
                 last_cmd_rx_time > 0.0 &&
                 (now - last_cmd_rx_time) <= DEFAULT_INPUT_TIMEOUT_SEC);

            if (stream_active) {
                send_actuators(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp, &state);
            } else {
                // Input timeout fallback: keep FC in neutral manual-stick state.
                send_manual_control_keepalive(tx_fd, &mav_addr, sysid, compid, target_sys);
            }
            double current_rate_hz = idle_rate_hz;
            if (stream_active && now < active_until) {
                current_rate_hz = adaptive_rate_hz;
                if (current_rate_hz < idle_rate_hz) current_rate_hz = idle_rate_hz;
                if (current_rate_hz > active_rate_hz) current_rate_hz = active_rate_hz;
            }
            const double current_period = 1.0 / current_rate_hz;
            double periods = floor((now - next_tx) / current_period) + 1.0;
            if (periods < 1.0) periods = 1.0;
            next_tx += periods * current_period;
        }
    }

    if (have_arm_state && armed) {
        int force_arm = (mav_read_mode_state() == MAV_MODE_STATE_TEST);
        send_arm_disarm(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp, 0, force_arm);
    }
    if (mode_switched_to_manual && original_mode.valid) {
        send_set_mode(tx_fd, &mav_addr, sysid, compid, target_sys, target_comp,
                      original_mode.base_mode, original_mode.main_mode, original_mode.sub_mode);
        printf("mode restored -> main=%u sub=%u\n",
               (unsigned)original_mode.main_mode, (unsigned)original_mode.sub_mode);
    }

    close(tx_fd);
    close(rx_fd);
    return 0;
}
