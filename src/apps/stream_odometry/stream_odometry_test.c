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

#include "rocket_mav_common.h"

#define DEFAULT_LISTEN_IP "127.0.0.1"
#define DEFAULT_LISTEN_PORT 14622
#define ODOM_FLOAT_COUNT 13
#define ODOM_PACKET_SIZE (ODOM_FLOAT_COUNT * (int)sizeof(float))
#define DEFAULT_CONSOLE_BYTES_PER_SEC 5760.0
#define DEFAULT_PRINT_HZ 20.0
#define COMMAND_FLOAT_COUNT 8
#define STREAM_STATE_MAGIC "RSCM"
#define STREAM_STATE_VERSION 2
#define MAX_RC_COUNT 16
#define DEFAULT_STREAM_CMD_STATE_PATH "/tmp/stream_commander.latest"

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

static float read_f32_le(const uint8_t *src) {
    uint8_t raw[sizeof(float)];
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    raw[0] = src[0];
    raw[1] = src[1];
    raw[2] = src[2];
    raw[3] = src[3];
#else
    raw[0] = src[3];
    raw[1] = src[2];
    raw[2] = src[1];
    raw[3] = src[0];
#endif
    float out = 0.0f;
    memcpy(&out, raw, sizeof(out));
    return out;
}

static int parse_packet(const uint8_t *buf, size_t n,
                        float q[4], float *rollspeed, float *pitchspeed, float *yawspeed,
                        float *x, float *y, float *z,
                        float *vx, float *vy, float *vz) {
    if (n != ODOM_PACKET_SIZE) return 0;

    size_t off = 0;
    float fields[ODOM_FLOAT_COUNT];
    for (size_t i = 0; i < ODOM_FLOAT_COUNT; ++i) {
        fields[i] = read_f32_le(buf + off);
        off += sizeof(float);
    }

    q[0] = fields[0];
    q[1] = fields[1];
    q[2] = fields[2];
    q[3] = fields[3];
    *rollspeed = fields[4];
    *pitchspeed = fields[5];
    *yawspeed = fields[6];
    *x = fields[7];
    *y = fields[8];
    *z = fields[9];
    *vx = fields[10];
    *vy = fields[11];
    *vz = fields[12];
    return 1;
}

static int render_line_with_rate_limit(const char *line, size_t len,
                                       double *tokens, double max_tokens,
                                       double refill_bytes_per_sec, double *last_refill) {
    double now = monotonic_seconds();
    double dt = now - *last_refill;
    if (dt > 0.0) {
        *tokens += dt * refill_bytes_per_sec;
        if (*tokens > max_tokens) *tokens = max_tokens;
        *last_refill = now;
    }

    if (*tokens < (double)len) {
        return 0;
    }

    *tokens -= (double)len;
    fwrite(line, 1, len, stdout);
    fflush(stdout);
    return 1;
}

static int read_u16_le(const uint8_t *p) {
    return (int)p[0] | ((int)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int load_stream_command_state(const char *path, uint8_t *arm,
                                     float vals[COMMAND_FLOAT_COUNT],
                                     uint8_t *rc_count, float rc_vals[MAX_RC_COUNT]) {
    if (!path || !path[0]) return 0;
    FILE *fp = fopen(path, "rb");
    if (!fp) return 0;

    uint8_t header[4 + 1 + 1 + 2];
    if (fread(header, 1, sizeof(header), fp) != sizeof(header)) {
        fclose(fp);
        return 0;
    }
    if (memcmp(header, STREAM_STATE_MAGIC, 4) != 0) {
        fclose(fp);
        return 0;
    }
    uint8_t version = header[4];
    if (version != 1 && version != STREAM_STATE_VERSION) {
        fclose(fp);
        return 0;
    }
    if (arm) *arm = header[5] ? 1u : 0u;
    int count = read_u16_le(header + 6);
    if (count <= 0 || count > COMMAND_FLOAT_COUNT) {
        fclose(fp);
        return 0;
    }

    for (int i = 0; i < count; ++i) {
        uint8_t b[4];
        if (fread(b, 1, sizeof(b), fp) != sizeof(b)) {
            fclose(fp);
            return 0;
        }
        union { uint32_t u; float f; } cvt;
        cvt.u = read_u32_le(b);
        vals[i] = cvt.f;
    }
    if (rc_count) *rc_count = 0;
    if (rc_vals) {
        for (int i = 0; i < MAX_RC_COUNT; ++i) rc_vals[i] = 0.0f;
    }
    if (version == 1) {
        fclose(fp);
        return 1;
    }

    uint8_t rc_n = 0;
    if (fread(&rc_n, 1, 1, fp) != 1) {
        fclose(fp);
        return 1;
    }
    if (rc_n > MAX_RC_COUNT) rc_n = MAX_RC_COUNT;
    if (rc_count) *rc_count = rc_n;
    for (int i = 0; i < rc_n; ++i) {
        uint8_t b[4];
        if (fread(b, 1, sizeof(b), fp) != sizeof(b)) break;
        union { uint32_t u; float f; } cvt;
        cvt.u = read_u32_le(b);
        if (rc_vals) rc_vals[i] = cvt.f;
    }
    fclose(fp);
    return 1;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Options:\n"
            "  -l <ip>    listen ip (default %s)\n"
            "  -p <port>  listen port (default %d)\n"
            "  -b <Bps>   max console bytes/sec (default %.0f)\n"
            "  -r <Hz>    refresh rate in Hz (default %.1f)\n"
            "  -f <path>  stream_commander state path (default %s)\n"
            "  -h         help\n",
            prog, DEFAULT_LISTEN_IP, DEFAULT_LISTEN_PORT,
            DEFAULT_CONSOLE_BYTES_PER_SEC, DEFAULT_PRINT_HZ,
            DEFAULT_STREAM_CMD_STATE_PATH);
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(
        MAV_CFG_KEY_STREAM_ODOM_TEST_LISTEN_IP,
        mav_cfg_get_str(MAV_CFG_KEY_STREAM_ODOM_OUT_IP, DEFAULT_LISTEN_IP));
    int listen_port = mav_cfg_get_int(
        MAV_CFG_KEY_STREAM_ODOM_TEST_LISTEN_PORT,
        mav_cfg_get_int(MAV_CFG_KEY_STREAM_ODOM_OUT_PORT, DEFAULT_LISTEN_PORT));
    double max_bytes_per_sec = mav_cfg_get_double(MAV_CFG_KEY_STREAM_ODOM_TEST_CONSOLE_BPS, DEFAULT_CONSOLE_BYTES_PER_SEC);
    double print_hz = mav_cfg_get_double(MAV_CFG_KEY_STREAM_ODOM_TEST_PRINT_HZ, DEFAULT_PRINT_HZ);
    const char *stream_cmd_state_path = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_STATE_PATH, DEFAULT_STREAM_CMD_STATE_PATH);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            max_bytes_per_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            print_hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            stream_cmd_state_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (!(max_bytes_per_sec > 0.0) || !isfinite(max_bytes_per_sec)) {
        fprintf(stderr, "invalid -b value: %.3f\n", max_bytes_per_sec);
        return 1;
    }
    if (!(print_hz > 0.0) || !isfinite(print_hz)) {
        fprintf(stderr, "invalid -r value: %.3f\n", print_hz);
        return 1;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int fd = setup_udp_listener(listen_ip, listen_port);
    if (fd < 0) {
        fprintf(stderr, "failed to bind %s:%d: %s\n", listen_ip, listen_port, strerror(errno));
        return 1;
    }

    fprintf(stdout, "stream_odometry_test: listening %s:%d (line-refresh %.2fHz, max %.0f B/s)\n",
            listen_ip, listen_port, print_hz, max_bytes_per_sec);

    const double byte_rate = max_bytes_per_sec;
    double tokens = byte_rate;
    double max_tokens = byte_rate;
    double last_refill = monotonic_seconds();
    const double print_period = 1.0 / print_hz;
    const double stale_sec = 1.0;
    double next_print = monotonic_seconds();
    unsigned dropped_updates = 0;
    int have_data = 0;
    double last_rx = 0.0;
    float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float rollspeed = 0.0f, pitchspeed = 0.0f, yawspeed = 0.0f;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    uint8_t cmd_arm = 0;
    float cmd_vals[COMMAND_FLOAT_COUNT] = {0};
    uint8_t cmd_rc_count = 0;
    float cmd_rc_vals[MAX_RC_COUNT] = {0};
    int have_cmd = 0;

    while (g_running) {
        double now = monotonic_seconds();
        double timeout_sec = next_print - now;
        if (timeout_sec < 0.0) timeout_sec = 0.0;
        if (timeout_sec > 0.2) timeout_sec = 0.2;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;
        tv.tv_sec = (time_t)timeout_sec;
        tv.tv_usec = (suseconds_t)((timeout_sec - (double)tv.tv_sec) * 1e6);

        int r = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (r > 0 && FD_ISSET(fd, &rfds)) {
            uint8_t buf[1024];
            ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
            if (n < 0) {
                if (errno == EINTR) continue;
            } else {
                if (parse_packet(buf, (size_t)n, q, &rollspeed, &pitchspeed, &yawspeed,
                                 &x, &y, &z, &vx, &vy, &vz)) {
                    have_data = 1;
                    last_rx = monotonic_seconds();
                }
            }
        }

        now = monotonic_seconds();
        if (now >= next_print) {
            char line1[200];
            char line2[200];
            have_cmd = load_stream_command_state(stream_cmd_state_path, &cmd_arm, cmd_vals,
                                                  &cmd_rc_count, cmd_rc_vals);
            if (have_data && (now - last_rx) <= stale_sec) {
                snprintf(line1, sizeof(line1),
                         "q=[%+7.3f %+7.3f %+7.3f %+7.3f] rpy_rate=[%+7.3f %+7.3f %+7.3f] pos=[%+7.3f %+7.3f %+7.3f] vel=[%+7.3f %+7.3f %+7.3f]",
                         q[0], q[1], q[2], q[3],
                         rollspeed, pitchspeed, yawspeed,
                         x, y, z,
                         vx, vy, vz);
            } else {
                snprintf(line1, sizeof(line1), "waiting odometry packet...");
            }

            if (have_cmd) {
                char rc_buf[256];
                rc_buf[0] = '\0';
                if (cmd_rc_count > 0) {
                    size_t off = 0;
                    off += snprintf(rc_buf + off, sizeof(rc_buf) - off, " rc=[");
                    for (uint8_t i = 0; i < cmd_rc_count && off + 8 < sizeof(rc_buf); ++i) {
                        off += snprintf(rc_buf + off, sizeof(rc_buf) - off, "%s%+.3f", (i ? " " : ""), cmd_rc_vals[i]);
                    }
                    snprintf(rc_buf + off, sizeof(rc_buf) - off, "]");
                }
                snprintf(line2, sizeof(line2),
                         "cmd arm=%u motor=[%+6.3f %+6.3f %+6.3f %+6.3f] servo=[%+6.3f %+6.3f %+6.3f %+6.3f]%s drop=%u",
                         (unsigned)cmd_arm,
                         cmd_vals[0], cmd_vals[1], cmd_vals[2], cmd_vals[3],
                         cmd_vals[4], cmd_vals[5], cmd_vals[6], cmd_vals[7],
                         rc_buf,
                         dropped_updates);
            } else {
                snprintf(line2, sizeof(line2), "cmd waiting... drop=%u", dropped_updates);
            }

            char render_line[460];
            int len = snprintf(render_line, sizeof(render_line),
                               "\r\033[K%s\n\033[K%s\033[F\r",
                               line1, line2);
            if (len > 0) {
                if (!render_line_with_rate_limit(render_line, (size_t)len, &tokens, max_tokens, byte_rate, &last_refill)) {
                    dropped_updates++;
                } else {
                    dropped_updates = 0;
                }
            }

            double periods = floor((now - next_print) / print_period) + 1.0;
            if (periods < 1.0) periods = 1.0;
            next_print += periods * print_period;
        }
    }

    /* Clear the two-line live view before returning to shell prompt. */
    printf("\r\033[K\n\033[K\n");
    fflush(stdout);
    close(fd);
    return 0;
}
