#define _DEFAULT_SOURCE 1

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "rocket_mav_common.h"

#define START_TOKEN ((uint8_t)'$')
#define END_TOKEN ((uint8_t)'\n')

#define MAX_ACT_CHANNELS 12
#define DEFAULT_SEND_HZ 20.0
#define DEFAULT_ACT_COUNT 6
#define ACT_STEP 0.1f
#define ACT_MIN -1.0f
#define ACT_MAX 1.0f

static volatile sig_atomic_t g_running = 1;
static struct termios g_old_tio;
static int g_termios_ok = 0;

static void on_signal(int signo) {
    (void)signo;
    g_running = 0;
}

static void restore_termios(void) {
    if (g_termios_ok) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_old_tio);
    }
}

static int enable_raw_mode(void) {
    struct termios tio;
    if (tcgetattr(STDIN_FILENO, &g_old_tio) != 0) return -1;
    g_termios_ok = 1;
    atexit(restore_termios);

    tio = g_old_tio;
    tio.c_lflag &= ~(ICANON | ECHO);
    tio.c_iflag &= ~(IXON | ICRNL);
    tio.c_oflag &= ~(OPOST);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    return tcsetattr(STDIN_FILENO, TCSAFLUSH, &tio);
}

static double mono_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void write_f32_le(uint8_t *dst, float v) {
    union {
        uint32_t u;
        float f;
    } cvt;
    cvt.f = v;
    dst[0] = (uint8_t)(cvt.u & 0xFFu);
    dst[1] = (uint8_t)((cvt.u >> 8) & 0xFFu);
    dst[2] = (uint8_t)((cvt.u >> 16) & 0xFFu);
    dst[3] = (uint8_t)((cvt.u >> 24) & 0xFFu);
}

/* Packet format:
 * '$' arm(u8=0) rc_count(u8=0) act_count(u8) act(float32_le)*N '\n'
 */
static size_t encode_packet(uint8_t act_count, const float *act, uint8_t *out, size_t out_cap) {
    size_t need = 1 + 1 + 1 + 1 + ((size_t)act_count * 4) + 1 + 1;
    if (!out || out_cap < need) return 0;

    size_t off = 0;
    out[off++] = START_TOKEN;
    out[off++] = 0; /* arm fixed disarmed */
    out[off++] = 0; /* rc_count */
    out[off++] = act_count;
    for (uint8_t i = 0; i < act_count; ++i) {
        write_f32_le(out + off, act[i]);
        off += 4;
    }
    out[off++] = 0; /* manual_level */
    out[off++] = END_TOKEN;
    return off;
}

static float clamp_act(float v) {
    if (v < ACT_MIN) return ACT_MIN;
    if (v > ACT_MAX) return ACT_MAX;
    return v;
}

static int setup_udp_sender(const char *ip, int port, struct sockaddr_in *dest) {
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

static void print_ui(int selected, uint8_t act_count, const float *act) {
    printf("\r\033[2KACT ");
    for (uint8_t i = 0; i < act_count; ++i) {
        if ((int)i == selected) {
            printf("A%u[%+.1f] ", (unsigned)(i + 1), act[i]);
        } else {
            printf("A%u:%+.1f ", (unsigned)(i + 1), act[i]);
        }
    }
    fflush(stdout);
}

static int read_key_event(int *event) {
    uint8_t b[8];
    ssize_t n = read(STDIN_FILENO, b, sizeof(b));
    if (n <= 0) return 0;

    for (ssize_t i = 0; i < n; ++i) {
        if (b[i] == 'q' || b[i] == 'Q') {
            *event = 'q';
            return 1;
        }
    }

    if (n >= 3 && b[0] == 0x1b && b[1] == '[') {
        if (b[2] == 'A') { *event = 'U'; return 1; }
        if (b[2] == 'B') { *event = 'D'; return 1; }
        if (b[2] == 'C') { *event = 'R'; return 1; }
        if (b[2] == 'D') { *event = 'L'; return 1; }
    }
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [-n channels]\n"
            "  -n <count>  actuator channel count (1..12, default %d)\n",
            prog, DEFAULT_ACT_COUNT);
}

int main(int argc, char **argv) {
    uint8_t act_count = DEFAULT_ACT_COUNT;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            int n = atoi(argv[++i]);
            if (n < 1 || n > MAX_ACT_CHANNELS) {
                fprintf(stderr, "invalid -n (1..12)\n");
                return 1;
            }
            act_count = (uint8_t)n;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    const char *dest_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_LISTEN_IP, "127.0.0.1");
    int dest_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT, 15500);
    double send_hz = DEFAULT_SEND_HZ;
    double period = 1.0 / send_hz;

    struct sockaddr_in dest;
    int sock = setup_udp_sender(dest_ip, dest_port, &dest);
    if (sock < 0) {
        fprintf(stderr, "failed to setup sender %s:%d: %s\n", dest_ip, dest_port, strerror(errno));
        return 1;
    }

    if (enable_raw_mode() != 0) {
        fprintf(stderr, "failed to enable raw mode\n");
        close(sock);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    float act[MAX_ACT_CHANNELS];
    for (int i = 0; i < MAX_ACT_CHANNELS; ++i) act[i] = 0.0f;
    int selected = 0;

    uint8_t pkt[256];
    double next_send = mono_now();

    printf("stream_commander_ACTUATOR\n");
    printf("keys: <- -> select | up/down adjust(0.1) | q=quit\n");
    print_ui(selected, act_count, act);

    while (g_running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;

        int r = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (r > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
            int ev = 0;
            if (read_key_event(&ev)) {
                if (ev == 'q') {
                    break;
                } else if (ev == 'L') {
                    selected = (selected + act_count - 1) % act_count;
                } else if (ev == 'R') {
                    selected = (selected + 1) % act_count;
                } else if (ev == 'U') {
                    act[selected] = clamp_act(act[selected] + ACT_STEP);
                } else if (ev == 'D') {
                    act[selected] = clamp_act(act[selected] - ACT_STEP);
                }
                print_ui(selected, act_count, act);
            }
        }

        double now = mono_now();
        if (now >= next_send) {
            size_t n = encode_packet(act_count, act, pkt, sizeof(pkt));
            if (n > 0) {
                sendto(sock, pkt, n, 0, (const struct sockaddr *)&dest, sizeof(dest));
            }
            next_send += period;
            if (now - next_send > 1.0) {
                next_send = now + period;
            }
        }
    }

    printf("\nstream_commander_ACTUATOR stopped\n");
    close(sock);
    return 0;
}
