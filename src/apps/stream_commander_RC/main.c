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

#define RC_COUNT 8
#define DEFAULT_SEND_HZ 20.0
#define DEFAULT_ARM 0
#define DEFAULT_RC_INIT 1500
#define DEFAULT_RC_MIN 1000
#define DEFAULT_RC_MAX 2000
#define RC_STEP 100

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
    if (tcgetattr(STDIN_FILENO, &g_old_tio) != 0) {
        return -1;
    }
    g_termios_ok = 1;
    atexit(restore_termios);

    tio = g_old_tio;
    tio.c_lflag &= ~(ICANON | ECHO);
    tio.c_iflag &= ~(IXON | ICRNL);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    return tcsetattr(STDIN_FILENO, TCSAFLUSH, &tio);
}

static double mono_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void write_u16_le(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static size_t encode_packet(uint8_t arm, const uint16_t rc[RC_COUNT], uint8_t *out, size_t out_cap) {
    const size_t need = 1 + 1 + 1 + (RC_COUNT * 2) + 1 + 1 + 1;
    if (!out || out_cap < need) return 0;

    size_t off = 0;
    out[off++] = START_TOKEN;
    out[off++] = arm ? 1u : 0u;
    out[off++] = RC_COUNT;
    for (int i = 0; i < RC_COUNT; ++i) {
        write_u16_le(out + off, rc[i]);
        off += 2;
    }
    out[off++] = 0; /* actuator_count */
    out[off++] = 0; /* manual_level */
    out[off++] = END_TOKEN;
    return off;
}

static int clamp_rc(int v) {
    if (v < DEFAULT_RC_MIN) return DEFAULT_RC_MIN;
    if (v > DEFAULT_RC_MAX) return DEFAULT_RC_MAX;
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

static void print_ui(uint8_t arm, int selected, const uint16_t rc[RC_COUNT]) {
    printf("\r\033[2K");
    printf("ARM:%u ", (unsigned)arm);
    for (int i = 0; i < RC_COUNT; ++i) {
        if (i == selected) {
            printf("CH%d[%u] ", i + 1, (unsigned)rc[i]);
        } else {
            printf("CH%d:%u ", i + 1, (unsigned)rc[i]);
        }
    }
    fflush(stdout);
}

static int read_key_event(int *event) {
    uint8_t b[8];
    ssize_t n = read(STDIN_FILENO, b, sizeof(b));
    if (n <= 0) return 0;

    /* Arrow keys first: ESC [ A/B/C/D */
    if (n >= 3 && b[0] == 0x1b && b[1] == '[') {
        if (b[2] == 'A') { *event = 'U'; return 1; }
        if (b[2] == 'B') { *event = 'D'; return 1; }
        if (b[2] == 'C') { *event = 'R'; return 1; }
        if (b[2] == 'D') { *event = 'L'; return 1; }
    }

    for (ssize_t i = 0; i < n; ++i) {
        if (b[i] == 'q' || b[i] == 'Q') {
            *event = 'q';
            return 1;
        }
        if (b[i] == 'a') {
            *event = 'x';
            return 1;
        }
        if (b[i] == 'A') {
            *event = 'x';
            return 1;
        }
        if (b[i] == 'z' || b[i] == 'Z') {
            *event = 'z';
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    const char *dest_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_LISTEN_IP, "127.0.0.1");
    int dest_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT, 14531);
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

    uint8_t arm = DEFAULT_ARM;
    uint16_t rc[RC_COUNT];
    for (int i = 0; i < RC_COUNT; ++i) rc[i] = DEFAULT_RC_INIT;
    int selected = 0;

    uint8_t pkt[64];
    double next_send = mono_now();

    printf("stream_commander_RC\n");
    printf("keys: <- -> select | up/down adjust(100) | z=disarm | x=arm | q=quit\n");
    print_ui(arm, selected, rc);

    while (g_running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000; /* 10ms */

        int r = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (r > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
            int ev = 0;
            if (read_key_event(&ev)) {
                if (ev == 'q') {
                    break;
                } else if (ev == 'L') {
                    selected = (selected + RC_COUNT - 1) % RC_COUNT;
                } else if (ev == 'R') {
                    selected = (selected + 1) % RC_COUNT;
                } else if (ev == 'U') {
                    rc[selected] = (uint16_t)clamp_rc((int)rc[selected] + RC_STEP);
                } else if (ev == 'D') {
                    rc[selected] = (uint16_t)clamp_rc((int)rc[selected] - RC_STEP);
                } else if (ev == 'z') {
                    arm = 0;
                } else if (ev == 'x') {
                    arm = 1;
                }
                print_ui(arm, selected, rc);
            }
        }

        double now = mono_now();
        if (now >= next_send) {
            size_t n = encode_packet(arm, rc, pkt, sizeof(pkt));
            if (n > 0) {
                sendto(sock, pkt, n, 0, (const struct sockaddr *)&dest, sizeof(dest));
            }
            next_send += period;
            if (now - next_send > 1.0) {
                next_send = now + period;
            }
        }
    }

    printf("\nstream_commander_RC stopped\n");
    close(sock);
    return 0;
}
