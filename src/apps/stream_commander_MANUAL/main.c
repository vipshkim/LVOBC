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

#define DEFAULT_SEND_HZ 20.0
#define DEFAULT_ARM 0
#define DEFAULT_MANUAL_LEVEL 15
#define DEFAULT_EXT_MASK 0xFFu

#define MANUAL_STEP 50
#define BUTTON_STEP 1
#define EXT_STEP 1

#define MANUAL_MIN (-1000)
#define MANUAL_MAX (1000)
#define BUTTON_MIN 0
#define BUTTON_MAX 65535
#define EXT_MIN 0
#define EXT_MAX 255

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

static void write_i16_le(uint8_t *dst, int16_t v) {
    write_u16_le(dst, (uint16_t)v);
}

typedef struct {
    int16_t x, y, z, r;
    uint16_t buttons, buttons2;
    uint8_t enabled_extensions;
    int16_t s, t;
    int16_t aux1, aux2, aux3, aux4, aux5, aux6;
} manual_state_t;

typedef enum {
    FIELD_I16,
    FIELD_U16,
    FIELD_U8,
} field_type_t;

typedef struct {
    const char *name;
    void *ptr;
    field_type_t type;
    int32_t min;
    int32_t max;
    int32_t step;
} field_spec_t;

static int32_t clamp_i32(int32_t v, int32_t min_v, int32_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static int32_t field_get(const field_spec_t *f) {
    if (!f || !f->ptr) return 0;
    switch (f->type) {
        case FIELD_I16: return *(int16_t *)f->ptr;
        case FIELD_U16: return *(uint16_t *)f->ptr;
        case FIELD_U8: return *(uint8_t *)f->ptr;
        default: return 0;
    }
}

static void field_set(const field_spec_t *f, int32_t v) {
    if (!f || !f->ptr) return;
    v = clamp_i32(v, f->min, f->max);
    switch (f->type) {
        case FIELD_I16: *(int16_t *)f->ptr = (int16_t)v; break;
        case FIELD_U16: *(uint16_t *)f->ptr = (uint16_t)v; break;
        case FIELD_U8: *(uint8_t *)f->ptr = (uint8_t)v; break;
        default: break;
    }
}

static size_t encode_packet(uint8_t arm, uint8_t manual_level, const manual_state_t *m,
                            uint8_t *out, size_t out_cap) {
    if (!m || !out) return 0;

    size_t need = 1 + 1 + 1 + 0 + 1 + 0 + 1 + 1;
    if (manual_level >= 1) need += 2;
    if (manual_level >= 2) need += 2;
    if (manual_level >= 3) need += 2;
    if (manual_level >= 4) need += 2;
    if (manual_level >= 5) need += 2;
    if (manual_level >= 6) need += 2;
    if (manual_level >= 7) need += 1;
    if (manual_level >= 8) need += 2;
    if (manual_level >= 9) need += 2;
    if (manual_level >= 10) need += 2;
    if (manual_level >= 11) need += 2;
    if (manual_level >= 12) need += 2;
    if (manual_level >= 13) need += 2;
    if (manual_level >= 14) need += 2;
    if (manual_level >= 15) need += 2;
    if (out_cap < need) return 0;

    size_t off = 0;
    out[off++] = START_TOKEN;
    out[off++] = arm ? 1u : 0u;
    out[off++] = 0; /* rc_count */
    out[off++] = 0; /* act_count */
    out[off++] = manual_level;

    if (manual_level >= 1) { write_i16_le(out + off, m->x); off += 2; }
    if (manual_level >= 2) { write_i16_le(out + off, m->y); off += 2; }
    if (manual_level >= 3) { write_i16_le(out + off, m->z); off += 2; }
    if (manual_level >= 4) { write_i16_le(out + off, m->r); off += 2; }
    if (manual_level >= 5) { write_u16_le(out + off, m->buttons); off += 2; }
    if (manual_level >= 6) { write_u16_le(out + off, m->buttons2); off += 2; }
    if (manual_level >= 7) { out[off++] = m->enabled_extensions; }
    if (manual_level >= 8) { write_i16_le(out + off, m->s); off += 2; }
    if (manual_level >= 9) { write_i16_le(out + off, m->t); off += 2; }
    if (manual_level >= 10){ write_i16_le(out + off, m->aux1); off += 2; }
    if (manual_level >= 11){ write_i16_le(out + off, m->aux2); off += 2; }
    if (manual_level >= 12){ write_i16_le(out + off, m->aux3); off += 2; }
    if (manual_level >= 13){ write_i16_le(out + off, m->aux4); off += 2; }
    if (manual_level >= 14){ write_i16_le(out + off, m->aux5); off += 2; }
    if (manual_level >= 15){ write_i16_le(out + off, m->aux6); off += 2; }

    out[off++] = END_TOKEN;
    return off;
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

static void print_ui(uint8_t arm, int selected, const field_spec_t *fields, int count) {
    printf("\r\033[2KARM:%u ", (unsigned)arm);
    for (int i = 0; i < count; ++i) {
        int32_t v = field_get(&fields[i]);
        if (i == selected) {
            printf("%s[%d] ", fields[i].name, (int)v);
        } else {
            printf("%s:%d ", fields[i].name, (int)v);
        }
    }
    fflush(stdout);
}

static int read_key_event(int *event) {
    uint8_t b[8];
    ssize_t n = read(STDIN_FILENO, b, sizeof(b));
    if (n <= 0) return 0;

    if (n >= 3 && b[0] == 0x1b && b[1] == '[') {
        if (b[2] == 'A') { *event = 'U'; return 1; }
        if (b[2] == 'B') { *event = 'D'; return 1; }
        if (b[2] == 'C') { *event = 'R'; return 1; }
        if (b[2] == 'D') { *event = 'L'; return 1; }
    }

    for (ssize_t i = 0; i < n; ++i) {
        if (b[i] == 'q' || b[i] == 'Q') { *event = 'q'; return 1; }
        if (b[i] == 'x' || b[i] == 'X') { *event = 'x'; return 1; }
        if (b[i] == 'z' || b[i] == 'Z') { *event = 'z'; return 1; }
        if (b[i] == 'c' || b[i] == 'C') { *event = 'c'; return 1; }
    }
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [-l level]\n"
            "  -l <0..15>  manual payload level (default %d)\n",
            prog, DEFAULT_MANUAL_LEVEL);
}

int main(int argc, char **argv) {
    uint8_t manual_level = DEFAULT_MANUAL_LEVEL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            int lvl = atoi(argv[++i]);
            if (lvl < 0 || lvl > 15) {
                fprintf(stderr, "invalid -l (0..15)\n");
                return 1;
            }
            manual_level = (uint8_t)lvl;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

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
    manual_state_t manual;
    memset(&manual, 0, sizeof(manual));
    manual.enabled_extensions = DEFAULT_EXT_MASK;

    field_spec_t fields[] = {
        {"x", &manual.x, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"y", &manual.y, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"z", &manual.z, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"r", &manual.r, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"btn", &manual.buttons, FIELD_U16, BUTTON_MIN, BUTTON_MAX, BUTTON_STEP},
        {"btn2", &manual.buttons2, FIELD_U16, BUTTON_MIN, BUTTON_MAX, BUTTON_STEP},
        {"ext", &manual.enabled_extensions, FIELD_U8, EXT_MIN, EXT_MAX, EXT_STEP},
        {"s", &manual.s, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"t", &manual.t, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"aux1", &manual.aux1, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"aux2", &manual.aux2, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"aux3", &manual.aux3, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"aux4", &manual.aux4, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"aux5", &manual.aux5, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
        {"aux6", &manual.aux6, FIELD_I16, MANUAL_MIN, MANUAL_MAX, MANUAL_STEP},
    };

    int field_count = (int)(sizeof(fields) / sizeof(fields[0]));
    int selected = 0;

    uint8_t pkt[128];
    double next_send = mono_now();

    printf("stream_commander_MANUAL\n");
    printf("keys: <- -> select | up/down adjust | z=disarm | x=arm | c=center | q=quit\n");
    printf("manual_level=%u (enabled_extensions=0x%02X)\n", (unsigned)manual_level, (unsigned)manual.enabled_extensions);
    print_ui(arm, selected, fields, field_count);

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
                    selected = (selected + field_count - 1) % field_count;
                } else if (ev == 'R') {
                    selected = (selected + 1) % field_count;
                } else if (ev == 'U') {
                    int32_t v = field_get(&fields[selected]);
                    field_set(&fields[selected], v + fields[selected].step);
                } else if (ev == 'D') {
                    int32_t v = field_get(&fields[selected]);
                    field_set(&fields[selected], v - fields[selected].step);
                } else if (ev == 'z') {
                    arm = 0;
                } else if (ev == 'x') {
                    arm = 1;
                } else if (ev == 'c') {
                    field_set(&fields[selected], 0);
                }
                print_ui(arm, selected, fields, field_count);
            }
        }

        double now = mono_now();
        if (now >= next_send) {
            size_t n = encode_packet(arm, manual_level, &manual, pkt, sizeof(pkt));
            if (n > 0) {
                sendto(sock, pkt, n, 0, (const struct sockaddr *)&dest, sizeof(dest));
            }
            next_send += period;
            if (now - next_send > 1.0) {
                next_send = now + period;
            }
        }
    }

    printf("\nstream_commander_MANUAL stopped\n");
    close(sock);
    return 0;
}
