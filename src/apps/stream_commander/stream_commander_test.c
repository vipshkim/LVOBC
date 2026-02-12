#define _DEFAULT_SOURCE 1

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "rocket_mav_common.h"

#define DEFAULT_DEST_IP "127.0.0.1"
#define DEFAULT_DEST_PORT 14531
#define DEFAULT_ARM 0
#define DEFAULT_HZ 20.0
#define DEFAULT_DURATION 10.0
#define DEFAULT_CH1_HOLD 1500
#define DEFAULT_CH1_FINAL 1000
#define DEFAULT_FINAL_DURATION 1.0

#define START_TOKEN ((uint8_t)'$')
#define END_TOKEN ((uint8_t)'\n')
#define MAX_RC_CHANNELS 18
#define MAX_ACTUATOR_CHANNELS 12

typedef struct {
    uint8_t arm;
    uint8_t rc_count;
    uint16_t rc[MAX_RC_CHANNELS];
    uint8_t act_count;
    float act[MAX_ACTUATOR_CHANNELS];
    uint8_t manual_level;
    int16_t mx, my, mz, mr;
    uint16_t mbuttons, mbuttons2;
    uint8_t menabled_extensions;
    int16_t ms, mt, maux1, maux2, maux3, maux4, maux5, maux6;
} command_packet_t;

static double mono_now_sec(void);

static uint16_t clamp_u16(long v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return (uint16_t)v;
}

static int parse_rc_list(const char *s, command_packet_t *pkt) {
    if (!s || !pkt) return 0;
    pkt->rc_count = 0;
    if (*s == '\0') return 1;

    char *dup = strdup(s);
    if (!dup) return 0;

    char *save = NULL;
    char *tok = strtok_r(dup, ",", &save);
    while (tok) {
        if (pkt->rc_count >= MAX_RC_CHANNELS) {
            free(dup);
            return 0;
        }
        char *end = NULL;
        long v = strtol(tok, &end, 10);
        if (!end || *end != '\0') {
            free(dup);
            return 0;
        }
        pkt->rc[pkt->rc_count++] = clamp_u16(v);
        tok = strtok_r(NULL, ",", &save);
    }

    free(dup);
    return 1;
}

static int parse_act_list(const char *s, command_packet_t *pkt) {
    if (!s || !pkt) return 0;
    pkt->act_count = 0;
    if (*s == '\0') return 1;

    char *dup = strdup(s);
    if (!dup) return 0;

    char *save = NULL;
    char *tok = strtok_r(dup, ",", &save);
    while (tok) {
        if (pkt->act_count >= MAX_ACTUATOR_CHANNELS) {
            free(dup);
            return 0;
        }
        char *end = NULL;
        float v = strtof(tok, &end);
        if (!end || *end != '\0') {
            free(dup);
            return 0;
        }
        pkt->act[pkt->act_count++] = v;
        tok = strtok_r(NULL, ",", &save);
    }

    free(dup);
    return 1;
}

static void write_u16_le(uint8_t *dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void write_i16_le(uint8_t *dst, int16_t v) {
    write_u16_le(dst, (uint16_t)v);
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

static size_t encode_packet(const command_packet_t *pkt, uint8_t *out, size_t out_cap) {
    if (!pkt || !out) return 0;

    size_t need = 1 + 1 + 1 + ((size_t)pkt->rc_count * 2) + 1 + ((size_t)pkt->act_count * 4) + 1 + 1;
    if (pkt->manual_level >= 1) need += 2;  /* x */
    if (pkt->manual_level >= 2) need += 2;  /* y */
    if (pkt->manual_level >= 3) need += 2;  /* z */
    if (pkt->manual_level >= 4) need += 2;  /* r */
    if (pkt->manual_level >= 5) need += 2;  /* buttons */
    if (pkt->manual_level >= 6) need += 2;  /* buttons2 */
    if (pkt->manual_level >= 7) need += 1;  /* enabled_extensions */
    if (pkt->manual_level >= 8) need += 2;  /* s */
    if (pkt->manual_level >= 9) need += 2;  /* t */
    if (pkt->manual_level >= 10) need += 2; /* aux1 */
    if (pkt->manual_level >= 11) need += 2; /* aux2 */
    if (pkt->manual_level >= 12) need += 2; /* aux3 */
    if (pkt->manual_level >= 13) need += 2; /* aux4 */
    if (pkt->manual_level >= 14) need += 2; /* aux5 */
    if (pkt->manual_level >= 15) need += 2; /* aux6 */
    if (out_cap < need) return 0;

    size_t off = 0;
    out[off++] = START_TOKEN;
    out[off++] = pkt->arm ? 1u : 0u;
    out[off++] = pkt->rc_count;

    for (uint8_t i = 0; i < pkt->rc_count; ++i) {
        write_u16_le(out + off, pkt->rc[i]);
        off += 2;
    }

    out[off++] = pkt->act_count;

    for (uint8_t i = 0; i < pkt->act_count; ++i) {
        write_f32_le(out + off, pkt->act[i]);
        off += 4;
    }

    out[off++] = pkt->manual_level;
    if (pkt->manual_level >= 1) { write_i16_le(out + off, pkt->mx); off += 2; }
    if (pkt->manual_level >= 2) { write_i16_le(out + off, pkt->my); off += 2; }
    if (pkt->manual_level >= 3) { write_i16_le(out + off, pkt->mz); off += 2; }
    if (pkt->manual_level >= 4) { write_i16_le(out + off, pkt->mr); off += 2; }
    if (pkt->manual_level >= 5) { write_u16_le(out + off, pkt->mbuttons); off += 2; }
    if (pkt->manual_level >= 6) { write_u16_le(out + off, pkt->mbuttons2); off += 2; }
    if (pkt->manual_level >= 7) { out[off++] = pkt->menabled_extensions; }
    if (pkt->manual_level >= 8) { write_i16_le(out + off, pkt->ms); off += 2; }
    if (pkt->manual_level >= 9) { write_i16_le(out + off, pkt->mt); off += 2; }
    if (pkt->manual_level >= 10){ write_i16_le(out + off, pkt->maux1); off += 2; }
    if (pkt->manual_level >= 11){ write_i16_le(out + off, pkt->maux2); off += 2; }
    if (pkt->manual_level >= 12){ write_i16_le(out + off, pkt->maux3); off += 2; }
    if (pkt->manual_level >= 13){ write_i16_le(out + off, pkt->maux4); off += 2; }
    if (pkt->manual_level >= 14){ write_i16_le(out + off, pkt->maux5); off += 2; }
    if (pkt->manual_level >= 15){ write_i16_le(out + off, pkt->maux6); off += 2; }

    out[off++] = END_TOKEN;
    return off;
}

static int send_for_duration(int fd, const struct sockaddr_in *dest,
                             const command_packet_t *pkt,
                             double hz, double duration) {
    uint8_t payload[1024];
    size_t payload_len = encode_packet(pkt, payload, sizeof(payload));
    if (payload_len == 0) {
        fprintf(stderr, "packet encode failed\n");
        return -1;
    }

    if (hz <= 0.0) hz = DEFAULT_HZ;
    if (duration <= 0.0) return 0;

    double period = 1.0 / hz;
    double end_t = mono_now_sec() + duration;
    uint64_t sent = 0;

    while (mono_now_sec() < end_t) {
        ssize_t n = sendto(fd, payload, payload_len, 0, (const struct sockaddr *)dest, sizeof(*dest));
        if (n < 0) {
            perror("sendto");
            return -1;
        }
        sent++;

        struct timespec ts;
        ts.tv_sec = (time_t)period;
        ts.tv_nsec = (long)((period - (double)ts.tv_sec) * 1e9);
        if (ts.tv_nsec < 0) ts.tv_nsec = 0;
        nanosleep(&ts, NULL);
    }

    printf("sent %llu packet(s)\n", (unsigned long long)sent);
    return 0;
}

static double mono_now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Options:\n"
            "  -u <ip>         destination ip (default %s)\n"
            "  -p <port>       destination port (default %d)\n"
            "  -a <0|1>        arm value (default %d)\n"
            "  -r <csv>        rc channels uint16 csv, e.g. 1000,1500,1200\n"
            "  -A <csv>        actuator float csv, e.g. 0.1,0.2,-0.3\n"
            "  -m <0..15>      MANUAL_CONTROL payload level (default 0)\n"
            "  -X <val>        manual x (int16)\n"
            "  -Y <val>        manual y (int16)\n"
            "  -Z <val>        manual z (int16)\n"
            "  -R <val>        manual r (int16)\n"
            "  -B <val>        manual buttons (uint16)\n"
            "  -F <val>        manual buttons2 (uint16)\n"
            "  -z <hz>         send rate hz (default %.1f)\n"
            "  -d <sec>        duration sec (default %.1f)\n"
            "  -1              one-shot send\n"
            "  -h              help\n",
            prog,
            DEFAULT_DEST_IP, DEFAULT_DEST_PORT,
            DEFAULT_ARM,
            DEFAULT_HZ,
            DEFAULT_DURATION);
}

int main(int argc, char **argv) {
    const char *dest_ip = mav_cfg_get_str(MAV_CFG_KEY_STREAM_CMD_LISTEN_IP, DEFAULT_DEST_IP);
    int dest_port = mav_cfg_get_int(MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT, DEFAULT_DEST_PORT);
    double hz = DEFAULT_HZ;
    double duration = DEFAULT_DURATION;
    int one_shot = 0;

    command_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.arm = DEFAULT_ARM;

    const char *rc_csv = "";
    const char *act_csv = "";
    int rc_given = 0;
    int act_given = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            dest_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            dest_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            int a = atoi(argv[++i]);
            pkt.arm = a ? 1u : 0u;
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            rc_csv = argv[++i];
            rc_given = 1;
        } else if (strcmp(argv[i], "-A") == 0 && i + 1 < argc) {
            act_csv = argv[++i];
            act_given = 1;
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            int m = atoi(argv[++i]);
            if (m < 0) m = 0;
            if (m > 15) m = 15;
            pkt.manual_level = (uint8_t)m;
        } else if (strcmp(argv[i], "-X") == 0 && i + 1 < argc) {
            pkt.mx = (int16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-Y") == 0 && i + 1 < argc) {
            pkt.my = (int16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-Z") == 0 && i + 1 < argc) {
            pkt.mz = (int16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-R") == 0 && i + 1 < argc) {
            pkt.mr = (int16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-B") == 0 && i + 1 < argc) {
            pkt.mbuttons = (uint16_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-F") == 0 && i + 1 < argc) {
            pkt.mbuttons2 = (uint16_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-z") == 0 && i + 1 < argc) {
            hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            duration = atof(argv[++i]);
        } else if (strcmp(argv[i], "-1") == 0) {
            one_shot = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (hz <= 0.0) hz = DEFAULT_HZ;
    if (duration <= 0.0) duration = DEFAULT_DURATION;

    if (!parse_rc_list(rc_csv, &pkt)) {
        fprintf(stderr, "invalid -r csv\n");
        return 1;
    }
    if (!parse_act_list(act_csv, &pkt)) {
        fprintf(stderr, "invalid -A csv\n");
        return 1;
    }

    uint8_t payload[1024];
    size_t payload_len = encode_packet(&pkt, payload, sizeof(payload));
    if (payload_len == 0) {
        fprintf(stderr, "packet encode failed\n");
        return 1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)dest_port);
    if (inet_pton(AF_INET, dest_ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "invalid ip: %s\n", dest_ip);
        close(fd);
        return 1;
    }

    printf("stream_commander_test: send -> %s:%d arm=%u rc_count=%u act_count=%u manual=%u hz=%.2f dur=%.2f one_shot=%d\n",
           dest_ip, dest_port, (unsigned)pkt.arm, (unsigned)pkt.rc_count, (unsigned)pkt.act_count, (unsigned)pkt.manual_level,
           hz, duration, one_shot);

    /* No args/default behavior: CH1=1500 for 10s, then CH1=1000 for 1s */
    if (!rc_given && !act_given && !one_shot) {
        command_packet_t phase = pkt;
        memset(&phase, 0, sizeof(phase));
        phase.arm = pkt.arm;
        phase.rc_count = 1;
        phase.rc[0] = DEFAULT_CH1_HOLD;

        printf("default profile phase1: CH1=%u for %.1fs at %.2fHz\n",
               (unsigned)phase.rc[0], DEFAULT_DURATION, hz);
        if (send_for_duration(fd, &dest, &phase, hz, DEFAULT_DURATION) != 0) {
            close(fd);
            return 1;
        }

        phase.rc[0] = DEFAULT_CH1_FINAL;
        printf("default profile phase2: CH1=%u for %.1fs at %.2fHz\n",
               (unsigned)phase.rc[0], DEFAULT_FINAL_DURATION, hz);
        if (send_for_duration(fd, &dest, &phase, hz, DEFAULT_FINAL_DURATION) != 0) {
            close(fd);
            return 1;
        }

        close(fd);
        return 0;
    }

    if (one_shot) {
        ssize_t n = sendto(fd, payload, payload_len, 0, (const struct sockaddr *)&dest, sizeof(dest));
        if (n < 0) {
            perror("sendto");
            close(fd);
            return 1;
        }
        printf("sent 1 packet (%zu bytes)\n", payload_len);
        close(fd);
        return 0;
    }

    if (send_for_duration(fd, &dest, &pkt, hz, duration) != 0) {
        close(fd);
        return 1;
    }
    close(fd);
    return 0;
}
