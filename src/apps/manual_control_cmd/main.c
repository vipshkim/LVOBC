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

#include <mavlink.h>

#include "rocket_mav_common.h"

#define DEFAULT_RATE_HZ 20.0
#define DEFAULT_DURATION_SEC 1.0

static int clamp_axis(int v) {
    if (v < -1000) return -1000;
    if (v > 1000) return 1000;
    return v;
}

static int clamp_throttle(int v) {
    if (v < -1000) return -1000;
    if (v > 1000) return 1000;
    return v;
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
            "  -i <ip>        target ip (default from ROCKET_MAV_TOOLS_TARGET_IP)\n"
            "  -p <port>      target port (default from ROCKET_MAV_TOOLS_TARGET_PORT)\n"
            "  -s <sysid>     sender sysid (default from ROCKET_MAV_TOOLS_SYSID)\n"
            "  -c <compid>    sender compid (default from ROCKET_MAV_TOOLS_COMPID)\n"
            "  -t <target>    target system id (default from ROCKET_MAV_TOOLS_TARGET_SYS)\n"
            "  -k <target>    target component id (default from ROCKET_MAV_TOOLS_TARGET_COMP)\n"
            "  -x <val>       x axis [-1000..1000] (pitch)\n"
            "  -y <val>       y axis [-1000..1000] (roll)\n"
            "  -z <val>       z axis [-1000..1000] (throttle)\n"
            "  -r <val>       r axis [-1000..1000] (yaw)\n"
            "  -b <mask>      buttons bitmask uint16 (default 0)\n"
            "  -f <mask>      buttons2 bitmask uint16 (default 0)\n"
            "  -n <hz>        send rate hz (default %.1f)\n"
            "  -d <sec>       duration sec (default %.1f)\n"
            "  -1             one-shot send\n"
            "  -h             help\n",
            prog, DEFAULT_RATE_HZ, DEFAULT_DURATION_SEC);
}

int main(int argc, char **argv) {
    const char *ip = mav_cfg_get_str(MAV_CFG_KEY_TOOLS_TARGET_IP, MAV_DEFAULT_IP);
    int port = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_PORT, 15651);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_SYSID, MAV_DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_COMPID, MAV_DEFAULT_COMPID);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_SYS, MAV_DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_TOOLS_TARGET_COMP, MAV_DEFAULT_TARGET_COMP);

    int x = 0, y = 0, z = 500, r = 0;
    unsigned int buttons = 0;
    unsigned int buttons2 = 0;
    double rate_hz = DEFAULT_RATE_HZ;
    double duration_sec = DEFAULT_DURATION_SEC;
    int one_shot = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) ip = argv[++i];
        else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) port = atoi(argv[++i]);
        else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) sysid = atoi(argv[++i]);
        else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) compid = atoi(argv[++i]);
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) target_sys = atoi(argv[++i]);
        else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) target_comp = atoi(argv[++i]);
        else if (strcmp(argv[i], "-x") == 0 && i + 1 < argc) x = atoi(argv[++i]);
        else if (strcmp(argv[i], "-y") == 0 && i + 1 < argc) y = atoi(argv[++i]);
        else if (strcmp(argv[i], "-z") == 0 && i + 1 < argc) z = atoi(argv[++i]);
        else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) r = atoi(argv[++i]);
        else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) buttons = (unsigned int)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) buttons2 = (unsigned int)strtoul(argv[++i], NULL, 0);
        else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) rate_hz = atof(argv[++i]);
        else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) duration_sec = atof(argv[++i]);
        else if (strcmp(argv[i], "-1") == 0) one_shot = 1;
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (rate_hz <= 0.0) rate_hz = DEFAULT_RATE_HZ;
    if (duration_sec <= 0.0) duration_sec = DEFAULT_DURATION_SEC;

    x = clamp_axis(x);
    y = clamp_axis(y);
    z = clamp_throttle(z);
    r = clamp_axis(r);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &dest.sin_addr) != 1) {
        fprintf(stderr, "invalid ip: %s\n", ip);
        close(fd);
        return 1;
    }

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint64_t sent = 0;
    double period = 1.0 / rate_hz;
    double end_t = mono_now_sec() + duration_sec;

    printf("manual_control_cmd: target=%s:%d sys=%d comp=%d -> tsys=%d tcomp=%d\n",
           ip, port, sysid, compid, target_sys, target_comp);
    printf("x=%d y=%d z=%d r=%d buttons=0x%X buttons2=0x%X rate=%.1fHz dur=%.1fs one_shot=%d\n",
           x, y, z, r, buttons, buttons2, rate_hz, duration_sec, one_shot);

    do {
        mavlink_message_t msg;
        mavlink_msg_manual_control_pack(
            (uint8_t)sysid,
            (uint8_t)compid,
            &msg,
            (uint8_t)target_sys,
            (int16_t)x,
            (int16_t)y,
            (int16_t)z,
            (int16_t)r,
            (uint16_t)buttons,
            (uint16_t)buttons2,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0);

        uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        if (sendto(fd, buf, n, 0, (const struct sockaddr *)&dest, sizeof(dest)) < 0) {
            perror("sendto");
            close(fd);
            return 1;
        }
        sent++;
        if (one_shot) break;

        struct timespec ts;
        ts.tv_sec = (time_t)period;
        ts.tv_nsec = (long)((period - (double)ts.tv_sec) * 1e9);
        if (ts.tv_nsec < 0) ts.tv_nsec = 0;
        nanosleep(&ts, NULL);
    } while (mono_now_sec() < end_t);

    printf("sent %llu packet(s)\n", (unsigned long long)sent);
    close(fd);
    return 0;
}
