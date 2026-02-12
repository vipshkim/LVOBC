#define _DEFAULT_SOURCE 1

#ifndef MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_USE_MESSAGE_INFO 1
#endif

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>
#include "rocket_common.h"
#include "rocket_mav_common.h"

#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_RED     "\033[31m"
#define CLR_GREEN   "\033[32m"
#define CLR_YELLOW  "\033[33m"
#define CLR_CYAN    "\033[36m"

#define DEFAULT_LISTEN_IP MAV_DEFAULT_IP
#define DEFAULT_LISTEN_PORT 15541
#define DEFAULT_TARGET_IP MAV_DEFAULT_IP
#define DEFAULT_TARGET_PORT 15641
#define DEFAULT_SYSID MAV_DEFAULT_SYSID
#define DEFAULT_COMPID 193
#define DEFAULT_TARGET_SYS MAV_DEFAULT_TARGET_SYS
#define DEFAULT_TARGET_COMP MAV_DEFAULT_TARGET_COMP
#define DEFAULT_SERIAL_BAUD 921600

#define DEFAULT_TIMEOUT_SEC 3.0
#define DEFAULT_RETRIES 3

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static double decode_param_number(uint8_t type, float value) {
    return rocket_decode_param_number(type, value);
}

static float encode_param_value(uint8_t type, double value) {
    return rocket_encode_param_value(type, value);
}

static void format_param_value(uint8_t type, float value, char *out, size_t out_len) {
    rocket_format_param_value(type, value, out, out_len);
}

static int open_serial_port(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    speed_t speed = B115200;
    switch (baud) {
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: speed = B115200; break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static ssize_t send_mavlink_packet(int io_fd, bool serial_mode, const struct sockaddr_in *dest,
                                   const uint8_t *buf, size_t len) {
    if (serial_mode) return write(io_fd, buf, len);
    return sendto(io_fd, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static int value_matches(uint8_t type, float actual, double expected) {
    return rocket_param_value_matches(type, actual, expected, 1e-5);
}

static void fill_param_id(char *dst, size_t dst_len, const char *name) {
    memset(dst, 0, dst_len);
    size_t n = strlen(name);
    if (n > dst_len) n = dst_len;
    memcpy(dst, name, n);
}

static int request_param_value_once(int io_fd, bool serial_mode, const struct sockaddr_in *target_addr,
                                    int sysid, int compid, int target_sys, int target_comp,
                                    const char *param_name, double timeout_sec,
                                    mavlink_param_value_t *out_pv, uint8_t *out_src_sys, uint8_t *out_src_comp) {
    char param_id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN] = {0};
    fill_param_id(param_id, sizeof(param_id), param_name);

    mavlink_message_t msg;
    uint8_t tx[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_param_request_read_pack((uint8_t)sysid, (uint8_t)compid, &msg,
                                        (uint8_t)target_sys, (uint8_t)target_comp, param_id, -1);
    uint16_t tx_len = mavlink_msg_to_send_buffer(tx, &msg);
    (void)send_mavlink_packet(io_fd, serial_mode, target_addr, tx, tx_len);

    double deadline = monotonic_seconds() + timeout_sec;
    while (monotonic_seconds() < deadline) {
        double remaining = deadline - monotonic_seconds();
        if (remaining < 0) remaining = 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(io_fd, &rfds);
        struct timeval tv;
        tv.tv_sec = (int)remaining;
        tv.tv_usec = (suseconds_t)((remaining - tv.tv_sec) * 1e6);

        int ready = select(io_fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (ready == 0) continue;

        unsigned char rx[2048];
        ssize_t n = serial_mode ? read(io_fd, rx, sizeof(rx)) : recvfrom(io_fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n <= 0) continue;

        mavlink_message_t rmsg;
        mavlink_status_t status;
        memset(&status, 0, sizeof(status));
        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &rmsg, &status)) continue;
            if (rmsg.msgid != MAVLINK_MSG_ID_PARAM_VALUE) continue;

            mavlink_param_value_t pv;
            mavlink_msg_param_value_decode(&rmsg, &pv);
            char name[17];
            memcpy(name, pv.param_id, 16);
            name[16] = '\0';
            if (strcmp(name, param_name) != 0) continue;

            if (out_pv) *out_pv = pv;
            if (out_src_sys) *out_src_sys = rmsg.sysid;
            if (out_src_comp) *out_src_comp = rmsg.compid;
            return 1;
        }
    }
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s <name> [options]\n"
        "   or: %s -n <name> [options]\n"
        "Options:\n"
        "  -n <name>   : parameter name (optional if positional <name> used)\n"
        "  -l <ip>     : listen IP (default %s)\n"
        "  -q <port>   : listen port (default %d)\n"
        "  -u <ip>     : target IP (default %s)\n"
        "  -p <port>   : target port (default %d)\n"
        "  -t <sysid>  : target sysid (default %d)\n"
        "  -k <compid> : target compid (default %d)\n"
        "  -S <dev>    : serial device (ex: /dev/ttyAMA0)\n"
        "  -B <baud>   : serial baud (default %d)\n"
        "  -s <sysid>  : sender sysid (default %d)\n"
        "  -c <compid> : sender compid (default %d)\n"
        "  -w <value>  : write value then verify\n"
        "  -d <sec>    : timeout per try (default %.1f)\n"
        "  -r <count>  : retries (default %d)\n",
        prog, prog,
        DEFAULT_LISTEN_IP,
        DEFAULT_LISTEN_PORT,
        DEFAULT_TARGET_IP,
        DEFAULT_TARGET_PORT,
        DEFAULT_TARGET_SYS,
        DEFAULT_TARGET_COMP,
        DEFAULT_SYSID,
        DEFAULT_COMPID,
        DEFAULT_SERIAL_BAUD,
        DEFAULT_TIMEOUT_SEC,
        DEFAULT_RETRIES);
}

int main(int argc, char **argv) {
    const char *listen_ip = mav_cfg_get_str(MAV_CFG_KEY_SCAN_LISTEN_IP, DEFAULT_LISTEN_IP);
    int listen_port = mav_cfg_get_int(MAV_CFG_KEY_SCAN_LISTEN_PORT, DEFAULT_LISTEN_PORT);
    const char *target_ip = mav_cfg_get_str(MAV_CFG_KEY_SCAN_TARGET_IP, DEFAULT_TARGET_IP);
    int target_port = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_PORT, DEFAULT_TARGET_PORT);
    int target_sys = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_SYS, DEFAULT_TARGET_SYS);
    int target_comp = mav_cfg_get_int(MAV_CFG_KEY_SCAN_TARGET_COMP, DEFAULT_TARGET_COMP);
    int sysid = mav_cfg_get_int(MAV_CFG_KEY_SCAN_SYSID, DEFAULT_SYSID);
    int compid = mav_cfg_get_int(MAV_CFG_KEY_SCAN_COMPID, DEFAULT_COMPID);
    const char *param_name = NULL;
    double timeout_sec = mav_cfg_get_double(MAV_CFG_KEY_PARAM_FETCH_TIMEOUT_SEC, DEFAULT_TIMEOUT_SEC);
    int retries = mav_cfg_get_int(MAV_CFG_KEY_PARAM_FETCH_RETRIES, DEFAULT_RETRIES);
    int write_mode = 0;
    double write_value = 0.0;
    const char *serial_device = NULL;
    int serial_baud = mav_cfg_get_int(MAV_CFG_KEY_SCAN_SERIAL_BAUD, DEFAULT_SERIAL_BAUD);

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            param_name = argv[++i];
        } else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            listen_ip = argv[++i];
        } else if (strcmp(argv[i], "-q") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            target_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            target_sys = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            target_comp = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-S") == 0 && i + 1 < argc) {
            serial_device = argv[++i];
        } else if (strcmp(argv[i], "-B") == 0 && i + 1 < argc) {
            serial_baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            sysid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            compid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            write_mode = 1;
            write_value = atof(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            timeout_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            retries = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-' && (!param_name || !param_name[0])) {
            param_name = argv[i];
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (!param_name || !param_name[0]) {
        usage(argv[0]);
        return 1;
    }

    bool serial_mode = (serial_device != NULL);
    int io_fd = -1;
    if (serial_mode) {
        io_fd = open_serial_port(serial_device, serial_baud);
        if (io_fd < 0) {
            fprintf(stderr, "open_serial_port(%s) failed: %s\n", serial_device, strerror(errno));
            return 1;
        }
    } else {
        io_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (io_fd < 0) {
            perror("socket");
            return 1;
        }
        int reuse = 1;
        setsockopt(io_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in listen_addr;
        memset(&listen_addr, 0, sizeof(listen_addr));
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons((uint16_t)listen_port);
        if (inet_pton(AF_INET, listen_ip, &listen_addr.sin_addr) != 1) {
            fprintf(stderr, "Invalid listen IP: %s\n", listen_ip);
            close(io_fd);
            return 1;
        }
        if (bind(io_fd, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) != 0) {
            fprintf(stderr, "bind(%s:%d) failed: %s\n", listen_ip, listen_port, strerror(errno));
            close(io_fd);
            return 1;
        }
    }

    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    if (!serial_mode) {
        target_addr.sin_port = htons((uint16_t)target_port);
        if (inet_pton(AF_INET, target_ip, &target_addr.sin_addr) != 1) {
            fprintf(stderr, "Invalid target IP: %s\n", target_ip);
            close(io_fd);
            return 1;
        }
    }

    printf("%sPARAM%s Fetch single parameter: %s\n", CLR_CYAN, CLR_RESET, param_name);
    if (serial_mode) {
        printf("%sTARGET%s serial:%s@%d (tsys=%d tcomp=%d)\n",
               CLR_YELLOW, CLR_RESET, serial_device, serial_baud, target_sys, target_comp);
    } else {
        printf("%sTARGET%s %s:%d (tsys=%d tcomp=%d)\n", CLR_YELLOW, CLR_RESET, target_ip, target_port, target_sys, target_comp);
    }

    mavlink_param_value_t pv = {0};
    uint8_t src_sys = 0, src_comp = 0;
    int received = 0;

    if (!write_mode) {
        for (int attempt = 1; attempt <= retries; ++attempt) {
            if (request_param_value_once(io_fd, serial_mode, &target_addr,
                                         sysid, compid, target_sys, target_comp,
                                         param_name, timeout_sec, &pv, &src_sys, &src_comp)) {
                received = 1;
                break;
            }
            printf("%sRetry%s %d/%d\n", CLR_YELLOW, CLR_RESET, attempt, retries);
        }
        if (!received) {
            fprintf(stderr, "%sFailed%s to fetch %s\n", CLR_RED, CLR_RESET, param_name);
            close(io_fd);
            return 1;
        }
        char formatted[64];
        format_param_value(pv.param_type, pv.param_value, formatted, sizeof(formatted));
        printf("%sOK%s %s = %s (type=%u) from tsys=%u tcomp=%u\n",
               CLR_GREEN, CLR_RESET, param_name, formatted, (unsigned)pv.param_type, src_sys, src_comp);
    } else {
        if (!request_param_value_once(io_fd, serial_mode, &target_addr,
                                      sysid, compid, target_sys, target_comp,
                                      param_name, timeout_sec, &pv, &src_sys, &src_comp)) {
            fprintf(stderr, "%sFailed%s to read current value for %s before write\n",
                    CLR_RED, CLR_RESET, param_name);
            close(io_fd);
            return 1;
        }
        char cur[64];
        format_param_value(pv.param_type, pv.param_value, cur, sizeof(cur));
        printf("%sCUR%s %s = %s (type=%u)\n", CLR_CYAN, CLR_RESET, param_name, cur, (unsigned)pv.param_type);

        char param_id[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN] = {0};
        fill_param_id(param_id, sizeof(param_id), param_name);
        for (int attempt = 1; attempt <= retries; ++attempt) {
            mavlink_message_t tx_msg;
            uint8_t tx[MAVLINK_MAX_PACKET_LEN];
            float encoded = encode_param_value(pv.param_type, write_value);
            mavlink_msg_param_set_pack((uint8_t)sysid, (uint8_t)compid, &tx_msg,
                                       (uint8_t)target_sys, (uint8_t)target_comp,
                                       param_id, encoded, pv.param_type);
            uint16_t tx_len = mavlink_msg_to_send_buffer(tx, &tx_msg);
            (void)send_mavlink_packet(io_fd, serial_mode, &target_addr, tx, tx_len);

            mavlink_param_value_t verify = {0};
            uint8_t verify_sys = 0, verify_comp = 0;
            if (request_param_value_once(io_fd, serial_mode, &target_addr,
                                         sysid, compid, target_sys, target_comp,
                                         param_name, timeout_sec, &verify, &verify_sys, &verify_comp) &&
                value_matches(verify.param_type, verify.param_value, write_value)) {
                char formatted[64];
                format_param_value(verify.param_type, verify.param_value, formatted, sizeof(formatted));
                printf("%sOK%s %s = %s (type=%u) from tsys=%u tcomp=%u\n",
                       CLR_GREEN, CLR_RESET, param_name, formatted,
                       (unsigned)verify.param_type, verify_sys, verify_comp);
                received = 1;
                break;
            }
            printf("%sRetry%s write %d/%d\n", CLR_YELLOW, CLR_RESET, attempt, retries);
        }

        if (!received) {
            fprintf(stderr, "%sFailed%s to set %s to %.9g\n",
                    CLR_RED, CLR_RESET, param_name, write_value);
            close(io_fd);
            return 1;
        }
    }

    close(io_fd);
    return 0;
}
