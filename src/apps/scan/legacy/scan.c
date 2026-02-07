// Scan MAVLink messages from a serial port for a fixed duration.

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>

#define DEFAULT_DEVICE "/dev/ttyAMA0"
#define DEFAULT_BAUD 921600
#define DEFAULT_DURATION 5.0
#define DEFAULT_TIMEOUT 0.2

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [--device DEV] [--baud RATE] [--duration SEC] [--timeout SEC]\n",
		prog);
}

static int baud_to_constant(int baud)
{
	switch (baud) {
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 921600:
		return B921600;
	default:
		return -1;
	}
}

static int configure_serial(int fd, int baud)
{
	struct termios tio;

	if (tcgetattr(fd, &tio) != 0) {
		perror("tcgetattr");
		return -1;
	}

	cfmakeraw(&tio);
	int baud_constant = baud_to_constant(baud);
	if (baud_constant < 0) {
		fprintf(stderr, "Unsupported baud rate: %d\n", baud);
		return -1;
	}

	if (cfsetispeed(&tio, baud_constant) != 0 || cfsetospeed(&tio, baud_constant) != 0) {
		perror("cfsetispeed/cfsetospeed");
		return -1;
	}

	tio.c_cflag |= (CLOCAL | CREAD);
	if (tcsetattr(fd, TCSANOW, &tio) != 0) {
		perror("tcsetattr");
		return -1;
	}

	return 0;
}

static int string_in_list(char **list, size_t count, const char *value)
{
	for (size_t i = 0; i < count; ++i) {
		if (strcmp(list[i], value) == 0) {
			return 1;
		}
	}
	return 0;
}

static int compare_strings(const void *a, const void *b)
{
	const char *lhs = *(const char **)a;
	const char *rhs = *(const char **)b;
	return strcmp(lhs, rhs);
}

int main(int argc, char **argv)
{
	const char *device = DEFAULT_DEVICE;
	int baud = DEFAULT_BAUD;
	double duration = DEFAULT_DURATION;
	double timeout = DEFAULT_TIMEOUT;

	static struct option options[] = {
		{"device", required_argument, NULL, 'd'},
		{"baud", required_argument, NULL, 'b'},
		{"duration", required_argument, NULL, 'D'},
		{"timeout", required_argument, NULL, 't'},
		{NULL, 0, NULL, 0},
	};

	int opt;
	while ((opt = getopt_long(argc, argv, "d:b:D:t:", options, NULL)) != -1) {
		switch (opt) {
		case 'd':
			device = optarg;
			break;
		case 'b':
			baud = atoi(optarg);
			break;
		case 'D':
			duration = atof(optarg);
			break;
		case 't':
			timeout = atof(optarg);
			break;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	int fd = open(device, O_RDONLY | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", device, strerror(errno));
		return 1;
	}

	if (configure_serial(fd, baud) != 0) {
		close(fd);
		return 1;
	}

	char **topics = NULL;
	size_t topics_count = 0;

	mavlink_message_t message;
	mavlink_status_t status;
	memset(&status, 0, sizeof(status));

	struct timespec start_time;
	clock_gettime(CLOCK_MONOTONIC, &start_time);

	while (1) {
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		double elapsed = (now.tv_sec - start_time.tv_sec) +
				 (now.tv_nsec - start_time.tv_nsec) / 1e9;
		if (elapsed >= duration) {
			break;
		}

		struct timeval tv;
		tv.tv_sec = (time_t)timeout;
		tv.tv_usec = (suseconds_t)((timeout - tv.tv_sec) * 1e6);

		fd_set read_fds;
		FD_ZERO(&read_fds);
		FD_SET(fd, &read_fds);

		int ready = select(fd + 1, &read_fds, NULL, NULL, &tv);
		if (ready < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("select");
			break;
		}

		if (ready == 0) {
			continue;
		}

		unsigned char buffer[256];
		ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
		if (bytes_read < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("read");
			break;
		}

		for (ssize_t i = 0; i < bytes_read; ++i) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) {
				const char *name = NULL;
#ifdef MAVLINK_GET_MESSAGE_INFO
				const mavlink_message_info_t *info = mavlink_get_message_info(&message);
				name = info ? info->name : NULL;
#endif
				char fallback[32];
				if (name == NULL) {
					snprintf(fallback, sizeof(fallback), "MSG_ID_%u", message.msgid);
					name = fallback;
				}
				if (!string_in_list(topics, topics_count, name)) {
					char **next = realloc(topics, (topics_count + 1) * sizeof(char *));
					if (!next) {
						perror("realloc");
						goto cleanup;
					}
					topics = next;
					topics[topics_count] = strdup(name);
					if (!topics[topics_count]) {
						perror("strdup");
						goto cleanup;
					}
					topics_count++;
				}
			}
		}
	}

	if (topics_count == 0) {
		printf("No MAVLink messages received during scan.\n");
		goto cleanup;
	}

	qsort(topics, topics_count, sizeof(char *), compare_strings);

	printf("MAVLink topics observed:\n");
	for (size_t i = 0; i < topics_count; ++i) {
		printf("- %s\n", topics[i]);
	}

cleanup:
	for (size_t i = 0; i < topics_count; ++i) {
		free(topics[i]);
	}
	free(topics);
	close(fd);

	return topics_count == 0 ? 1 : 0;
}
