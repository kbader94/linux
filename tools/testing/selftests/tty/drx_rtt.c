// SPDX-License-Identifier: GPL-2.0
/*
 * drx_rtt - TTY direct-RX round-trip latency measurement.
 *
 * Companion userspace program to n_drx_test.ko. Drives an external
 * UART loopback (TX wired to RX) and measures the elapsed time
 * between writing a byte and that byte being delivered to the test
 * line discipline's receive_buf() handler. Run twice — once with
 * --mode=off, once with --mode=on — to isolate the direct-RX
 * latency benefit while every other factor (UART driver, wire
 * propagation, baud, system load) stays constant.
 *
 * See README.md in this directory for wiring and environment setup.
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/tty.h>
#include <sys/ioctl.h>

#include "n_drx_test/n_drx_test_uapi.h"

#define DRX_DEV		"/dev/n_drx_test"
#define DEFAULT_LDISC	29	/* N_DEVELOPMENT */

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [options] /dev/ttyXXX\n"
		"\n"
		"Options:\n"
		"  --mode=off|on        Direct-RX mode (default: off)\n"
		"  --baud=N             Baud rate (default: 19200)\n"
		"  --iterations=N       Measurement iterations (default: 1000)\n"
		"  --warmup=N           Warmup iterations to discard (default: 10)\n"
		"  --ldisc=N            Ldisc number (default: %d / N_DEVELOPMENT)\n"
		"  --output=table|csv   Output format (default: table)\n"
		"  --help               This help\n",
		prog, DEFAULT_LDISC);
}

static speed_t baud_to_speed(int baud)
{
	switch (baud) {
	case 9600:   return B9600;
	case 19200:  return B19200;
	case 38400:  return B38400;
	case 57600:  return B57600;
	case 115200: return B115200;
	default:     return 0;
	}
}

static int configure_termios(int fd, speed_t speed)
{
	struct termios tio;

	if (tcgetattr(fd, &tio) < 0)
		return -1;
	cfmakeraw(&tio);
	cfsetispeed(&tio, speed);
	cfsetospeed(&tio, speed);
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cflag &= ~CRTSCTS;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;
	return tcsetattr(fd, TCSANOW, &tio);
}

static int cmp_u64(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
	return (x > y) - (x < y);
}

static uint64_t pct(uint64_t *arr, size_t n, double p)
{
	size_t idx = (size_t)(p / 100.0 * (double)(n - 1));
	return arr[idx];
}

int main(int argc, char **argv)
{
	const char *mode_str = "off";
	int baud = 19200;
	int iterations = 1000;
	int warmup = 10;
	int ldisc = DEFAULT_LDISC;
	const char *output = "table";
	const char *tty_path;
	int tty_fd, drx_fd;
	unsigned int mode;
	uint8_t txbyte = 0x55;
	uint64_t *deltas;
	int i, c;

	static const struct option opts[] = {
		{ "mode",       required_argument, NULL, 'm' },
		{ "baud",       required_argument, NULL, 'b' },
		{ "iterations", required_argument, NULL, 'i' },
		{ "warmup",     required_argument, NULL, 'w' },
		{ "ldisc",      required_argument, NULL, 'l' },
		{ "output",     required_argument, NULL, 'o' },
		{ "help",       no_argument,       NULL, 'h' },
		{ NULL,         0,                 NULL,  0  },
	};

	while ((c = getopt_long(argc, argv, "m:b:i:w:l:o:h", opts, NULL)) != -1) {
		switch (c) {
		case 'm': mode_str   = optarg; break;
		case 'b': baud       = atoi(optarg); break;
		case 'i': iterations = atoi(optarg); break;
		case 'w': warmup     = atoi(optarg); break;
		case 'l': ldisc      = atoi(optarg); break;
		case 'o': output     = optarg; break;
		case 'h': usage(argv[0]); return 0;
		default:  usage(argv[0]); return 1;
		}
	}

	if (optind >= argc) {
		usage(argv[0]);
		return 1;
	}
	tty_path = argv[optind];

	speed_t speed = baud_to_speed(baud);
	if (!speed) {
		fprintf(stderr, "Unsupported baud: %d\n", baud);
		return 1;
	}

	if (strcmp(mode_str, "on") == 0)
		mode = DRX_TEST_MODE_ON;
	else if (strcmp(mode_str, "off") == 0)
		mode = DRX_TEST_MODE_OFF;
	else {
		fprintf(stderr, "Invalid --mode: %s\n", mode_str);
		return 1;
	}

	tty_fd = open(tty_path, O_RDWR | O_NOCTTY);
	if (tty_fd < 0) {
		perror(tty_path);
		return 1;
	}

	if (configure_termios(tty_fd, speed) < 0) {
		perror("tcsetattr");
		close(tty_fd);
		return 1;
	}

	if (ioctl(tty_fd, TIOCSETD, &ldisc) < 0) {
		perror("TIOCSETD");
		close(tty_fd);
		return 1;
	}

	drx_fd = open(DRX_DEV, O_RDWR);
	if (drx_fd < 0) {
		perror(DRX_DEV);
		goto out_tty;
	}

	if (ioctl(drx_fd, DRX_TEST_IOC_SET_MODE, &mode) < 0) {
		perror("DRX_TEST_IOC_SET_MODE");
		goto out_drx;
	}

	deltas = calloc((size_t)iterations, sizeof(uint64_t));
	if (!deltas) {
		perror("calloc");
		goto out_drx;
	}

	/* Warmup: stabilise caches, scheduler, and FIFO state. */
	for (i = 0; i < warmup; i++) {
		struct drx_test_event ev;

		if (write(tty_fd, &txbyte, 1) != 1 ||
		    tcdrain(tty_fd) < 0 ||
		    read(drx_fd, &ev, sizeof(ev)) != sizeof(ev)) {
			perror("warmup");
			goto out_free;
		}
	}

	if (ioctl(drx_fd, DRX_TEST_IOC_RESET) < 0) {
		perror("DRX_TEST_IOC_RESET");
		goto out_free;
	}

	/* Measurement loop. */
	for (i = 0; i < iterations; i++) {
		struct drx_test_event ev;
		struct timespec t0;
		uint64_t t0_ns;

		clock_gettime(CLOCK_MONOTONIC, &t0);
		if (write(tty_fd, &txbyte, 1) != 1) {
			perror("write");
			goto out_free;
		}
		if (tcdrain(tty_fd) < 0) {
			perror("tcdrain");
			goto out_free;
		}
		if (read(drx_fd, &ev, sizeof(ev)) != sizeof(ev)) {
			perror("read");
			goto out_free;
		}
		t0_ns = (uint64_t)t0.tv_sec * 1000000000ULL +
			(uint64_t)t0.tv_nsec;
		deltas[i] = (ev.rx_ktime_ns > t0_ns) ?
			    (ev.rx_ktime_ns - t0_ns) : 0;
	}

	qsort(deltas, (size_t)iterations, sizeof(uint64_t), cmp_u64);

	{
		uint64_t mn   = deltas[0];
		uint64_t mx   = deltas[iterations - 1];
		uint64_t p10v = pct(deltas, iterations, 10);
		uint64_t p50v = pct(deltas, iterations, 50);
		uint64_t p90v = pct(deltas, iterations, 90);
		uint64_t p99v = pct(deltas, iterations, 99);
		uint64_t sum  = 0;
		uint64_t mean;

		for (i = 0; i < iterations; i++)
			sum += deltas[i];
		mean = sum / (uint64_t)iterations;

		if (strcmp(output, "csv") == 0) {
			printf("mode,iterations,baud,min_us,p10_us,p50_us,p90_us,p99_us,max_us,mean_us\n");
			printf("%s,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
			       mode_str, iterations, baud,
			       mn / 1000.0, p10v / 1000.0, p50v / 1000.0,
			       p90v / 1000.0, p99v / 1000.0, mx / 1000.0,
			       mean / 1000.0);
		} else {
			printf("Mode: %-3s  Iterations: %d  Baud: %d\n",
			       mode_str, iterations, baud);
			printf("Latency (microseconds, write_done -> receive_buf):\n");
			printf("  min:    %10.2f\n", mn / 1000.0);
			printf("  p10:    %10.2f\n", p10v / 1000.0);
			printf("  p50:    %10.2f\n", p50v / 1000.0);
			printf("  p90:    %10.2f\n", p90v / 1000.0);
			printf("  p99:    %10.2f\n", p99v / 1000.0);
			printf("  max:    %10.2f\n", mx / 1000.0);
			printf("  mean:   %10.2f\n", mean / 1000.0);
		}
	}

out_free:
	free(deltas);
out_drx:
	close(drx_fd);
out_tty:
	{
		int n_tty = N_TTY;
		ioctl(tty_fd, TIOCSETD, &n_tty);
	}
	close(tty_fd);
	return 0;
}
