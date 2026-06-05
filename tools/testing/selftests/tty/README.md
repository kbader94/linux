# TTY direct-RX RTT measurement

This directory contains a round-trip-time (RTT) measurement harness
for the TTY layer's opt-in direct-RX path
(`tty_port_enable_direct_rx()` / `tty_port_drain_flip_buffer()`).

The point of the harness is to quantify, on real hardware, how much
RX-side latency the direct-RX path removes versus the normal flush
workqueue path — specifically how much of the workqueue's scheduling
delay is eliminated on single-CPU systems under load.

## What's here

| File                       | Purpose                                              |
|----------------------------|------------------------------------------------------|
| `n_drx_test/n_drx_test.c`  | Test-only TTY line discipline (out-of-tree module)   |
| `n_drx_test/Makefile`      | Builds `n_drx_test.ko`                               |
| `n_drx_test/n_drx_test_uapi.h` | ioctl numbers + event struct shared with userspace |
| `drx_rtt.c`                | Userspace measurement program                        |
| `drx_rtt_compare.sh`       | Runs both modes and prints a comparison table        |

## Hardware setup

You need a UART exposed to the running kernel as `/dev/ttyXXX` with
its TX wired externally to its RX. On a DE-9 connector:

```
       UART                         loopback jumper
    .---------.                  .------------------.
    |   TX  o-+------------------+--.               |
    |       |                    |  |               |
    |   RX  o-+------------------+--'               |
    |       |                                       |
    | (GND) o-+--- (optional, only matters off-port) |
    '---------'                  '------------------'
```

On a Raspberry Pi GPIO header: physical pin 8 (UART TX) to physical
pin 10 (UART RX). On a USB-serial adapter with bare wires: TX wire
shorted to RX wire. On an AX99100 PCIe UART card: any matching pair on
the same port, using the supplied DE-9 to header cable.

USB-CDC bridges (`cp210x`, `ft232`, `ch340`, `pl2303`) work as a
sanity-check loopback but the USB packetisation latency dominates over
both paths, so you will not see a useful direct-RX delta. Use a
hardware UART for meaningful numbers.

## Building

In this directory:

```sh
make                        # build drx_rtt
make modules                # build n_drx_test/n_drx_test.ko
```

The `modules` target requires kernel headers; set `KDIR` if not at
`/lib/modules/$(uname -r)/build`.

## Running

Load the module, then run the comparison script with the loopback'd
TTY's path:

```sh
sudo insmod n_drx_test/n_drx_test.ko
sudo ./drx_rtt_compare.sh /dev/ttyS0
```

Sample output (representative; numbers depend on hardware and load):

```
Direct-RX RTT comparison (/dev/ttyS0 @ 19200 baud, 1000 iterations/mode):

Latency (us)    workqueue    direct-RX        delta   improvement
------------  -----------  -----------  -----------  ------------
min                572.00       540.00        32.00     5.6%
p10                580.00       541.00        39.00     6.7%
p50                798.00       542.00       256.00    32.1%
p90              1,242.00       546.00       696.00    56.0%
p99              3,142.00       551.00     2,591.00    82.5%
max              9,810.00       589.00     9,221.00    94.0%
mean               911.00       543.00       368.00    40.4%
```

The fixed component (~540 µs at 19200 baud) is the on-wire time of one
byte plus IRQ-to-`flip_buffer_push` cost; that's what both modes pay
unconditionally. The delta is what direct-RX saves.

The user-mode program also accepts `--output=csv` for machine-readable
output, and you can run it directly without the comparison wrapper:

```sh
./drx_rtt --mode=on --baud=19200 --iterations=5000 /dev/ttyS0
```

## Environment recipes

The harness does not manipulate the system; it only measures. The
following are the recommended environments for showing direct-RX's
effect:

### Single CPU

```sh
# Offline all CPUs except CPU 0:
for n in /sys/devices/system/cpu/cpu[1-9]*/online; do
    echo 0 | sudo tee $n > /dev/null
done

sudo ./drx_rtt_compare.sh /dev/ttyS0
```

### Single CPU under load

```sh
# In another shell (or as a backgrounded job):
stress-ng --cpu 1 --cpu-load 100 &
STRESS_PID=$!

sudo ./drx_rtt_compare.sh /dev/ttyS0

kill $STRESS_PID
```

Direct-RX's largest improvement appears here: the SCHED_FIFO drain
kthread preempts the load, while the workqueue queues behind it.

### Defeat cpuidle wake latency

```sh
sudo sh -c '
    for f in /sys/devices/system/cpu/cpu*/cpuidle/state*/disable; do
        echo 1 > $f
    done
'

sudo ./drx_rtt_compare.sh /dev/ttyS0
```

Disabling cpuidle shortens the cold-CPU wake-up cost. The direct-RX
delta will look smaller because both modes get a faster wake; this is
the case where the kernel patch is *least* needed, and it serves as a
control.

## Limitations

- Measurement granularity is one byte. RTT for a multi-byte burst will
  not scale linearly — at low budget settings the kthread drains the
  whole burst in one call.
- The wire propagation time and the UART's RX FIFO behaviour are
  baked into the floor. To compare across UARTs you also need to
  control for the FIFO trigger threshold (see the FIFO Control patch
  series in the same tree).
- The script does not enforce real-time scheduling on the test
  process itself. If you want to chase the absolute floor, run
  `chrt -f 50` on `drx_rtt`.

## Module parameters

| Param        | Default          | Notes                                              |
|--------------|------------------|----------------------------------------------------|
| `ldisc_num`  | 29 (N_DEVELOPMENT) | TTY line discipline slot to register. Use the matching `--ldisc=N` on `drx_rtt`. |

## ioctl reference

See `n_drx_test/n_drx_test_uapi.h` for the event struct and ioctl
numbers. Usable from userspace by including that header.

```c
#include "n_drx_test/n_drx_test_uapi.h"

unsigned int mode = DRX_TEST_MODE_ON;
ioctl(drx_fd, DRX_TEST_IOC_SET_MODE, &mode);
ioctl(drx_fd, DRX_TEST_IOC_RESET);

struct drx_test_event ev;
read(drx_fd, &ev, sizeof ev);  /* blocks until receive_buf fires */
```
