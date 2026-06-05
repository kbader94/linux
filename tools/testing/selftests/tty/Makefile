# SPDX-License-Identifier: GPL-2.0
CFLAGS = -O2 -Wall -I.
TEST_GEN_PROGS := tty_tstamp_update tty_tiocsti_test drx_rtt
TEST_PROGS := drx_rtt_compare.sh
TEST_PROGS_EXTENDED := README.md
TEST_GEN_MODS_DIR := n_drx_test
LDLIBS += -lcap

include ../lib.mk

# Add libcap for TIOCSTI test
$(OUTPUT)/tty_tiocsti_test: LDLIBS += -lcap
