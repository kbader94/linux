# SPDX-License-Identifier: GPL-2.0
# Makefile for PF_LIN (Local Interconnect Network) selftests
top_srcdir = ../../../../..

CFLAGS += -Wall -Wl,--no-as-needed -O2 -g $(KHDR_INCLUDES)

TEST_GEN_PROGS := \
	lin_sock_test \
	lin_filter_test \
	lin_master_test \
	lin_publish_test \
	lin_sched_validate_test \
	lin_sched_run_test \
	lin_sporadic_test \
	lin_event_test \
	lin_loopback_test \
	lin_wakeup_test \
	lin_err_test \
	lin_lifecycle_test \
	lin_stats_test

TEST_INCLUDES := lin_harness.h

include ../../lib.mk
