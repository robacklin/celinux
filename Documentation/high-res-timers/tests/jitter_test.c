/*
 *   Copyright (C) 2002  MontaVista Software 
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <errno.h>
#include <signal.h>
#include <assert.h>
#ifdef __linux__
#include <posix_time.h>
#endif

#define NOF_JITTER (1*60*100) /* 1 minute test */

int jitter[NOF_JITTER];
int ijit;
struct timeval start;

void alrm_handler(int signo, siginfo_t *info, void *context)
{
  struct timeval end;
  int delta;
  static int first = 1;

  gettimeofday(&end, NULL);

  delta = (end.tv_sec - start.tv_sec) * 1000000 +
    (end.tv_usec - start.tv_usec);
  delta -= 10000;
  start = end;
  if (first) { first = 0; return; }
  jitter[ijit++] = delta;
}

void print_hist(void)
{
#define HISTSIZE 1000

  int hist[1000 + 1];
  int i;

  printf("jitter[0] = %d\n", jitter[0]);
  memset(hist, 0, sizeof(hist));

  for (i = 1; i < NOF_JITTER; i++) {
    if (jitter[i] >= HISTSIZE/2) {
      printf("sample %d over max hist: %d\n", i, jitter[i]);
      hist[HISTSIZE]++;
    }
    else if (jitter[i] <= -HISTSIZE/2) {
      printf("sample %d under min hist: %d\n", i, jitter[i]);
      hist[0]++;
    }
    else {
      hist[jitter[i] + HISTSIZE/2]++;
    }
  }
  for (i = 1; i < HISTSIZE; i++) {
    if (hist[i]) {
      printf("%d: %d\n", i-HISTSIZE/2, hist[i]);
    }
  }
  printf("HC-: %d\n", hist[0]);
  printf("HC+: %d\n", hist[HISTSIZE]);
}

void print_avg(void)
{
  double sum;
  int i;

  sum = 0;
  for (i = 1; i < NOF_JITTER; i++) {
    sum += jitter[i];
  }

  printf("avg. jitter: %f\n", sum/(NOF_JITTER-1));
}

int main(void)
{
	int retval;
	timer_t t = 0;
	struct itimerspec ispec;
	struct itimerspec ospec;
	struct sigaction sa;
	struct sched_param sched;

	retval = mlockall(MCL_CURRENT|MCL_FUTURE);
	if (retval) {
	  perror("mlockall(MCL_CURRENT|MCL_FUTURE) failed");
	}
	assert(retval == 0);

	sched.sched_priority = 2;
	retval = sched_setscheduler(0, SCHED_FIFO, &sched); 
	if (retval) {
	  perror("sched_setscheduler(SCHED_FIFO)");
	}
	assert(retval == 0);

	sa.sa_sigaction = alrm_handler;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);

	if (sigaction(SIGALRM, &sa, NULL)) {
		perror("sigaction failed");
		exit(1);
	}

	if (sigaction(SIGRTMIN, &sa, NULL)) {
		perror("sigaction failed");
		exit(1);
	}

	retval = timer_create(CLOCK_REALTIME, NULL, &t);
	if (retval) {
		perror("timer_create(CLOCK_REALTIME) failed");
	}
	assert(retval == 0);

	retval = clock_gettime(CLOCK_REALTIME, &ispec.it_value);
	if (retval) {
		perror("clock_gettime(CLOCK_REALTIME) failed");
	}
	ispec.it_value.tv_sec += 1;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 10*1000*1000; /* 100 Hz */

	retval = timer_settime(t, TIMER_ABSTIME, &ispec, &ospec);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME) failed");
	}

	do { pause(); } while (ijit < NOF_JITTER);

	retval = timer_delete(t);
	if (retval) {
		perror("timer_delete(existing timer) failed");
	}
	assert(retval == 0);

	print_hist();

	print_avg();

	return 0;
}
