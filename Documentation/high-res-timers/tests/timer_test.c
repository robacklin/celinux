/*
 *   Copyright (C) 2002  MontaVista Software
 *                       George Anzinger (george@mvista.com)
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
#include <errno.h>
#include <signal.h>
// #include <assert.h>
#include <sys/utsname.h>
#ifdef __linux__
#include <posix_time.h>
#endif
#include "utils.h"


#define MAX_NOF_TIMERS 6000

int timer_id_in_sival_int = 0;

struct timespec ref;

static void reset_ref_time(void)
{
  clock_gettime(CLOCK_REALTIME, &ref);
}

static void print_rel_time(void)
{
  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  now.tv_sec -= ref.tv_sec;
  now.tv_nsec -= ref.tv_nsec;
  if (now.tv_nsec < 0) {
    now.tv_sec--;
    now.tv_nsec += 1000000000;
  }
  printf("%ld.%09ld", now.tv_sec, now.tv_nsec);
}

int sival_expected;

static void print_siginfo(int signo, siginfo_t *info)
{
  int overrun;

  print_rel_time();
  printf(": siginfo dump: ");
  printf("si_signo: %d [%d], ", info->si_signo, signo);
  printf("si_errno: %d, ", info->si_errno);
  printf("si_value: int=%d ptr=%p ",
	 info->si_value.sival_int, info->si_value.sival_ptr);
  if (sival_expected != -1) {
    assert(info->si_value.sival_int == sival_expected);
  }
  switch (info->si_code) {
  case SI_USER:
    printf("si_code: SI_USER");
    printf(", si_pid: %d", info->si_pid);
    printf(", si_uid: %d\n", info->si_uid);
    break;
  case SI_QUEUE:
    printf("si_code: SI_QUEUE");
    printf(", si_pid: %d", info->si_pid);
    printf(", si_uid: %d\n", info->si_uid);
    break;
  case SI_ASYNCIO:
    printf("si_code: SI_ASYNCIO\n");
    break;
  case SI_TIMER:
#if 0
    t = (timer_id_in_sival_int) ? info->si_value.sival_int : *(timer_t*)info->si_value.sival_ptr;
#endif
    printf("si_code: SI_TIMER");
#ifdef __linux__
    printf(", timer id: %d", info->si_tid);
#endif
    overrun = timer_getoverrun(info->si_tid);
    switch (overrun) {
    case -1:
      printf(", timer_getoverrun() failed: %s\n", strerror(errno));
      break;
    case 0:
      printf("\n");
      break;
    default:
      printf(", overrun: %d\n", overrun);
      break;
    }
    break;
  case SI_MESGQ:
    printf("si_code: SI_MESGQ\n");
    break;
  default:
    printf("si_code: %d\n", info->si_code);
    break;
  }
}

void alrm_handler(int signo, siginfo_t *info, void *context)
{
	print_siginfo(signo, info);
}

int main(void)
{
	int retval;
	int signo;
	timer_t t = 0;
	int i,j;
	timer_t tt[MAX_NOF_TIMERS];
	struct itimerspec ispec;
	struct itimerspec ospec;
	struct sigevent timer_event_spec;
	struct sigaction sa;
	union sigval val;
	sigset_t set;
	siginfo_t info;
	struct utsname utsname;

	sigemptyset(&set);
	sigprocmask(SIG_SETMASK, &set, NULL);

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

	if (sigaction(SIGRTMIN + 1, &sa, NULL)) {
		perror("sigaction failed");
		exit(1);
	}

	printf("\ntest 1: delete non existing timer: expect failure\n");
	retval = timer_delete(t);
	if (retval) {
		perror("timer_delete(bogus timer) failed");
	}
	assert(retval == -1);

	printf("\ntest 2: create default timer\n");
	retval = timer_create(CLOCK_REALTIME, NULL, &t);
	if (retval) {
		perror("timer_create(CLOCK_REALTIME) failed");
	}
	assert(retval == 0);

	printf("\ntest 3: delete timer\n");
	retval = timer_delete(t);
	if (retval) {
		perror("timer_delete(existing timer) failed");
	}
	assert(retval == 0);

	printf("\ntest 4: delete timer again: expect failure\n");
	retval = timer_delete(t);
	if (retval) {
		perror("timer_delete(deleted timer) failed");
	}
	assert(retval == -1);

	printf("\ntest 5: delete non-existing timer: expect failure\n");
	retval = timer_delete(-1);
	if (retval) {
		perror("timer_delete(-1) failed");
	}
	assert(retval == -1);

	printf("\ntest 6: delete non-existing timer: expect failure\n");
	retval = timer_delete(100);
	if (retval) {
		perror("timer_delete(100) failed");
	}
	assert(retval == -1);

	printf("\ntest 7: attempt to create too many timers: expect failure at timer %d\n", MAX_NOF_TIMERS);
	for (i = 0; i < MAX_NOF_TIMERS; i++) {
	  	retval = timer_create(CLOCK_REALTIME, NULL, &tt[i]);
		if (retval) {
			fprintf(stderr, "timer_create(CLOCK_REALTIME) %d failed: %s\n", i, strerror(errno));
			break;
		}
	}
        if ( i < MAX_NOF_TIMERS) {
                assert((i > 30) && (retval == -1));
                j = i;
        }else {
                assert( (i == MAX_NOF_TIMERS) && (retval != -1));
                fprintf(stderr, "timer_create(CLOCK_REALTIME) %d timers created\n",i);
                j = i ;
        }

	printf("\ntest 8: delete these timers: expect failure at timer %d\n", j);
	for (i = 0; i <= j; i++) {
	  	retval = timer_delete(tt[i]);
		if (retval) {
			fprintf(stderr, "timer_delete(CLOCK_REALTIME) %d failed: %s\n", i, strerror(errno));
			break;
		}
	}
	assert((i == j) && (retval == -1));

	printf("\ntest 9: create default timer\n");
	retval = timer_create(CLOCK_REALTIME, NULL, &t);
	if (retval) {
		perror("timer_create(CLOCK_REALTIME) failed");
	}
	assert(retval == 0);

	printf("\ntest 10: set absolute time (no time specification): expect failure\n");
	retval = timer_settime(t, TIMER_ABSTIME, NULL, NULL);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME) failed");
	}
	assert(retval == -1);

	printf("\ntest 11: set relative time (no time specification): expect failure\n");
	retval = timer_settime(t, 0, NULL, NULL);
	if (retval) {
		perror("timer_settime(0) failed");
	}
	assert(retval == -1);

	printf("\ntest 12: set absolute time (bogus time specification): expect failure\n");
	retval = timer_settime(t, TIMER_ABSTIME, (struct itimerspec*)1, NULL);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME, 1, 0) failed");
	}
	assert(retval == -1);

	printf("\ntest 13: set absolute time (bogus time specification): expect failure\n");
	retval = timer_settime(t, TIMER_ABSTIME, NULL, (struct itimerspec*)1);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME, 0, 1) failed");
	}
	assert(retval == -1);

	printf("\ntest 14: set absolute time (bogus time specification): expect failure\n");
	retval = timer_settime(t, TIMER_ABSTIME, (struct itimerspec*)1, (struct itimerspec*)1);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME, 1, 1) failed");
	}
	assert(retval == -1);

	printf("\ntest 15: set relative time (bogus time specification): expect failure\n");
	retval = timer_settime(t, 0, (struct itimerspec*)1, NULL);
	if (retval) {
		perror("timer_settime(0, 1, 0) failed");
	}
	assert(retval == -1);

	printf("\ntest 16: set relative time (bogus time specification): expect failure\n");
	retval = timer_settime(t, 0, NULL, (struct itimerspec*)1);
	if (retval) {
		perror("timer_settime(0, 0, 1) failed");
	}
	assert(retval == -1);

	printf("\ntest 17: set relative time (bogus time specification): expect failure\n");
	retval = timer_settime(t, 0, (struct itimerspec*)1, (struct itimerspec*)1);
	if (retval) {
		perror("timer_settime(0, 1, 1) failed");
	}
	assert(retval == -1);

	retval = clock_gettime(CLOCK_REALTIME, &ispec.it_value);
	if (retval) {
		perror("clock_gettime(CLOCK_REALTIME) failed");
	}
	assert(retval == 0);
	ispec.it_value.tv_sec += 2;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 0;
	reset_ref_time();

	printf("\ntest 18: set timer (absolute time) 2 seconds in the future\n");
	retval = timer_settime(t, TIMER_ABSTIME, &ispec, &ospec);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME) failed");
	}
	assert(retval == 0);
	printf("timer_settime: old setting value=%ld.%09ld, interval=%ld.%09ld\n",
	       ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
	       ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	reset_ref_time();

	printf("\ntest 19: set timer (absolute time) same time\n");
	retval = timer_settime(t, TIMER_ABSTIME, &ispec, &ospec);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME) failed");
	}
	assert(retval == 0);
	printf("timer_settime: old setting value=%ld.%09ld, interval=%ld.%09ld\n",
	       ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
	       ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	printf("\ntest 20: timer_gettime bogus timer id (-1): expect failure\n");
	retval = timer_gettime(-1, NULL);
	if (retval) {
		perror("timer_gettime() failed");
	}
	assert(retval == -1);

	printf("\ntest 21: timer_gettime good timer id, NULL timespec pointer: expect failure\n");
	retval = timer_gettime(t, NULL);
	if (retval) {
		perror("timer_gettime() failed");
	}
	assert(retval == -1);

	printf("\ntest 22: timer_gettime good timer id, bogus timespec pointer: expect failure\n");
	retval = timer_gettime(t, (struct itimerspec*)1);
	if (retval) {
		perror("timer_gettime() failed");
	}
	assert(retval == -1);

	printf("\ntest 23: timer_gettime good timer id, good timespec pointer\n");
	retval = timer_gettime(t, &ispec);
	if (retval) {
		perror("timer_gettime() failed");
	}
	assert(retval == 0);
	printf("timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);

	printf("\ntest 24: send ALRM signal to self with kill()\n");
	reset_ref_time();
	sival_expected = -1;
	retval = kill(getpid(), SIGALRM);
	if (retval) {
		perror("kill(myself with SIGALRM) failed");
	}
	assert(retval == 0);
	
	printf("\ntest 25: send ALRM signal to self with sigqueue()\n");
	reset_ref_time();
	sival_expected = val.sival_int = 4;
	retval = sigqueue(getpid(), SIGALRM, val);
	if (retval) {
		perror("sigqueue(myself with SIGALRM) failed");
	}
	assert(retval == 0);

	sigemptyset(&set);
	sigaddset(&set, SIGALRM);
	sigprocmask(SIG_BLOCK, &set, NULL);
	
	printf("\ntest 26: send ALRM signal to self with kill() (signal blocked)\n");
	retval = kill(getpid(), SIGALRM);
	if (retval) {
		perror("kill(myself with SIGALRM) failed");
	}
	assert(retval == 0);

	printf("\ntest 27: wait for ALRM signal with info\n");
	reset_ref_time();
	info.si_uid = 1;
	sival_expected = -1;
	signo = sigwaitinfo(&set, &info);
	print_siginfo(signo, &info);
	
	printf("\ntest 28: send ALRM signal to self with sigqueue() (signal blocked)\n");
	sival_expected = val.sival_int = 4;
	retval = sigqueue(getpid(), SIGALRM, val);
	if (retval) {
		perror("sigqueue(myself with SIGALRM) failed");
	}
	assert(retval == 0);
	uname(&utsname);
	if (strncmp(utsname.release, "2.3", 3) <= 0) {
	  printf("\nLinux <= 2.3 does not carry siginfo data for SIGALRM\n");
	  sival_expected = -1;
	}

	printf("\ntest 29: wait for ALRM signal with info\n");
	reset_ref_time();
	info.si_uid = 1;
	signo = sigwaitinfo(&set, &info);
	print_siginfo(signo, &info);
	sigprocmask(SIG_UNBLOCK, &set, NULL);

	printf("\ntest 30: timer_gettime()\n");
	sleep(1);
	retval = timer_gettime(t, &ispec);
	if (retval) {
		perror("timer_gettime() failed");
	}
	assert(retval == 0);
	printf("timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);

	sival_expected = -1;
	sleep(1);		/* wait for timer expiration of test 18 */
	
	printf("\ntest 31: timer_delete()\n");
	retval = timer_delete(t);
	if (retval) {
		perror("timer_delete(deleted timer) failed");
	}
	assert(retval == 0);

	printf("\ntest 32: timer_gettime: deleted timer and NULL itimer_spec: expect failure\n");
	retval = timer_gettime(t, NULL);
	if (retval) {
		perror("timer_gettime() failed");
	}
	assert(retval == -1);

	/*
	 * test to check timer cancellation by deletion
	 */

	printf("\ntest 33: create default timer\n");
	retval = timer_create(CLOCK_REALTIME, NULL, &t);
	if (retval) {
		perror("timer_create(CLOCK_REALTIME) failed");
	}

	ispec.it_value.tv_sec = 2;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 0;
	reset_ref_time();

	retval = timer_settime(t, 0, &ispec, &ospec);
	if (retval) {
		perror("timer_settime() failed");
	}
	printf("timer_settime: old setting value=%ld.%09ld, interval=%ld.%09ld\n",
	       ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
	       ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	printf("delete the timer\n");
	retval = timer_delete(t);
	if (retval) {
		perror("timer_delete(deleted timer) failed");
	}
	printf("wait 3 seconds\n");
	sleep(3);
	printf("no timer expiration expected\n");

	/*
	 * test to check relative timer expirations
	 */

	printf("\nTest 34: Expiration in one second with relative timer\n");

	retval = timer_create(CLOCK_REALTIME, NULL, &t);
	if (retval) {
		perror("timer_create(CLOCK_REALTIME) failed");
	}

	ispec.it_value.tv_sec = 1;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 0;
	reset_ref_time();
	retval = timer_settime(t, 0, &ispec, &ospec);
	if (retval) {
		perror("timer_settime() failed");
	}

	printf("timer_settime: old setting value=%ld.%09ld, interval=%ld.%09ld\n",
	       ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
	       ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	retval = timer_gettime(t, &ispec);
	if (retval) {
		perror("timer_gettime() failed");
	}
	printf("timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	
	printf("waiting for signal to arrive...\n");
	sleep(2);

	retval = timer_gettime(t, &ispec);
	if (retval) {
		perror("timer_gettime() failed");
	}

	printf("timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	
	reset_ref_time();
	retval = timer_settime(t, 0, &ispec, &ospec);
	if (retval) {
		perror("timer_settime() failed");
	}

	printf("timer_settime: old setting value=%ld.%09ld, interval=%ld.%09ld\n",
	       ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
	       ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	retval = timer_gettime(t, &ispec);
	if (retval) {
		perror("timer_gettime() failed");
	}
	printf("timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	timer_delete(t);

	/*
	 * Test to see if timer goes off immediately if not a future time is
	 * provided with TIMER_ABSTIME 
	 */
	printf("\ntest 35: set up timer to go off immediately, followed by 10 ticks at 10 Hz\n");
	timer_event_spec.sigev_notify = SIGEV_SIGNAL;
	timer_event_spec.sigev_signo = SIGRTMIN + 0;
	sival_expected = timer_event_spec.sigev_value.sival_int = 0x1234;
	retval = timer_create(CLOCK_REALTIME, &timer_event_spec, &t);

	retval = clock_gettime(CLOCK_REALTIME, &ispec.it_value);
	if (retval) {
		perror("clock_gettime(CLOCK_REALTIME) failed");
	}
	ispec.it_value.tv_sec -= 1;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 100000000;
	reset_ref_time();
	retval = timer_settime(t, TIMER_ABSTIME, &ispec, &ospec);
	if (retval) {
		perror("timer_settime(TIMER_ABSTIME) failed");
	}
	printf("timer should have expired now\n");
	printf("timer_settime: old setting value=%ld.%09ld, interval=%ld.%09ld\n",
	       ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
	       ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);
	retval = timer_gettime(t, &ispec);
	if (retval) {
		perror("timer_gettime() failed");
	}
	printf("timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	printf("catch 10 signals\n");
	for (i = 0; i < 10; i++) sleep(1);

        by_now();
}
