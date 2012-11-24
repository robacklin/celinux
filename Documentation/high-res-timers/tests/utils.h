#ifdef OPTION_U
#if OPTION_U == USAGE
#undef OPTION_H
#undef PAR
#undef NIL
#define OPTION_H(shortc,short,long,par,desc) "  -"#short" --"#long par"\t"#desc
#define PAR "\tX"
#define NIL "\t"
#undef OPTION_U 
#elif OPTION_U == LONG
#undef OPTION_H
#undef PAR
#undef NIL
#undef OPTION_U 
#define OPTION_H(shortc,short,long,parm,desc)  { #long,parm,NULL,shortc},
#define PAR 1
#define NIL 0
#elif  OPTION_U == SHORT
#undef OPTION_H
#undef PAR
#undef NIL
#define OPTION_H(shortc,short,long,parm,desc)  #short parm
#define PAR ":"
#define NIL
#undef OPTION_U 
#else
#error "OPTION_U error"
#undef OPTION_U 
#endif
#endif // OPTION_U
#ifndef _UTILS_H
#define _UTILS_H
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sched.h>
#include <assert.h>
#include <pthread.h>  // for sigwait
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <getopt.h>
#define SHORT 1
#define LONG 2
#define USAGE 0

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof(a[0]))

#define IF_HIGH_RES if (({struct timespec ts;  \
                         (clock_getres(CLOCK_REALTIME_HR, &ts) != -1) || \
                         (errno != EINVAL);}))

/*
 * The "wait" set of macros are designed to allow parent and child to wait on
 * one another without spin loop or sleep overhead.  We just use a given signal
 * to pass the wait or not info.  In order to eliminate races, you will want
 * to call "wait_setup(sig)" prior to doing anything that may lead to a signal.
 * "wait_sync()" waits for the signal, and "wait_send(who)" sends the signal.
 * "wait_flush()" flushes out any left over signals.
 */
#define WAIT_VARS() int wait_for_this_sig;sigset_t wait_set;
#define wait_setup(signal) do{wait_for_this_sig = signal;\
                              sigemptyset(&wait_set); \
                              sigaddset(&wait_set, signal); \
                              Try(sigprocmask(SIG_BLOCK, &wait_set, NULL));\
                           } while(0)
#define wait_sync() do{int sig = 0;\
                       while(sig != wait_for_this_sig) { \
                            Try(sigwait(&wait_set,&sig));\
                        }\
                    }while(0)
#define wait_send(who) do{ Try(kill(who,wait_for_this_sig));}while(0)
/*
 * Flush any pending signals, then restore the old action.
 */
#define wait_flush() do{struct sigaction oldact,sigact; \
                        sigact.sa_handler = SIG_IGN; \
                        sigemptyset(&sigact.sa_mask);\
                        Try(sigaction(wait_for_this_sig,&sigact,&oldact));\
                        Try(sigaction(wait_for_this_sig,&oldact,NULL)); \
                     }while(0)
/* 
 * Define some nice color stuff
 */
#define  MOVE_TO_COL(col) "\033["#col"G"
#define      SETCOLOR_SUCCESS "\033[1;32m"
#define      SETCOLOR_FAILURE "\033[1;31m"
#define      SETCOLOR_WARNING "\033[1;33m"
#define      SETCOLOR_NORMAL "\033[0;39m"

#undef __assert_fail
# define __assert_fail(exp,file,line,fun)	\
({fprintf(stderr,SETCOLOR_FAILURE ": %s:%d: %s:assert(%s) failed\n"SETCOLOR_NORMAL,file,line,fun,exp);\
global_error++;})
//#define NSEC_PER_SEC 1000000000                          
#define USEC_PER_SEC	1000000
#define roundtores(t,r) (t)->tv_nsec += (r - 1); \
                        if ((t)->tv_nsec >NSEC_PER_SEC){ \
                           (t)->tv_nsec -= NSEC_PER_SEC;(t)->tv_sec++;}\
                        (t)->tv_nsec -= (t)->tv_nsec % r 
#define timerplusnsec(c,d) (c)->tv_nsec +=(d);             \
                           if ((c)->tv_nsec >NSEC_PER_SEC){ \
                           (c)->tv_nsec -= NSEC_PER_SEC;(c)->tv_sec++;}

#define timerdiff(a,b) ((float)((a)->tv_sec - (b)->tv_sec) + \
                         (float)((a)->tv_nsec - (b)->tv_nsec)/NSEC_PER_SEC)

#define timersum(c,a,b) (c)->tv_sec = ((a)->tv_sec + (b)->tv_sec); \
                       (c)->tv_nsec = ((a)->tv_nsec + (b)->tv_nsec); \
                       if ((c)->tv_nsec > NSEC_PER_SEC){ \
                           (c)->tv_nsec -= NSEC_PER_SEC;(c)->tv_sec++;}
#define timersubtract(c,a,b) (c)->tv_sec = ((a)->tv_sec - (b)->tv_sec); \
                       (c)->tv_nsec = ((a)->tv_nsec - (b)->tv_nsec); \
                       if ((c)->tv_nsec < 0){ \
                           (c)->tv_nsec += NSEC_PER_SEC;(c)->tv_sec--;}
#define timer_gt(a,b) (((a)->tv_sec > (b)->tv_sec) ? 1 :  \
                      (((a)->tv_sec < (b)->tv_sec) ? 0 :  \
                      ((a)->tv_nsec > (b)->tv_nsec)))
#define timeval_to_timespec(a,b) (b)->tv_sec = (a)->tv_sec; \
                                 (b)->tv_nsec = (a)->tv_usec * 1000
#define timespec_to_timeval(a,b) (b)->tv_sec = (a)->tv_sec; \
                                 (b)->tv_usec = (a)->tv_nsec / 1000
#define timevaldiff(a,b) (((a)->tv_sec - (b)->tv_sec)*1000000 + \
                         (a)->tv_usec - (b)->tv_usec)

int global_error;
#define by_now() { char * excolor = (global_error) ? SETCOLOR_FAILURE : \
                         SETCOLOR_WARNING;                     \
                         fprintf(stderr, \
                         "%sEnd of test, %d error(s) found.\n"  \
                         SETCOLOR_NORMAL,                      \
                         excolor,global_error);                       \
                         exit(global_error);}
#ifdef debug
#undef debug
#define debug(a) do {a}while (0)
#else
#define debug(a) do {} while (0)
#endif
#define MASK(x) (1<<(x))
#define long_short(f) (f>0 ? " long" : " short")
/*
 * This is the generic non-errno error header
 */
#define myerror(s) MYerror(__BASE_FILE__,__LINE__,s)
void MYerror(char * where,int line,char *what)
{
        fprintf(stderr,SETCOLOR_FAILURE"%s,%d:%s" SETCOLOR_NORMAL,
                where,line,what);
	fflush(stderr);
	fflush(stdout);
        ++global_error;
}
/*
 * This is an error reporter for errors we don't want to continue after
 */
#define myperror(s) MYperror(__BASE_FILE__,__LINE__,s)
int MYperror(char * where,int line,char *what)
{
        fprintf(stderr,SETCOLOR_FAILURE"%s,%d:",where,line);
        perror(what);
        fprintf(stderr,SETCOLOR_NORMAL);
	fflush(stderr);
	fflush(stdout);
        ++global_error;
        by_now();
}
/*
 * This is a result reporter.  Set e = 0 if the expected value is 0 (no error)
 * Set the expected error otherwise. (For now we don't check errno against e)
 */
#define my_c_perror(e,s) MY_c_perror(e,__BASE_FILE__,__LINE__,s)
int MY_c_perror(int e,char * where,int line,char *what)
{
        char *fmt;
        fmt = e ? "expected" :SETCOLOR_FAILURE"UNEXPECTED";
        fprintf(stderr,"%s %s,%d:",fmt,where,line);
        perror(what);
        if ((e > 0) && (e != errno)){
                errno = e;
                perror(SETCOLOR_FAILURE"Expected");
                global_error++;
                fprintf(stderr,SETCOLOR_NORMAL);
        }
        if ( ! e) {
                if ((e > 0) && (e != errno)){
                        perror("Found   ");
                        errno = e;
                        perror("Expected");
                }
                global_error++;
                fprintf(stderr,SETCOLOR_NORMAL);
        }
	fflush(stderr);
	fflush(stdout);
        return e ? 0:-1;
}
#define no_error(e,s) No_error(e,__BASE_FILE__,__LINE__,s)
int No_error(int e,char * where,int line,char *what)
{
        char *fmt;
        fmt = e ? SETCOLOR_FAILURE"ERROR expected but not found" : "Cool";
        fprintf(stderr,"%s %s,%d:%s\n"SETCOLOR_NORMAL,fmt,where,line,what);
        if ( e) {
                if ((e > 0) && (e != errno)){
                        errno = 0;
                        perror("Found   ");
                        errno = e;
                        perror("Expected");
                }
                global_error++;
                fprintf(stderr,SETCOLOR_NORMAL);
        }
	fflush(stderr);
	fflush(stdout);
        return e ? -1 : 0;
}
/*
 * Try() is a macro for making system calls where an error return is -1.
 * Used as a function, it returns any value other than -1.  Mostly we
 * use it as a straight call, but one never knows.  Also can be called
 * with some expression which will exit the program with an error message
 * if it ever gets to -1.  For example:
 * count = 100;
 * ---loop stuff ---
 * Try (--count);
 * ---end of loop ---
 * Will stop the loop with an error after 101 times around.  Great for 
 * stopping run away while loops.
 * Since it exits via myperror, above, it will report where the exit
 * came from (source and line number).  Error text will be the expression.
 */
#define Try(f) ({int foobas = (f); \
                if( foobas == -1) \
                myperror( #f );   \
                foobas;})
#define try(e,f) ({int foobas = (f); if( foobas == -1){  my_c_perror( e,#f );\
                                     } else{  no_error(e,#f);} foobas;})
/* These two printf() calls take a string with "%d.%03d" somewhere in it
 * and an integer.  They print the interger as a decimal with 3 digits
 * to the right of the decimal point.  Use to print microsecond values
 * as milliseconds.  The second version is used to print two such values.
 */
#define printf1(s, d) printfx(s,d,1000)
void printfx(char *s,int d, int x){printf(s,d/x,d%x);}
void printf2(char *s,int d, int e){ printf(s,d/1000,d%1000,e/1000,e%1000);}
/* Bit map routines.  These routines work on a bit map of N words.  We assume
 * the bit width of a word is 32.
 * Here we only need test, set and clear.  There are others to be found in
 * the bitops.h headers.
 */
#ifdef ASM_BITOPS
#include <asm/bitops.h>
#define BIT_SHIFT 5
#define WORD_MASK (int)(-1<<BIT_SHIFT)
#define BIT_MASK  (~WORD_MASK)
#define clearbit(bit, add) clear_bit((bit)&BIT_MASK,(add)+((bit)>>BIT_SHIFT))
#define setbit(bit, add) set_bit((bit)&BIT_MASK,(add)+((bit)>>BIT_SHIFT))
#define testbit(bit, add) __test_bit((bit)&BIT_MASK,(add)+((bit)>>BIT_SHIFT))
#else
#define BIT_SHIFT 5
#define WORD_MASK (int)(-1<<BIT_SHIFT)
#define BIT_MASK  (~WORD_MASK)
#define clearbit(bit, add) *((add)+((bit)>>BIT_SHIFT)) &= ~(1<<((bit)&BIT_MASK))
#define setbit(bit, add)   *((add)+((bit)>>BIT_SHIFT)) |= 1<<((bit)&BIT_MASK)
#define testbit(bit, add) (*((add)+((bit)>>BIT_SHIFT)) &  1<<((bit)&BIT_MASK))
#endif
#define NO_BITS 10000
#define NO_WDS ((NO_BITS/32)+1)

#endif
