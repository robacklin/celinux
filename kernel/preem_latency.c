/*
 * Preemption Latency Measurement Tool
 * Copyright (C) 2001 MontaVista Software Inc.
 *
 * Measures the duration of locks held in the kernel.
 * Useful for pinpointing long-held locks, which we
 * can not preempt during.
 *
 * * 20011220 Robert M. Love
 * 	- add SH arch support
 * 	- tidy things to better support new arches
 * 	- add include/asm/preem_latency.h
 * 	- now synced with 2.4.17
 * 
 * * 20011115 Robert M. Love
 * 	- resync with 2.4.15-pre4 and the latest preempt
 * 	  kernel patch
 * 
 * * 20010923 Robert M. Love
 * 	- remove causes "lowlat" and "check"
 * 	- update for revised preemption patch
 * 	- remove defines for preempt_lock_check and window
 * 
 * * 20010919 Robert M. Love
 * 	whitespace cleanup, remove unneeded code, remove
 * 	ifdefs around latency_cause, etc
 * 
 * * 20010918 Robert M. Love
 * 	update for 2.4.10-pre11 and 2.4.9-ac13
 */

#include <asm/system.h>
#include <asm/current.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#ifdef CONFIG_ARM
#include <asm/mach/irq.h>
#else
#include <linux/irq.h>
#endif
#include <linux/interrupt.h>

/*
 * we need a method of timing.  On i386 this is easy
 * with the TSC.  On other arches we can use their
 * existing timing systems or hack something together.
 *
 * The defines are:
 * 	readclock(x)      put timing value in x (unsigned int)
 * 	readclock_init()  initialize timing
 *
 * readclock must be a monotonic incremental counter.
 */ 
#include <asm/preem_latency.h>

#define NUM_LOG_ENTRY 20
#define NUM_COUNT 2
#define COMMAND_LENGTH 16
#define NUM_INTERRUPT 16

const char * syscallStartFileName = "system_call";
const char * entryCheckFileName = "ret_with_reschedule";
const char * errorCodeStart = "error_code start";
const char * deviceNotAvailableStart = "device_not_availablestart";

extern unsigned long cpu_khz;  /* number of rdtsc ticks per second/1000 */

extern void latency_check(const char *, unsigned, int);
extern void latency_end(const char *, unsigned);

unsigned theSyscallNumber;

#define SfS 0 /* waiting for a preempt off start1 call */
#define SfC 1 /* waiting for a preempt off end call */
#define SfM 2 /* waiting for either of the above */

struct pointId {
	const char * FileName;
	unsigned Address;    
	unsigned FileLine;
	unsigned ProcId;
	char ProcName[COMMAND_LENGTH];
};

/* worst recorded latencies */
struct log_entry {
	unsigned latency;
	int cause;
	int mask;
	unsigned intrCount[NUM_INTERRUPT+1];
	struct pointId start;
	struct pointId end;
};

static unsigned logFlag = 0;	/* 0 - no logging; 1 - logging */
static unsigned panicFlag = 0;	/* set to 1 if something is really wrong */

struct LatencyData {
	int breakCount;		/* count interrupt and iret */
	const char * testName;	/* the test name */
	unsigned syncFlag;	/* for synchro between start and end */
	unsigned numIntrs;	/* total number interrupts & intrs in range*/
	unsigned numCount;	/* layout */
	unsigned numEntry;
	int least;		/* lowest latency we care about */
	unsigned count[NUM_COUNT];
	struct log_entry  entry[NUM_LOG_ENTRY]; /* worst recorded latencies */
	int theCause;
	int mask;
	struct pointId latencyStart;
	unsigned latencyStartCount;
	unsigned startFlag;	/* to debug between start() and start1() */

};

struct LatencyData latencyData[NR_CPUS] = { {0} };

static char *cause_name[] = {
	"Unknown",
	"spin_lock",
	"reacqBKL",
	"BKL",
	"Softirq"
};

#define CAUSE_MAX (sizeof(cause_name)/sizeof(cause_name[0]) - 1)

unsigned numIntr = NUM_INTERRUPT;
unsigned intrCount[NUM_INTERRUPT+1];

/* LOCAL */
#ifdef CONFIG_CLOCK_COUNTS_DOWN
#define CLKDIFF(start,end) ((start) - (end))
#else
#define CLKDIFF(start,end) ((end) - (start))
#endif
/* LOCAL */

#define CLEAR(x) do { \
			int zz; \
			for (zz=0; zz<= NUM_INTERRUPT; zz++) \
			x[zz] = 0; \
		} while(0)

#define COPY(x, y) do { \
			int zz; \
			for (zz=0; zz<= NUM_INTERRUPT; zz++) \
			x[zz] = y[zz]; \
		} while(0)

/* strategy : 
 * 
 * Four points are identified
 *   S : the starting point right after system call, user -> kernel
 *   S1: alternative starting points 
 *      . faults (including device_not_available which does go through 
 *              error_code : label)
 *      . interrupts (all go through do_IRQ)
 *   C : need_resched is checked and schedule() is invoked if flag is set
 *   B : before RESTORE ALL and if control is returnning to user land
 *
 * For S points :
 *      it MUST follow a break point
 *      it initializes timing values for the starting point
 *
 * For S1 points :
 *      if it follows a breakpoint, treat it as an S point
 *      otherwise, ignore it.
 *
 * For C points :
 *      it must follow a C point or a S point
 *      calculate the interval and reset the starting point
 *
 * For B points :
 *      It must follow a C point or an S point
 *      calculate the internval and invalidate the starting point
 *
 */

static void __noinstrument reset_latencyData(void)
{
	int i;
	int cpu;

	for (cpu = 0; cpu < smp_num_cpus; cpu++) {
		latencyData[cpu].numCount = NUM_COUNT;
		latencyData[cpu].numEntry = NUM_LOG_ENTRY;
		latencyData[cpu].startFlag = 0;
		latencyData[cpu].breakCount = 0;
		latencyData[cpu].syncFlag = SfS;
		latencyData[cpu].numIntrs = 0;
		latencyData[cpu].least = 0;
		for (i = 0; i < latencyData[cpu].numCount; 
		     latencyData[cpu].count[i++] = 0);
	}
	return;
}

asmlinkage void __noinstrument latency_start(const char *fname,
					     unsigned lineno, int cause)
{
	struct LatencyData *l = &latencyData[smp_processor_id()];

	/* if we are not logging or we have an error, do nothing */
	if ((logFlag == 0) || (panicFlag != 0))
		return;

	if (l->syncFlag != SfC) {
		readclock(l->latencyStartCount);
		l->startFlag = 1;
		l->mask = 0;
		l->theCause = cause;
		l->syncFlag = SfC;
		l->latencyStart.Address = (int)__builtin_return_address(0);
		l->latencyStart.FileName = fname;
		l->latencyStart.FileLine = lineno;
		l->latencyStart.ProcId = current->pid;
		memcpy(l->latencyStart.ProcName, current->comm, 16);
		CLEAR(intrCount);
        }
}

void __noinstrument latency_cause(int was, int tobe)
{
	struct LatencyData *l = &latencyData[smp_processor_id()];

	if (was == -100) {
		l->mask |= tobe;
		return;
        }

	if (l->theCause == was)
		l->theCause = tobe;
}

void __noinstrument latency_logentry(unsigned diff, const char *fname,
				     unsigned lineno, unsigned Address,
				     int mask)
{
	struct LatencyData *l = &latencyData[smp_processor_id()];
	unsigned lowest = 0xffffffff;
	unsigned lowestIndex = 0;
	unsigned sameIndex = 0xffffffff;
	int i;

	if (diff < l->least) 
		return;

	/* check if we need to log this event */
	for (i = 0; i < NUM_LOG_ENTRY; i++) {
		if (lowest > l->entry[i].latency) {
			lowest = l->entry[i].latency;
			lowestIndex = i;
		}
		/* If the call addresses match, it is the same path */
		if ((Address == l->entry[i].end.Address) &&
		 (l->latencyStart.Address == l->entry[i].start.Address)){
			sameIndex = i;
			break;
		}
	}

	if (sameIndex == 0xffffffff)  {
		i = lowestIndex;
		/*
		 * we would like "least" to be the smallest latency in the table
		 * but we are pressed for time so we settle for it being what 
		 * may have been the lowest, as this new entry may replace 
		 * the found lowest with a higher value, in which case we don't 
		 * have the true lowest.  Oh well, we  get closer as time 
		 * passes.  The real reason for this is to speed things up, 
		 * so we don't worry too much if this isn't exact.
		 */
		l->least = l->entry[i].latency;
	} else {
		i = sameIndex;
	}

	if (diff > l->entry[i].latency) {
		l->entry[i].latency = diff;
		l->entry[i].cause = l->theCause; 
		l->entry[i].mask = l->mask; 

		l->entry[i].end.FileName = fname;
		l->entry[i].end.FileLine = lineno;
		l->entry[i].end.Address = Address;
		l->entry[i].end.ProcId = current->pid;
		memcpy(l->entry[i].end.ProcName,
			current->comm, COMMAND_LENGTH);

		l->entry[i].start.FileName = l->latencyStart.FileName;
		l->entry[i].start.FileLine = l->latencyStart.FileLine;
		l->entry[i].start.Address = l->latencyStart.Address; 
		l->entry[i].start.ProcId = l->latencyStart.ProcId;
		memcpy(l->entry[i].start.ProcName, 
			l->latencyStart.ProcName, COMMAND_LENGTH);
	}

	l->numIntrs++;
}

/* Called at end of preemption time */
asmlinkage void __noinstrument latency_end(const char *fname, unsigned lineno)
{
	struct LatencyData *l = &latencyData[smp_processor_id()];
	unsigned endCount;

	/* if we are not logging or we have an error, do nothing */
	if ((logFlag == 0) || (panicFlag != 0)) {
		return;
	}

	/* read entry again */
	readclock(endCount);

	if (l->syncFlag == SfS) {
		l->count[SfS] ++;
		return;
	}

	/* LOCAL */
	latency_logentry(CLKDIFF(l->latencyStartCount, endCount), fname, lineno,
		(int)__builtin_return_address(0), l->mask);
	/* LOCAL */
	l->syncFlag = SfS;
	return;
}

/* latency_check is for the end of one period and the start of another */
asmlinkage void __noinstrument latency_check(const char *fname,
					     unsigned lineno, int cause)
{
	struct LatencyData *l = &latencyData[smp_processor_id()];
	unsigned endCount;

	/* if we are not logging or we have an error, do nothing */
	if ((logFlag == 0) || (panicFlag != 0)) {
		return;
	}
	/* read entry again */
	readclock(endCount);

	if (l->syncFlag != SfS) {
		/* LOCAL */
		latency_logentry(CLKDIFF(l->latencyStartCount, endCount), fname,
			lineno, (int)__builtin_return_address(0), l->mask);
		/* LOCAL */
	}

	/* re-set the starting point */
	l->syncFlag = SfM;
	l->mask = 0;
	l->theCause = cause;
	l->latencyStart.Address = (int) __builtin_return_address(0);
	l->latencyStart.FileName = fname;
	l->latencyStart.FileLine = lineno;
	l->latencyStart.ProcId = current->pid;
	memcpy(l->latencyStart.ProcName, current->comm, 16);
	CLEAR(intrCount);
	readclock(l->latencyStartCount);
}

/* Note following code to read /proc file is not SMP-safe. */

#define HEAD_LINES 2
#define HEAD1 (1 - HEAD_LINES)
#define HEAD2 (2 - HEAD_LINES)
static int g_read_completed = 0;
static int g_cpu = 0;
static char g_buff[84];
static int g_buffidx = 0;
static int g_buf_end = 0;

static int __noinstrument latencytimes_read_proc(char *page_buffer,
						 char **my_first_byte,
						 off_t virtual_start,
						 int length,
						 int *eof,
						 void *data)
{
	int my_buffer_offset = 0;
	char * const my_base = page_buffer;
	int i,j,max;
	struct LatencyData *l;

	*my_first_byte = page_buffer;

	if (virtual_start == 0){
	/* Just been opened */
		logFlag = 0;  /* stop logging */
		g_read_completed = HEAD1;
		g_cpu = 0;
		g_buffidx = 0;
		g_buf_end = 0;
	} else if ((i = g_buf_end - g_buffidx) > 0){
		if (i > length)
			i = length;
		memcpy(my_base, &g_buff[g_buffidx], i);
		g_buffidx += i;
		return i;
	} else if (g_read_completed == NUM_LOG_ENTRY) {
		if (++g_cpu >= smp_num_cpus) {
			*eof = 1;
			reset_latencyData();
			logFlag = 1;  /* start logging */
			return 0;
		}

		g_read_completed = HEAD1;
	}

	g_buffidx = 0;
	l = &latencyData[g_cpu];

	switch (g_read_completed) {
	case HEAD1:
		g_buf_end = sprintf(&g_buff[0],
				    "cpu %d worst %d latency times of %d measured in this period.\n",
				    g_cpu,NUM_LOG_ENTRY,l->numIntrs);
		break;
	case HEAD2:
		g_buf_end = sprintf(&g_buff[0],
				    "  usec      cause     mask   start line/file      address   end line/file\n");
		break;
	default:
		for (j = max = i = 0;j < NUM_LOG_ENTRY; j++){
			if( l->entry[j].latency > max ){
				max = l->entry[j].latency;
				i = j;
			}
		}
		g_buf_end = sprintf(&g_buff[0],
				    "%6d %10s %8x %5d/%-15s %8x %5d/%s\n",
				    (int)clock_to_usecs(l->entry[i].latency),
				    (l->entry[i].cause == -99) ? "unknown" :
				    (l->entry[i].cause < 0) ?
				    irq_desc[~l->entry[i].cause].action->name :
				    (l->entry[i].cause > CAUSE_MAX) ? "Unknown" :
				    cause_name[l->entry[i].cause],
				    l->entry[i].mask,
				    l->entry[i].start.FileLine,
				    l->entry[i].start.FileName == (char *)0 ?
				    "entry.S" : 
				    l->entry[i].start.FileName,
				    l->entry[i].start.Address,
				    /*l->entry[i].start.ProcId,
				      l->entry[i].start.ProcName,*/
				    l->entry[i].end.FileLine,
				    l->entry[i].end.FileName == (char *)0 ?
				    "entry.S" : 
				    l->entry[i].end.FileName /*,
				    l->entry[i].end.ProcId,
				    l->entry[i].end.ProcName*/);

		/* Clearing these two effectivly clears the entry */

		l->entry[i].latency = 0;
		l->entry[i].end.Address = 0;
	}
	g_read_completed++;
	if ((i = g_buf_end) > length)
		i = length;
	memcpy(my_base, &g_buff[0], i);
	g_buffidx += i;
	return i;
}

int __noinstrument __init latencytimes_init(void)
{
	readclock_init();
#ifdef CONFIG_PROC_FS
	create_proc_read_entry("latencytimes", 0, 0, latencytimes_read_proc, 0);
#endif

	reset_latencyData();
	return 0;
}

__initcall(latencytimes_init);

static int detail_cpu = -1;
static int detail_lineno = -1;
static char *detail_file = NULL;
#define DETAILFILEBUFSIZ 64
static char detail_filebuf[DETAILFILEBUFSIZ + 1];

#define DETAILMAXENTRIES 256

static int detail_do = 0;

struct detail_entry {
  int code;
  const char *fname;
  int lineno;
  unsigned rtnaddr;
  int pid;
  int count;
  int intr;
  unsigned clock;
};

struct detail_info {
  unsigned latency;
  int entries;
  struct detail_entry entry[DETAILMAXENTRIES];
};

static struct detail_info detail_info = { 0 };
static struct detail_info detail_report = { 0 };
static int detail_freeze_collect = 1;
static int detail_freeze_report = 1;

static void __noinstrument detail_reset(void)
{
	detail_info.entries = 0;
	detail_info.latency = 0;
	detail_report.entries = 0;
	detail_report.latency = 0;
	detail_freeze_collect = 0;
	detail_freeze_report = 0;
}

static int __noinstrument detail_match(int lineno, char *file, int cpu)
{
	return ((detail_lineno == -1) || (lineno == detail_lineno)) &&
		((detail_cpu == -1) || (cpu == detail_cpu)) &&
		((detail_file == NULL) || (strcmp(file, detail_file) == 0));
}

asmlinkage void __noinstrument latency_detail(int code, void *rtnaddr, const char *fname, 
			       unsigned lineno)
{
	struct detail_entry *e;
	unsigned long flags;

	if (detail_freeze_collect == 1)
		return;

	save_flags(flags);
	cli();

	if (! detail_do && 
	    (((code == 1) &&
	      ((preempt_get_count() & ~PREEMPT_ACTIVE) == 1))
	     || (code == 3)) &&
	    detail_match(lineno, (char *) fname, smp_processor_id())) {
		detail_do = 1;
		readclock(detail_info.latency);
	}

	if (detail_do && detail_match(detail_lineno, detail_file,
				      smp_processor_id())) {
		detail_freeze_collect = 1;
		barrier();

		if (detail_info.entries == DETAILMAXENTRIES)
			e = &detail_info.entry[DETAILMAXENTRIES - 1];
		else {
			e = &detail_info.entry[detail_info.entries];
			detail_info.entries++;
		}

		barrier();
		e->code = code;
		e->fname = fname;
		e->lineno = lineno;
		e->rtnaddr = (int) rtnaddr;
		e->pid = current->pid;
		e->count = preempt_get_count();
		e->intr = in_interrupt();
		readclock(e->clock);

		if (((code == 0) && ((e->count & ~PREEMPT_ACTIVE) == 1)) ||
		    (code == 2) || 
		    ((code == 3) && (detail_info.entries != 0))) {
			detail_info.latency = CLKDIFF(detail_info.latency, e->clock);

			if ((detail_freeze_report == 0) && 
			    (detail_info.latency > detail_report.latency))
				detail_report = detail_info;
		  
			detail_do = 0;
			detail_info.entries = 0;
			barrier();
		}
		
		detail_freeze_collect = 0;
		barrier();
	}

	restore_flags(flags);
	return;
}

static ssize_t __noinstrument detail_read(struct file *file,
			   char *buf,
			   size_t buflen,
			   loff_t *offset)
{
	int bytes = 0;
	static int idet = 0;
	char *activity[] = { "stop  ", "start ", "fstop ", "fstart" };

	if (*offset == 0){
		detail_freeze_collect = 1;
		detail_freeze_report = 1;
		barrier();
		idet = 0;
		bytes = sprintf(buf, "%d entries latency %d usecs:\n", 
				detail_report.entries, 
				(int) clock_to_usecs(detail_report.latency));
	} else if (idet >= detail_report.entries) {
		detail_reset();
		return 0;
	} else {
		bytes = 
		sprintf(buf, "%s %5d/%-25s %8x %6d %5x %d %d\n",
			activity[detail_report.entry[idet].code],
			detail_report.entry[idet].lineno,
			detail_report.entry[idet].fname ?
		        detail_report.entry[idet].fname :
		        "entry.S",
			detail_report.entry[idet].rtnaddr,
			detail_report.entry[idet].pid,
			detail_report.entry[idet].count,
			detail_report.entry[idet].intr,
			idet == 0 ? 0 : 
			    (int)clock_to_usecs(CLKDIFF(detail_report.entry[idet-1].clock,
		 	    detail_report.entry[idet].clock)));
		idet++;
	}
	
	*offset += bytes;
	return bytes;
}

#define WRBUFSIZ 1024
static char wrbuf[WRBUFSIZ];

static ssize_t __noinstrument detail_write(struct file *file,
			    const char *buf,
			    size_t buflen,
			    loff_t *offset)
{
	int wrbuflen = buflen > WRBUFSIZ - 1 ? WRBUFSIZ - 1 : buflen;
	char *cp = NULL;

	memcpy(wrbuf, buf, wrbuflen);
	wrbuf[wrbuflen] = '\0';
	cp = strtok(wrbuf, " \t\n");

	while (cp != NULL) {
		if (strcmp(cp, "line") == 0) {
			cp = strtok(NULL, " \t\n");

			if (cp) {
				detail_lineno = 
					(int) simple_strtoul(cp, NULL, 10);
	
				printk(KERN_INFO 
				       "PREEMPT_TIMES_DETAIL line=%d\n",
				       detail_lineno);
			}
		} else if (strcmp(cp, "cpu") == 0) {
			cp = strtok(NULL, " \t\n");

			if (cp) {
				detail_cpu = 
					(int) simple_strtoul(cp, NULL, 10);
	
				printk(KERN_INFO 
				       "PREEMPT_TIMES_DETAIL cpu=%d\n",
				       detail_cpu);
			}
		} else if (strcmp(cp, "file") == 0) {
			cp = strtok(NULL, " \t\n");

			if (cp) {
				int len = strlen(cp);

				if (len > DETAILFILEBUFSIZ)
					len = DETAILFILEBUFSIZ;

				if (strcmp(cp, "*") == 0) 
					detail_file = NULL;
				else {
					memcpy(detail_filebuf, cp, len);
					detail_filebuf[len] = '\0';
					detail_file = detail_filebuf;
				}
	
				printk(KERN_INFO 
				       "PREEMPT_TIMES_DETAIL file=%s\n",
				       detail_file ? detail_file : "*");
			}
		} else {
			printk(KERN_ERR 
			       "PREEMPT_TIMES_DETAIL invalid key %s\n",
			       cp);
		}

		cp = strtok(NULL," \t\n");
	}

	detail_reset();
	return wrbuflen;
}	

int __init __noinstrument latencydetail_init(void)
{
#ifdef CONFIG_PROC_FS
	static struct file_operations detail_fileops =
	{
		read: detail_read,
		write: detail_write,
	};

	struct proc_dir_entry *entry;

	entry = create_proc_entry("latencydetail", S_IRUGO|S_IWUSR, NULL);

	if (entry)
		entry->proc_fops = &detail_fileops;

#endif

	return 0;
}

__initcall(latencydetail_init);

