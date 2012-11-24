#ifndef _ASM_SMP_BALANCE_H
#define _ASM_SMP_BALANCE_H

/*
 * We have an architecture-specific SMP load balancer to improve
 * scheduling behavior on hyperthreaded CPUs.  Since only P4s have
 * HT, we use the code only if CONFIG_MPENTIUM4 is set.
 *
 * Distributions may want to make this unconditional to support all
 * x86 machines on one kernel.  The overhead in the non-P4 case is
 * minimal while the benefit to SMP P4s is probably decent.
 */
#if defined(CONFIG_X86_USE_SMP_BALANCE)

/*
 * Find any idle processor package (i.e. both virtual processors are idle)
 */
static inline int find_idle_package(int this_cpu)
{
	int i;

	this_cpu = cpu_number_map(this_cpu);

	for (i = (this_cpu + 1) % smp_num_cpus;
	     i != this_cpu;
	     i = (i + 1) % smp_num_cpus) {
		int physical = cpu_logical_map(i);
		int sibling = cpu_sibling_map[physical];

		if (idle_cpu(physical) && idle_cpu(sibling))
			return physical;
	}
	return -1;	/* not found */
}

static inline int arch_load_balance(int this_cpu, int idle)
{
	/* Special hack for hyperthreading */
       if (unlikely(smp_num_siblings > 1 && idle && !idle_cpu(cpu_sibling_map[this_cpu]))) {
               int found;
               struct runqueue *rq_target;

               if ((found = find_idle_package(this_cpu)) >= 0 ) {
                       rq_target = cpu_rq(found);
                       resched_task(rq_target->idle);
                       return 1;
               }
       }
       return 0;
}

#else
#define arch_load_balance(x, y)		(0)
#endif

#endif /* _ASM_SMP_BALANCE_H */
