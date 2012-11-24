/***********************************************************************
 *
 * Based on:
 *   Copyright 2001 MontaVista Software Inc.
 *   Author: jsun@mvista.com or jsun@junsun.net
 *
 *   arch/mips/ddb5xxx/common/prom.c
 *       prom.c file.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 ***********************************************************************
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/bootmem.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/debug.h>
#include <asm/tcube.h>

char arcs_cmdline[CL_SIZE];

const char *get_system_type(void)
{
    return "SHIMAFUJI T-Cube (VR5701)";
}

/* [jsun@junsun.net] PMON passes arguments in C main() style */
void __init prom_init(int argc, char **argv, char **envp, int *prom_vec)
{
    int i;
	
    /* argv[0] is "g", the rest is boot parameters */
    arcs_cmdline[0] = '\0';
    for (i = 1; i < argc; i++) {
	if (strlen(arcs_cmdline) + strlen(argv[i]) + 1 >= sizeof(arcs_cmdline))
	    break;
	strcat(arcs_cmdline, argv[i]);
	strcat(arcs_cmdline, " ");
    }
    /* by default all these boards use dhcp/nfs root fs */
    /* strcat(arcs_cmdline, "ip=bootp"); */

    mips_machgroup = MACH_GROUP_SHIMA;
    mips_machtype =  MACH_SHIMA_TCUBE;
    add_memory_region(0, TCUBE_SDRAM_SIZE, BOOT_MEM_RAM);
}

void __init prom_free_prom_memory(void)
{
}

int prom_get_para(const char *name,char **pValue)
{
#ifdef KOSE_____________________
    const char *p,*n,*v;

    p=TCUBE_PROM_PARA_ADDRESS;
    while(*p!=0){
	n = p;
	v = p + strlen(n) + 1;
	p = v + strlen(v) + 1;
	if(strcmp(name,n)==0){
	    *pValue = v;
	    return 1;
	}
    }
#endif
    return 0;
}

int prom_get_int(const char *name, int *pVal)
{
    char *v;
  
    if(prom_get_para(name, &v)){
	if(sscanf(v,"%d",pVal)<1)
	    return 0;
	return 1;
    }
    return 0;
}

int prom_get_bool(const char *name, int *pVal)
{
    char *v;
  
    if(prom_get_para(name, &v)){
	if(strcmp(v,"On")==0 ||strcmp(v,"on")==0 || v[0] == '1')
	    *pVal = 1;
	else
	    *pVal = 0;
	return 1;
    }
    return 0;
}

int prom_get_macadr(const char *name,u_char *mac_addr)
{
    char *v;
    unsigned int d[6];
    int i;

    if(prom_get_para(name, &v)){
	if(sscanf(v,"%x-%x-%x-%x-%x-%x",d,d+1,d+2,d+3,d+4,d+5) < 6)
	    return 0;
	for(i=0;i<6;i++)
	    mac_addr[i] = d[i];
	return 1;
    }
    return 0;
}
