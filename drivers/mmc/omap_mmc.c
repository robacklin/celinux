/*
 * BRIEF MODULE DESCRIPTION
 * Low-level MMC/SD functions for the OMAP1510 MMC controller
 * Based on: arch/arm/mach-sa1100/h3600_asic_mmc.c
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#include <asm/irq.h>        
#include <asm/unaligned.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/innovator.h>
#include <asm/dma.h>
#include <linux/mmc/mmc_ll.h>

#include <linux/pm.h>

struct response_info {
	int length;
	u16 cdc_flags;
};

enum omap_request_type {
	RT_NO_RESPONSE,
	RT_RESPONSE_ONLY,
	RT_READ,
	RT_WRITE
};

struct omap_mmc_data {
	struct timer_list        sd_detect_timer;

	u32                      clock;        /* Current clock frequency */
        struct mmc_request      *request;      /* Current request */
	enum omap_request_type  type;
        int sd;                                /* 1 - SD card or 0 - MMC card */  

        dma_regs_t              *dma_regs;
        dma_addr_t              buf_dma_phys;
        void *                  buf_dma_virt;

};

static struct omap_mmc_data g_omap_mmc_data;
static u8 card_state = 1 << 4;

#define OMAP_MMC_MASTER_CLOCK 48000000

static __inline__ void mmc_delay( void ) { udelay(1); }

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */
#include <linux/device.h>

static int omap_mmc_dpm_suspend(struct device * dev, u32 state, u32 level);
static int omap_mmc_dpm_resume(struct device * dev, u32 level);

static struct device_driver omap_mmc_driver_ldm = {
       name:      "omap1510-mmc",
       devclass:  NULL,
       probe:     NULL,
       suspend:   omap_mmc_dpm_suspend,
       resume:    omap_mmc_dpm_resume,
       remove:    NULL,
};

static struct device omap_mmc_device_ldm = {
       name: "OMAP1510 MMC-SD",
       bus_id: "MMC-SD",
       driver: NULL,
       power_state: DPM_POWER_ON,
};

static void omap_mmc_ldm_driver_register(void)
{
   extern void mpu_public_driver_register(struct device_driver *driver);

   mpu_public_driver_register(&omap_mmc_driver_ldm);
}

static void omap_mmc_ldm_device_register(void)
{
   extern void mpu_public_device_register(struct device *device);

   mpu_public_device_register(&omap_mmc_device_ldm);
}

static void omap_mmc_ldm_driver_unregister(void)
{
   extern void mpu_public_driver_unregister(struct device_driver *driver);

   mpu_public_driver_unregister(&omap_mmc_driver_ldm);
}

static void omap_mmc_ldm_device_unregister(void)
{
   extern void mpu_public_device_unregister(struct device *device);

   mpu_public_device_unregister(&omap_mmc_device_ldm);
}


#endif /* MVL-CEE */

/**************************************************************************
 *   Clock routines
 **************************************************************************/

static int omap_mmc_stop_clock( void )
{
  /* Clear 7:0 bits of the MMC_CON to disable clock */

        outw(inw(OMAP_MMC_CON) & 0xff00, OMAP_MMC_CON);
        return MMC_NO_ERROR;

}

static int omap_mmc_set_clock( u32 rate )
{
	int retval;
	DEBUG(3, ": Set rate: %d \n", rate);

	u32 master = OMAP_MMC_MASTER_CLOCK;   /* Default master clock */

	u8 divisor = master / rate;
        
	retval = omap_mmc_stop_clock();
	if ( retval )
		return retval;
        
	outw(inw(OMAP_MMC_CON) | divisor, OMAP_MMC_CON);
	mmc_delay();

        /* FIX-ME: Time out values */

	outw(0x90, OMAP_MMC_CTO);
	mmc_delay();
	
	outw(0xffff, OMAP_MMC_DTO) ;
	mmc_delay();

	g_omap_mmc_data.clock = rate;
	
	return MMC_NO_ERROR;
}

static void omap_mmc_set_transfer( u16 block_len, u16 nob )
{
        outw(block_len-1, OMAP_MMC_BLEN); /* Only 10:0 bits are valid */
	outw(nob - 1, OMAP_MMC_NBLK);       /* Only 10:0 bits are valid */
        DEBUG(3, "Set transfer: %x \n", inw(OMAP_MMC_BLEN));
}

static void omap_mmc_start_tx_dma_transfer (int size);
static void omap_mmc_start_rx_dma_transfer (int size);
static void omap_mmc_handle_int( struct omap_mmc_data *sd, u16 status);

static void omap_mmc_rx_dma_callback (void *data, int size)
{

        memcpy(g_omap_mmc_data.request->buffer, g_omap_mmc_data.buf_dma_virt,
               g_omap_mmc_data.request->block_len);

        omap_free_dma(g_omap_mmc_data.dma_regs);
        g_omap_mmc_data.dma_regs = NULL;

        g_omap_mmc_data.request->buffer +=  g_omap_mmc_data.request->block_len;

	DEBUG(3,": RX Transfer finished\n");

        g_omap_mmc_data.request->nob--;

        omap_mmc_handle_int( &g_omap_mmc_data, inw(OMAP_MMC_STAT));
}

static void omap_mmc_tx_dma_callback (void *data, int size)
{

        omap_free_dma(g_omap_mmc_data.dma_regs);
        g_omap_mmc_data.dma_regs = NULL; 

        g_omap_mmc_data.request->buffer +=  g_omap_mmc_data.request->block_len;

        g_omap_mmc_data.request->nob--;

	DEBUG(3, ": TX Transfer finished. Status: %d\n", inw(OMAP_MMC_STAT));
}

static void omap_mmc_start_tx_dma_transfer (int size)
{
   dma_regs_t *regs;

   if(omap_request_dma(eMMCTx, "MMC/SD Tx DMA", (dma_callback_t)omap_mmc_tx_dma_callback, 
                    &g_omap_mmc_data, &(g_omap_mmc_data.dma_regs)))
   {
     printk(KERN_ERR "Failed to request MMC Tx DMA \n");
     return;
   }

   memcpy(g_omap_mmc_data.buf_dma_virt, g_omap_mmc_data.request->buffer,
          g_omap_mmc_data.request->block_len);

   regs = g_omap_mmc_data.dma_regs; 

         /* Configure DMA */
              
   regs->csdp = (1 << 10) | (1 << 9) | 1; 
   regs->ccr |= (1 << 13) | (1 << 5) | (1 << 6);
   regs->cicr = (1 << 5) | (1 << 1) | 1 ;
   regs->cdsa_l = OMAP_MMC_DATA & 0xffff;
   regs->cdsa_u = OMAP_MMC_DATA >> 16;
   regs->cssa_l =  g_omap_mmc_data.buf_dma_phys & 0xffff;
   regs->cssa_u =  g_omap_mmc_data.buf_dma_phys >> 16;
   regs->cen = 32;          /* 32 words per frame */
   regs->cfn = size / 64 ; 
   regs->cfi = 0;
   regs->cei = 1;

	/* Start DMA */

   regs->ccr |= (1 << 7); 

}

static void omap_mmc_start_rx_dma_transfer (int size)
{
   dma_regs_t *regs;

   if(omap_request_dma(eMMCRx, "MMC/SD Rx DMA", (dma_callback_t)omap_mmc_rx_dma_callback, 
                    &g_omap_mmc_data, &(g_omap_mmc_data.dma_regs)))
   {
     printk(KERN_ERR "Failed to request MMC Rx DMA \n");
     return; 
   }
 
   regs = g_omap_mmc_data.dma_regs; 

         /* Configure DMA */
              
   regs->csdp = (1 << 3) | (1 << 2) | 1; 
   regs->ccr |= (1 << 15) | (1 << 5) | (1 << 6);
   regs->cicr = (1 << 5) | (1 << 1) | 1 ;
   regs->cssa_l = OMAP_MMC_DATA & 0xffff;
   regs->cssa_u = OMAP_MMC_DATA >> 16;
   regs->cdsa_l =  g_omap_mmc_data.buf_dma_phys & 0xffff;
   regs->cdsa_u =  g_omap_mmc_data.buf_dma_phys >> 16;
   regs->cen = 32;          /* 32 words per frame */
   regs->cfn = size / 64 ; 
   regs->cfi = 0;
   regs->cei = 1;

	/* Start DMA */

   regs->ccr |= (1 << 7); 

}

static int omap_mmc_exec_command( struct mmc_request *request )
{

        u16 cmd_mask = 0;

        disable_irq( INT_MMC );
        outw(0xffff, OMAP_MMC_STAT); /* Clear status bits */ 

        /* FIX-ME: This valid only for MMC cards */

        if(((request->cmd == MMC_SEND_OP_COND) || 
           (request->cmd == MMC_ALL_SEND_CID) ||
           (request->cmd == MMC_SET_RELATIVE_ADDR) ||
           (request->cmd == MMC_GO_IRQ_STATE)) && (g_omap_mmc_data.sd == 0))
	{
	  /* Set Open Drain Mode for MMC commands 1,2,3,40 */
	  	  cmd_mask |= (1 << 6);
        }
        
	/* Set response type */

        switch(request->rtype)
	{         
	case RESPONSE_NONE:   
	  break;
        case RESPONSE_R1:
          cmd_mask |= 1 << 8;
          break;
        case RESPONSE_R1B:
          cmd_mask |= 1 << 8 | 1 << 11;
          break;
        case RESPONSE_R2_CID:
        case RESPONSE_R2_CSD:
          cmd_mask |= 1 << 9;
          break;
        case RESPONSE_R3:
          cmd_mask |= 1 << 8 | 1 << 9;
          break;
        case RESPONSE_R4:
          cmd_mask |= 1 << 10;
          break;
        case RESPONSE_R5:
          cmd_mask |= 1 << 10 | 1 << 8;
          break;
        case RESPONSE_R6:
          cmd_mask |= (1 << 9) | (1 << 10);  
          break;
        }
        
        /* Set command type */ 
        switch(request->cmd) {

          /* MMC core extra command */
        case MMC_CIM_RESET:
	  cmd_mask |= 1 << 7; /* Initialization sequence send prior to command */
          break;
	  /* bc - broadcast - no response */
        case MMC_GO_IDLE_STATE:
        case MMC_SET_DSR:
          break;
        /* bcr - broadcast with response */
        case MMC_SEND_OP_COND:
        case MMC_ALL_SEND_CID:
        case MMC_GO_IRQ_STATE:
          cmd_mask |= 1 << 12;
          break;
          /* adtc - addressed with data transfer */
        case MMC_READ_DAT_UNTIL_STOP:
        case MMC_READ_SINGLE_BLOCK:
        case MMC_READ_MULTIPLE_BLOCK:
        case SEND_SCR:
          cmd_mask |= 1 << 15 | 1 << 12 | 1 << 13;
          break;
        case MMC_WRITE_DAT_UNTIL_STOP:
        case MMC_WRITE_BLOCK:
        case MMC_WRITE_MULTIPLE_BLOCK:
        case MMC_PROGRAM_CID:
        case MMC_PROGRAM_CSD:
        case MMC_SEND_WRITE_PROT:
        case MMC_LOCK_UNLOCK:
        case MMC_GEN_CMD:
          cmd_mask |= 1 << 12 | 1 << 13;      
          break;
	  /* ac - no data transfer */
        default: 
          cmd_mask |= 1 << 13;
        }  
        
        /* Set command index */
        if( cmd_mask & (1 << 7))
	{
	/* Use this command to reset card */
	   cmd_mask |= MMC_GO_IDLE_STATE;
	}
	else
	{     
           cmd_mask |= request->cmd;
	}

        /* Set argument */    

      	outw(request->arg >> 16, OMAP_MMC_ARGH);
	outw(request->arg & 0xffff, OMAP_MMC_ARGL);

	switch (request->cmd) {
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
              	omap_mmc_set_transfer( request->block_len, request->nob );
		omap_mmc_start_rx_dma_transfer(request->block_len);
		outw((0x1F << 8) | (1 << 15), OMAP_MMC_BUF);  /* Set AF_level to 32 words */
	        g_omap_mmc_data.type = RT_READ;
                outw((1 << 7 | 1 << 8 ), OMAP_MMC_IE);	
		break;

	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
	        omap_mmc_set_transfer( request->block_len, request->nob );
		omap_mmc_start_tx_dma_transfer(request->block_len);
		g_omap_mmc_data.type = RT_WRITE;
                outw((1 << 7), OMAP_MMC_BUF);      /* Set AE_level to 1 word */
                outw((1 << 4 | 1 << 7 | 1 << 8 ), OMAP_MMC_IE);
		break;
	case SEND_SCR:
	        omap_mmc_set_transfer(8, 1); /* Get SCR */
	        outw(7, OMAP_MMC_BUF);
                outw((1 << 10 | 1 << 7 | 1 << 8), OMAP_MMC_IE);
                break;
	default:
                outw((1 | 1 << 7 | 1 << 8 | 1 << 14), OMAP_MMC_IE);	
		g_omap_mmc_data.type = RT_RESPONSE_ONLY;
		break;
	}
	
        /* Send command */

	DEBUG(3, ": Send cmd: %x \n Arg: %d\n", cmd_mask, request->arg);

        outw(cmd_mask, OMAP_MMC_CMD);        

	enable_irq( INT_MMC );
	
	return MMC_NO_ERROR;
}

static void omap_mmc_send_command( struct mmc_request *request )
{
	int retval;

	/* TODO: Grab a lock???? */
	g_omap_mmc_data.request = request;
	request->result = MMC_NO_RESPONSE; /* Flag to indicate don't have a result yet */

	if ( request->cmd == MMC_CIM_RESET ) 
        {

	  /* Reset OMAP MMC hardware */

	  	retval = omap_mmc_set_clock( MMC_CLOCK_SLOW );
                if ( retval ) 
                {

	  /* If any error occured -> exit with error code */
 
		   request->result = retval;
		   mmc_cmd_complete( request );
                   return;
	        }
	}
	
        if (request->cmd == SD_SEND_OP_COND) 
	{
	  g_omap_mmc_data.sd = 1;
        }

        if (request->cmd == MMC_SEND_OP_COND)
	{
          g_omap_mmc_data.sd = 0;
        } 
        
	    
	retval = omap_mmc_exec_command( request );

	if ( retval ) 
        {

	  /* If any error occured -> exit with error code */
 
		request->result = retval;
		mmc_cmd_complete( request );
	}
}

/**************************************************************************/
/* TODO: Fix MMC core for correct response processing */

static void omap_mmc_get_response( struct mmc_request *request )
{
	int i;

	u8 *buf = request->response;
	u16 data;
         
	request->result = MMC_NO_ERROR; /* Mark this as having a request result of some kind */
	DEBUG(3, ": Get response \n");
	switch(request->rtype)
	{
	case RESPONSE_R1:     
	case RESPONSE_R1B:
        case RESPONSE_R6:
	  data = inw(OMAP_MMC_RSP7);      /* Get response in little endian format */
	  DEBUG(3,": Response data: %x", data);
          *(buf) = request->cmd;          /* for mmc core */
          *(buf+1) = data >> 8;
          *(buf+2) = data & 0xff;

          data = inw(OMAP_MMC_RSP6);
	  DEBUG(3, ": %x \n", data);
          *(buf+3) = data >> 8;
          *(buf+4) = data & 0xff;
          break;

        case RESPONSE_R3:     
	case RESPONSE_R4:      
	case RESPONSE_R5:
 	  data = inw(OMAP_MMC_RSP7);      /* Get response in little endian format */
	  DEBUG(3, ": Response data: %x", data);
          *(buf) = 0x3f;              /* for mmc core */
	  *(buf+1) = data >> 8;
          *(buf+2) = data & 0xff;
          data = inw(OMAP_MMC_RSP6);
	  DEBUG(3," %x \n", data);
          *(buf+3) = data >> 8;
          *(buf+4) = data & 0xff;
          break;

	case RESPONSE_R2_CID: 
	case RESPONSE_R2_CSD:  
          *(buf) = 0x3f;              /* for mmc core */
          for ( i = 0 ; i < 8 ; )
	  { 
	     data = inw(OMAP_MMC_RSP7 - (i * 4));
             *(buf + (i * 2) + 1) = data >> 8;
             *(buf + (i * 2) + 2) = data & 0xff;
             i++;
          } 
	
        case RESPONSE_NONE:
	default:   
          break;
        }	

        if(request->cmd == SEND_SCR)
	{
          /* Read SCR */
          for (i = 0; i < 4; i++)
	  {
            data = inw(OMAP_MMC_DATA);
            udelay(1);
            DEBUG(3, ": SCR: %x\n", data);
            *(buf + (i * 2) + 6) = data >> 8;
            *(buf + (i * 2) + 5) = data & 0xff;
          }
        }

}

static void omap_mmc_handle_int( struct omap_mmc_data *sd, u16 status)
{
	int retval = MMC_NO_ERROR;
      

	DEBUG(3,": Interrupt. Status: %d\n", status);

        
        /* Data or Command time-out? */ 

	if ( status & (OMAP_MMC_CMD_TIMEOUT | OMAP_MMC_DATA_TIMEOUT)) 
        {
	        DEBUG(3, ": MMC/SD status: %d \n", inw(OMAP_MMC_STAT));
		retval = MMC_ERROR_TIMEOUT;
		goto terminate_int;
	}

	/* CRC error? */
         
	if (( status & (OMAP_MMC_CMD_CRC | OMAP_MMC_DATA_CRC)) &&
	    !(inw(OMAP_MMC_CON) & (1 << 15)))  
	{
                DEBUG(3, ": MMC/SD status: %d \n", inw(OMAP_MMC_STAT));
		retval = MMC_ERROR_CRC;
		goto terminate_int;
	}

	if (( (status & OMAP_MMC_END_OF_CMD) && (sd->request->result == MMC_NO_RESPONSE))
	  || (status & 1 << 14))  
            	omap_mmc_get_response( sd->request );

        if((sd->request->cmd == SET_BUS_WIDTH) && (sd->request->arg == 0x02))
	{

         /* Set 4-bit data width */
          DEBUG(3, ": Switching to 4-bit bus width \n");
	  outw(inw(OMAP_MMC_CON) | (1 << 15), OMAP_MMC_CON);

        }

	switch (g_omap_mmc_data.type) {
	case RT_NO_RESPONSE:
	        break;

        case RT_WRITE: 
              if(sd->request->nob) {
		  /* Start another transfer */
                  outw(0xffff, OMAP_MMC_STAT);
                  omap_mmc_start_tx_dma_transfer( g_omap_mmc_data.request->block_len );
                  return; 
	       }
	       break;

        case RT_READ:
               if(sd->request->nob) {
		  /* Start another transfer */
                 outw(0xffff, OMAP_MMC_STAT);
                 omap_mmc_start_rx_dma_transfer( g_omap_mmc_data.request->block_len );
                 return;
                } 
                break; 

	case RT_RESPONSE_ONLY:
		if ( sd->request->result < 0 ) {
			printk(KERN_INFO "MMC/SD: Illegal interrupt - command hasn't finished\n");
			retval = MMC_ERROR_TIMEOUT;
		}
		break;
        	
	}
	
terminate_int:

        outw(0, OMAP_MMC_IE); /* Clear all interrupts */

	sd->request->result = retval;
	mmc_cmd_complete( sd->request );
}

/* Handle IRQ */

static void omap_mmc_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_mmc_data *sd = (struct omap_mmc_data *) dev_id;
	u16 status = inw(OMAP_MMC_STAT);
	
	omap_mmc_handle_int( sd, status);
}

/* Detect Card */
static void omap_mmc_fix_sd_detect( unsigned long nr )
{
	card_state = inb(INNOVATOR_FPGA_INFO) & (1 << 4);

	if ( card_state == 0 ) 
        {
	        outw( (1 << 11), OMAP_MMC_CON); 
		mmc_insert(0);
	}
	else 
        {
		mmc_eject(0);
	}
}

static void omap_mmc_sd_detect_int(int irq, void *dev_id, struct pt_regs *regs)
{
	struct omap_mmc_data *sd = (struct omap_mmc_data *) dev_id;
	
	if(card_state != (inb(INNOVATOR_FPGA_INFO) & (1 << 4)))
	{
	   mod_timer( &sd->sd_detect_timer, jiffies + (500 * HZ) / 1000 );
	}    
}

static int omap_mmc_slot_is_empty( int slot )
{
	card_state = inb(INNOVATOR_FPGA_INFO) & (1 << 4);
	
	return card_state != 0;
}


/**************************************************************************
 *
 *                       Hardware initialization                                             
 *
 *************************************************************************/
static void omap_mmc_slot_up( void ) 
{
  
      /* OMAP MMC slot hardware init */

        outb(inb(OMAP1510P1_FPGA_POWER) | (1 << 3), OMAP1510P1_FPGA_POWER);
 
      /* Enable MMC/SD clock and disable HI-Z on the MMC.DAT2 pin*/

        outl(inl(MOD_CONF_CTRL_0) | (1 << 23) | (1 << 21), MOD_CONF_CTRL_0);

      /* Set up correct pin multiplexing (OMAP1510 mode) */
 
        /* Configure MMC.DAT3 pin */    
       
        outl(inl(FUNC_MUX_CTRL_D) & ~((1 << 14) | (1 << 13) | (1 << 12)), FUNC_MUX_CTRL_D);
   
        /* Configure MMC.CLK pin */

        outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 23) | (1 << 22) | (1 << 21)), FUNC_MUX_CTRL_A);

	/* Configure MMC_DAT0_SPI.DI pin */ 
    
        outl(inl(FUNC_MUX_CTRL_B) & ~((1 << 2) | (1 << 1) | (1 << 0)), FUNC_MUX_CTRL_B);
  
        /* Configure MMC.DAT2 pin */
      
        outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 20) | (1 << 19) | (1 << 18)), FUNC_MUX_CTRL_A);

        /* Configure MMC.DAT1 pin */

        outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 26) | (1 << 25) | (1 << 24)), FUNC_MUX_CTRL_A);

	/* Configure MMC.CMD_SPI.DO pin  */

        outl(inl(FUNC_MUX_CTRL_A) & ~((1 << 29) | (1 << 28) | (1 << 27)), FUNC_MUX_CTRL_A);
     
        /* Configure MMC_CON register */
       
        outw((1 << 11), OMAP_MMC_CON);

        /* Clear interrupts and status */
        
        outw(0, OMAP_MMC_IE);   /* Clear all interrupts */
        outw(0xffff, OMAP_MMC_STAT); /* Clear status bits */  

  
}

static void omap_mmc_slot_down( void )
{
    
	disable_irq( INT_FPGA_CD );
	disable_irq( INT_MMC );

	/* Power Off MMC through FPGA */
 
        outb(inb(OMAP1510P1_FPGA_POWER) & ~(1 << 3), OMAP1510P1_FPGA_POWER);
        
      /* Disable MMC/SD clock and enable HI-Z on the MMC.DAT2 pin*/

        outl(inl(MOD_CONF_CTRL_0) & ~((1 << 23) | (1 << 21)), MOD_CONF_CTRL_0);
      
        outw(0, OMAP_MMC_CON); 

	del_timer_sync(&g_omap_mmc_data.sd_detect_timer);
	
}

/* Standard PM functions */

static int omap_mmc_suspend(void)
{
        omap_mmc_stop_clock();
        omap_mmc_slot_down();
	return 0;
}

static void omap_mmc_resume(void)
{
	omap_mmc_slot_up();
        omap_mmc_set_clock(g_omap_mmc_data.clock);
       	/* Set up timers */

	g_omap_mmc_data.sd_detect_timer.function = omap_mmc_fix_sd_detect;
	g_omap_mmc_data.sd_detect_timer.data     = (unsigned long) &g_omap_mmc_data;
	init_timer(&g_omap_mmc_data.sd_detect_timer);

        enable_irq ( INT_MMC );        
        enable_irq ( INT_FPGA_CD ); /* Enable card detect IRQ */
}

#ifdef CONFIG_PM

static int omap_mmc_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{

  switch(req) {
  case PM_SUSPEND:
    mmc_eject(0);
    omap_mmc_suspend();
    break;

  case PM_RESUME:
    omap_mmc_resume();
    omap_mmc_fix_sd_detect(0);
    break;
  }

  return 0;

}

#endif

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CEE */

static int omap_mmc_dpm_suspend(struct device *dev, u32 state, u32 level)
{
 
  switch(level)
  {

     case SUSPEND_POWER_DOWN: 

       /* Turn off power to MMC/SD */
       mmc_eject(0);
       omap_mmc_suspend();     
       break;
  }

  return 0;

}

static int omap_mmc_dpm_resume(struct device *dev, u32 level)
{
  
  switch(level)
  {
     case RESUME_POWER_ON:

       /* Turn on power to MMC/SD */
       omap_mmc_resume();
       omap_mmc_fix_sd_detect(0);
       break;
  }

  return 0;
}

#endif

static int omap_mmc_slot_init( void )
{
	int retval;
        long flags;

	/* Set up timers */

	g_omap_mmc_data.sd_detect_timer.function = omap_mmc_fix_sd_detect;
	g_omap_mmc_data.sd_detect_timer.data     = (unsigned long) &g_omap_mmc_data;
	init_timer(&g_omap_mmc_data.sd_detect_timer);

	/* Basic service interrupt */

        local_irq_save(flags);

	retval = request_irq( INT_MMC, omap_mmc_int,
			      SA_INTERRUPT, "omap_mmc_int", &g_omap_mmc_data );
	if ( retval ) {
		printk(KERN_CRIT "MMC/SD: unable to grab MMC IRQ\n");
		return retval;
	}

	disable_irq( INT_MMC );

        /* Card Detect interrupt */

	retval = request_irq( INT_FPGA_CD, omap_mmc_sd_detect_int, 
			      SA_INTERRUPT, "omap_mmc_fpga_cd", &g_omap_mmc_data );

	if ( retval ) {
		printk(KERN_CRIT "MMC/SD: unable to grab FPGA_CD_IRQ\n");
		free_irq(INT_MMC, &g_omap_mmc_data);
	}

	disable_irq( INT_FPGA_CD );

        /* Allocate DMA buffers (max BLOCK LENGTH = 2048 (11 bit)) */

        g_omap_mmc_data.buf_dma_virt = consistent_alloc(GFP_KERNEL | GFP_DMA | GFP_ATOMIC,
                                       2048, &(g_omap_mmc_data.buf_dma_phys));

#ifdef CONFIG_PM
        pm_register(PM_UNKNOWN_DEV, PM_SYS_UNKNOWN, omap_mmc_pm_callback);
#endif
       	omap_mmc_slot_up(); 

        enable_irq ( INT_FPGA_CD ); /* Enable IRQ to detect card*/
        
        local_irq_restore(flags);          

	return retval;
}

static void omap_mmc_slot_cleanup( void )
{
        long flags;

        local_irq_save(flags); 

	omap_mmc_slot_down();
        
	free_irq(INT_FPGA_CD, &g_omap_mmc_data);
	free_irq(INT_MMC, &g_omap_mmc_data);

        /* Free DMA buffers */

        consistent_free(g_omap_mmc_data.buf_dma_virt, 2048, g_omap_mmc_data.buf_dma_phys);

        local_irq_restore(flags);
}

/***********************************************************/

static struct mmc_slot_driver dops = {
	owner:     THIS_MODULE,
	name:      "OMAP1510 MMC",
	ocr:       0x00ffc000,
	flags:     MMC_SDFLAG_MMC_MODE,

	init:      omap_mmc_slot_init,
	cleanup:   omap_mmc_slot_cleanup,
	is_empty:  omap_mmc_slot_is_empty,
	send_cmd:  omap_mmc_send_command,
	set_clock: omap_mmc_set_clock,
};

int __init omap_mmc_init(void)
{
	int retval;

#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
	omap_mmc_ldm_device_register();
        omap_mmc_ldm_driver_register();
#endif /* MVL-CCE */

	retval = mmc_register_slot_driver(&dops, 1);
	if ( retval < 0 )
		printk(KERN_INFO "MMC/SD: unable to register slot\n");

	return retval;
}

void __exit omap_mmc_cleanup(void)
{
#ifdef CONFIG_OMAP_INNOVATOR  /* MVL-CCE */
	omap_mmc_ldm_device_unregister();
        omap_mmc_ldm_driver_unregister();
#endif /* MVL-CCE */

	mmc_unregister_slot_driver(&dops);
}

#ifdef MODULE
module_init(omap_mmc_init);
module_exit(omap_mmc_cleanup);
#endif

MODULE_AUTHOR("Alexey Lugovskoy");
MODULE_DESCRIPTION("OMAP1510 Innovator MMC/SD");
MODULE_LICENSE("GPL");
EXPORT_NO_SYMBOLS;
