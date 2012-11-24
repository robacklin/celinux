/******************************************************************************
 *                  QLOGIC LINUX SOFTWARE
 *
 * QLogic ISP2x00 device driver for Linux 2.4.x
 * Copyright (C) 2003 Qlogic Corporation
 * (www.qlogic.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 ******************************************************************************/

/****************************************************************************
              Please see revision.notes for revision history.
*****************************************************************************/

static __u8	hwbroadcast_addr[ETH_ALEN] = { [0 ... ETH_ALEN-1] = 0xFF };


/**
 * qla2x00_ip_initialize() - Initialize RISC IP support.
 * @ha: SCSI driver HA context
 *
 * Prior to RISC IP initialization, this routine, if necessary, will reset all
 * buffers in the receive buffer ring.
 *
 * Returns TRUE if the RISC IP initialization succeeds.
 */
static int
qla2x00_ip_initialize(scsi_qla_host_t *ha)
{
	int		i;
	int		status;
	unsigned long	flags;
	device_reg_t	*reg;
	static mbx_cmd_t mc;
	mbx_cmd_t	*mcp = &mc;
	struct ip_init_cb *ipinit_cb;
	dma_addr_t	ipinit_cb_dma;

	DEBUG12(printk("%s: enter\n", __func__);)

	status = FALSE;

	/* Initialize IP data in ha */
	ha->ipdev_db_top = NULL;
	ha->ipdev_db_bottom = NULL;
	ha->ipdev_db_next_free = &ha->ipdev_db[0];
	for (i = 0; i < QLLAN_MAX_IP_DEVICES; i++) {
		ha->ipdev_db[i].index = i;
		ha->ipdev_db[i].next = &ha->ipdev_db[i+1];
	}
	ha->ipdev_db[QLLAN_MAX_IP_DEVICES-1].next = NULL;

	/* Reset/pack buffers owned by RISC in receive buffer ring */
	if (ha->rec_entries_in != ha->rec_entries_out) {
		struct buffer_cb	*bcb;
		uint16_t		rec_out;
		struct risc_rec_entry	*rec_entry;

		bcb = ha->receive_buffers;
		rec_out = ha->rec_entries_out;

		/*
		 * Must locate all RISC owned buffers and pack them in the
		 * buffer ring.
		 */
		/* between IpBufferOut and IpBufferIN */
		for (i = 0; i < ha->max_receive_buffers; i++, bcb++) {
			if (test_bit(BCB_RISC_OWNS_BUFFER, &bcb->state)) {
				/*
				 * Set RISC owned buffer into receive buffer
				 * ring.
				 */
				rec_entry = &ha->risc_rec_q[rec_out];
				rec_entry->handle = bcb->handle;
				rec_entry->data_addr_low =
					LS_64BITS(bcb->skb_data_dma);
			       	rec_entry->data_addr_high =
					MS_64BITS(bcb->skb_data_dma);
				if (rec_out < IP_BUFFER_QUEUE_DEPTH - 1)
					rec_out++;
				else
					rec_out = 0;
			}
		}

		/* Verify correct number of RISC owned buffers were found */
		if (rec_out != ha->rec_entries_in) {
			/* Incorrect number of RISC owned buffers?? */
			DEBUG12(printk("%s: incorrect number of RISC "
					"owned buffers, disable IP\n",
					__func__);)
			ha->flags.enable_ip = FALSE;
			return (FALSE);
		}
	}

	/* Init RISC buffer pointer */
	spin_lock_irqsave(&ha->hardware_lock, flags);
	reg = ha->iobase;
	WRT_REG_WORD(&reg->mailbox8, ha->rec_entries_in);
	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	/* Wait for a ready state from the adapter */
	while (!ha->init_done || ha->dpc_active) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}

	/* Setup IP initialization control block */
	ipinit_cb = pci_alloc_consistent(ha->pdev,
				sizeof(struct ip_init_cb), 
				&ipinit_cb_dma);
	if (ipinit_cb) {
		memset(ipinit_cb, 0, sizeof(struct ip_init_cb));
		ipinit_cb->version = IPICB_VERSION;
		ipinit_cb->firmware_options =
			__constant_cpu_to_le16(
				IPICB_OPTION_NO_BROADCAST_FASTPOST |
				 IPICB_OPTION_64BIT_ADDRESSING);
		ipinit_cb->header_size = cpu_to_le16(ha->header_size);
		ipinit_cb->mtu = cpu_to_le16((uint16_t)ha->mtu);
		ipinit_cb->receive_buffer_size =
			cpu_to_le16((uint16_t)ha->receive_buff_data_size);
		ipinit_cb->receive_queue_size =
			 __constant_cpu_to_le16(IP_BUFFER_QUEUE_DEPTH);
		ipinit_cb->low_water_mark =
			 __constant_cpu_to_le16(IPICB_LOW_WATER_MARK);
		ipinit_cb->receive_queue_addr[0] =
			cpu_to_le16(LSW(ha->risc_rec_q_dma));
		ipinit_cb->receive_queue_addr[1] =
			cpu_to_le16(MSW(ha->risc_rec_q_dma));
		ipinit_cb->receive_queue_addr[2] =
			cpu_to_le16(QL21_64BITS_3RDWD(ha->risc_rec_q_dma));
		ipinit_cb->receive_queue_addr[3] =
			cpu_to_le16(QL21_64BITS_4THWD(ha->risc_rec_q_dma));
		ipinit_cb->receive_queue_in = cpu_to_le16(ha->rec_entries_out);
		ipinit_cb->fast_post_count =
			 __constant_cpu_to_le16(IPICB_FAST_POST_COUNT);
		ipinit_cb->container_count =
			 __constant_cpu_to_le16(IPICB_BUFFER_CONTAINER_COUNT);
		ipinit_cb->resource_allocation =
			 __constant_cpu_to_le16(IPICB_IOCB_RESERVE_COUNT);

		/* Issue mailbox command to initialize IP firmware */
		mcp->mb[0] = MBC_INITIALIZE_IP;
		mcp->mb[2] = MSW(ipinit_cb_dma);
		mcp->mb[3] = LSW(ipinit_cb_dma);
		mcp->mb[6] = QL21_64BITS_4THWD(ipinit_cb_dma);
		mcp->mb[7] = QL21_64BITS_3RDWD(ipinit_cb_dma);
		mcp->out_mb = MBX_7|MBX_6|MBX_3|MBX_2|MBX_0;
		mcp->in_mb = MBX_0;
		mcp->tov = 30;
		mcp->buf_size = sizeof(struct ip_init_cb);
		mcp->flags = MBX_DMA_OUT;

		status = qla2x00_mailbox_command(ha, mcp);
		if (status == QL_STATUS_SUCCESS) {
			/* IP initialization successful */
			DEBUG12(printk("%s: successful\n", __func__);)

			ha->flags.enable_ip = TRUE;

			/* Force database update */
			set_bit(LOOP_RESYNC_NEEDED, &ha->dpc_flags);
			set_bit(LOCAL_LOOP_UPDATE, &ha->dpc_flags);
			set_bit(REGISTER_FC4_NEEDED, &ha->dpc_flags);

			/* qla2x00_loop_resync(ha); */
			if (ha->dpc_wait && !ha->dpc_active) {
				up(ha->dpc_wait);
			}
			status = TRUE;
		}
		else {
			DEBUG12(printk("%s: MBC_INITIALIZE_IP "
					"failed %x MB0 %x\n",
					__func__, 
					status,
					mcp->mb[0]);)
			status = FALSE;
		}
		pci_free_consistent(ha->pdev, sizeof(struct ip_init_cb),
					ipinit_cb, ipinit_cb_dma);

	}
	else {
		DEBUG12(printk("%s: memory allocation error\n", __func__);)
	}

	return (status);
}

/**
 * qla2x00_ip_send_complete() - Handle IP send completion.
 * @ha: SCSI driver HA context
 * @handle: handle to completed send_cb
 * @comp_status: Firmware completion status of send_cb
 *
 * Upon cleanup of the internal active-scb queue, the IP driver is notified of
 * the completion.
 */
static void
qla2x00_ip_send_complete(scsi_qla_host_t *ha,
			uint32_t handle, uint16_t comp_status)
{
	struct send_cb *scb;

	/* Set packet pointer from queue entry handle */
	if (handle < MAX_SEND_PACKETS) {
		scb = ha->active_scb_q[handle];
		if (scb) {
			ha->ipreq_cnt--;
			ha->active_scb_q[handle] = NULL;

			scb->comp_status = comp_status;
			pci_unmap_single(ha->pdev,
					scb->skb_data_dma,
					scb->skb->len,
					PCI_DMA_TODEVICE);
	
			/* Return send packet to IP driver */
			(*ha->send_completion_routine)(scb);
			return;
		}
	}

	/* Invalid handle from RISC, reset RISC firmware */
	printk(KERN_WARNING
		"%s: Bad IP send handle %x - aborting ISP\n",
		__func__, handle);

	set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
}

/**
 * qla2x00_ip_receive() - Handle IP receive IOCB.
 * @ha: SCSI driver HA context
 * @pkt: RISC IP receive packet
 *
 * Upon preparation of one or more buffer_cbs, the IP driver is notified of
 * the received packet.
 */
static void
qla2x00_ip_receive(scsi_qla_host_t *ha, response_t *pkt)
{
	uint32_t	handle;
	uint32_t	packet_size;
	uint16_t	linked_bcb_cnt;
	uint32_t	rec_data_size;
	uint16_t	comp_status;
	struct buffer_cb *bcb;
	struct buffer_cb *nbcb;
	struct ip_rec_entry *iprec_entry;

	DEBUG12(printk("%s: enter\n", __func__);)

	iprec_entry = (struct ip_rec_entry *)pkt;
	comp_status = le16_to_cpu(iprec_entry->comp_status);

	/* If split buffer, set header size for 1st buffer */
	if (comp_status & IPREC_STATUS_SPLIT_BUFFER)
		rec_data_size = ha->header_size;
	else
		rec_data_size = ha->receive_buff_data_size;

	handle = iprec_entry->buffer_handles[0];
	if (handle >= ha->max_receive_buffers) {
		/* Invalid handle from RISC, reset RISC firmware */
		printk(KERN_WARNING
			"%s: Bad IP buffer handle %x (> buffer_count)...Post "
			"ISP Abort\n",
			__func__,
			handle);
		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		return;
	}

	bcb = &ha->receive_buffers[handle];

	if (!test_and_clear_bit(BCB_RISC_OWNS_BUFFER, &bcb->state)) {
		/* Invalid handle from RISC, reset RISC firmware */
		printk(KERN_WARNING
			"%s: Bad IP buffer handle %x (!RISC_owned)...Post "
			"ISP Abort\n",
			__func__,
			handle);
		set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		return;
	}

	packet_size = le16_to_cpu(iprec_entry->sequence_length);
	bcb->comp_status = comp_status;
	bcb->packet_size = packet_size;
	nbcb = bcb;

	/* Prepare any linked buffers */
	for (linked_bcb_cnt = 1; ; linked_bcb_cnt++) {
		if (packet_size > rec_data_size) {
			nbcb->rec_data_size = rec_data_size;
			packet_size -= rec_data_size;

			/*
			 * If split buffer, only use header size on 1st buffer
			 */
			rec_data_size = ha->receive_buff_data_size;

			handle = iprec_entry->buffer_handles[linked_bcb_cnt];
			if (handle >= ha->max_receive_buffers) {
				/*
				 * Invalid handle from RISC reset RISC firmware
				 */
				printk(KERN_WARNING
					"%s: Bad IP buffer handle %x (> "
					"buffer_count - PS)...Post ISP Abort\n",
					__func__,
					handle);
				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
				return;
			}
			nbcb->next_bcb = &ha->receive_buffers[handle];
			nbcb = nbcb->next_bcb;

			if (!test_and_clear_bit(BCB_RISC_OWNS_BUFFER,
							&nbcb->state)) {
				/*
				 * Invalid handle from RISC reset RISC firmware
				 */
				printk(KERN_WARNING
					"%s: Bad IP buffer handle %x "
					"(!RISC_owned - PS)...Post ISP Abort\n",
					__func__,
					handle);
				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
				return;
			}
		}
		else {
			/* Single buffer_cb */
			nbcb->rec_data_size = packet_size;
			nbcb->next_bcb = NULL;
			break;
		}
	}

	/* Check for incoming ARP packet with matching IP address */
	if (le16_to_cpu(iprec_entry->service_class) == 0) {
		uint8_t	port_id[3];
		struct ip_device *ipdev;
		struct packet_header *packethdr;

		packethdr = (struct packet_header *)bcb->skb_data;

		/* Scan list of IP devices to see if login needed */
		for (ipdev = ha->ipdev_db_top; ipdev; ipdev = ipdev->next) {
			if (!memcmp(&ipdev->port_name[2],
				packethdr->networkh.s.na.addr, ETH_ALEN)) {
				/* Device already in IP list, skip login */
				goto skip_device_login;
			}
		}

		/* Device not in list, need to do login */
		port_id[2] = iprec_entry->s_idhigh;
// FIXME: endianess?
		port_id[1] = MSB(iprec_entry->s_idlow);
		port_id[0] = LSB(iprec_entry->s_idlow);

		/* Make sure its not a local device */
		if (port_id[2] == ha->d_id.b.domain &&
			port_id[1] == ha->d_id.b.area) {

			goto skip_device_login;
		}

		if (qla2x00_add_new_ip_device(ha,
					PUBLIC_LOOP_DEVICE,
					port_id,
					packethdr->networkh.s.fcaddr,
					TRUE,
					1) == QL_STATUS_FATAL_ERROR) {

			/* Fatal error, reinitialize */
			set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);
		}

	}

skip_device_login:

	/* Pass received packet to IP driver */
	bcb->linked_bcb_cnt = linked_bcb_cnt;
	(*ha->receive_packets_routine)(ha->receive_packets_context, bcb);

	/* Keep track of RISC buffer pointer (for IP reinit) */
	ha->rec_entries_out += linked_bcb_cnt;
	if (ha->rec_entries_out >= IP_BUFFER_QUEUE_DEPTH)
		ha->rec_entries_out -= IP_BUFFER_QUEUE_DEPTH;
}

/**
 * qla2x00_ip_receive_fastpost() - Handle IP receive fastpost.
 * @ha: SCSI driver HA context
 * @type: RISC fastpost type
 *
 * Upon preparation of one or more buffer_cbs, the IP driver is notified of
 * the received packet.
 */
static void
qla2x00_ip_receive_fastpost(scsi_qla_host_t *ha, uint16_t type)
{
	uint32_t	handle;
	uint32_t	packet_size;
	uint16_t	linked_bcb_cnt;
	uint32_t	rec_data_size;
	volatile uint16_t *next_mb;
	device_reg_t	*reg = ha->iobase;
	struct buffer_cb *bcb;
	struct buffer_cb *nbcb;

	DEBUG12(printk("%s: enter\n", __func__);)

	next_mb = &reg->mailbox10;

	/* If split buffer, set header size for 1st buffer */
	if (type == MBA_IP_RECEIVE_COMPLETE_SPLIT)
		rec_data_size = ha->header_size;
	else
		rec_data_size = ha->receive_buff_data_size;

	handle = RD_REG_WORD(next_mb);
	if (handle >= ha->max_receive_buffers) {
		goto invalid_handle;
	}

	bcb = &ha->receive_buffers[handle];

	if (!test_and_clear_bit(BCB_RISC_OWNS_BUFFER, &bcb->state)) {
		goto invalid_handle;
	}

	packet_size = RD_REG_WORD(&reg->mailbox3);
	/* Fastpost entries are always successfully transferred */
	bcb->comp_status = CS_COMPLETE;
	bcb->packet_size = packet_size;
	nbcb = bcb;

	/* Prepare any linked buffers */
	for (linked_bcb_cnt = 1; ; linked_bcb_cnt++) {
		if (packet_size > rec_data_size) {
			nbcb->rec_data_size = rec_data_size;
			packet_size -= rec_data_size;
			/*
			 * If split buffer, only use header size on 1st buffer
			 */
			rec_data_size = ha->receive_buff_data_size;

			next_mb++;
			handle = RD_REG_WORD(next_mb);
			if (handle >= ha->max_receive_buffers) {
invalid_handle:
				printk(KERN_WARNING
					"%s: bad IP receive fast post handle "
					"%x\n", 
					__func__,
					handle);
				set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);

				/* Clear interrupt - before leaving */
				WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
#if defined(ISP2200)
				WRT_REG_WORD(&reg->semaphore, 0);
#endif
				return;
			}

			nbcb->next_bcb = &ha->receive_buffers[handle];
			nbcb = nbcb->next_bcb;

			if (!test_and_clear_bit(BCB_RISC_OWNS_BUFFER,
							&nbcb->state)) {
				goto invalid_handle;
			}
		}
		else {
			/* Single buffer_cb */
			nbcb->rec_data_size = packet_size;
			nbcb->next_bcb = NULL;
			break;
		}
	}

	/* Clear interrupt */
	WRT_REG_WORD(&reg->host_cmd, HC_CLR_RISC_INT);
#if defined(ISP2200)
	WRT_REG_WORD(&reg->semaphore, 0);
#endif

	/* Pass received packet to IP driver */
	bcb->linked_bcb_cnt = linked_bcb_cnt;
	(*ha->receive_packets_routine)(ha->receive_packets_context, bcb);

	/* Keep track of RISC buffer pointer (for IP reinit) */
	ha->rec_entries_out += linked_bcb_cnt;
	if (ha->rec_entries_out >= IP_BUFFER_QUEUE_DEPTH)
		ha->rec_entries_out -= IP_BUFFER_QUEUE_DEPTH;
}

/**
 * qla2x00_convert_to_arp() - Convert an IP send packet to an ARP packet
 * @ha: SCSI driver HA context
 * @scb: The send_cb structure to convert
 *
 * Returns TRUE if conversion successful.
 */
static int
qla2x00_convert_to_arp(scsi_qla_host_t *ha, struct send_cb *scb)
{
	struct sk_buff		*skb;
	struct packet_header	*packethdr;
	struct arp_header	*arphdr;
	struct ip_header	*iphdr;

	DEBUG12(printk("%s: convert packet to ARP\n", __func__);)

	skb = scb->skb;
	packethdr = scb->header;
	arphdr = (struct arp_header *)skb->data;
	iphdr  = (struct ip_header *)skb->data;

	if (packethdr->snaph.ethertype == __constant_htons(ETH_P_IP)) {
		/* Convert IP packet to ARP packet */
		packethdr->networkh.d.na.naa = NAA_IEEE_MAC_TYPE;
		packethdr->networkh.d.na.unused = 0;
		memcpy(packethdr->networkh.d.na.addr,
				hwbroadcast_addr, ETH_ALEN);
		packethdr->snaph.ethertype = __constant_htons(ETH_P_ARP);

		arphdr->ar_tip = iphdr->iph.daddr;
		arphdr->ar_sip = iphdr->iph.saddr;
		arphdr->arph.ar_hrd = __constant_htons(ARPHRD_IEEE802);
		arphdr->arph.ar_pro = __constant_htons(ETH_P_IP);
		arphdr->arph.ar_hln = ETH_ALEN;
		arphdr->arph.ar_pln = sizeof(iphdr->iph.daddr); /* 4 */
		arphdr->arph.ar_op = __constant_htons(ARPOP_REQUEST);
		memcpy(arphdr->ar_sha, packethdr->networkh.s.na.addr, ETH_ALEN);
		memset(arphdr->ar_tha, 0, ETH_ALEN);

		skb->len = sizeof(struct arp_header);

		return (TRUE);
	}
	else {
		return (FALSE);
	}
}

/**
 * qla2x00_get_ip_loopid() - Retrieve loop id of an IP device.
 * @ha: SCSI driver HA context
 * @packethdr: IP device to remove
 * @loop_id: loop id of discovered device
 *
 * This routine will interrogate the packet header to determine if the sender is
 * in the list of active IP devices.  The first two bytes of the destination
 * address will be modified to match the port name stored in the active IP
 * device list.
 *
 * Returns TRUE if a valid loop id is returned.
 */
static int
qla2x00_get_ip_loopid(scsi_qla_host_t *ha,
		struct packet_header *packethdr, uint8_t *loop_id)
{
	struct ip_device *ipdev;

	/* Scan list of logged in IP devices for match */
	for (ipdev = ha->ipdev_db_top; ipdev; ipdev = ipdev->next) {
		if (memcmp(&ipdev->port_name[2],
				&(packethdr->networkh.d.fcaddr[2]), ETH_ALEN))
			continue;

		/* Found match, return loop ID  */
		*loop_id = (uint8_t)ipdev->loop_id;

		/* Update first 2 bytes of port name */
		packethdr->networkh.d.fcaddr[0] = ipdev->port_name[0];
		packethdr->networkh.d.fcaddr[1] = ipdev->port_name[1];

		if (ipdev != ha->ipdev_db_top) {
			/* Device not at top, move it to top of list */
			/* Unhook it first */
			if (ipdev == ha->ipdev_db_bottom) {
				ha->ipdev_db_bottom = ipdev->last;
				ipdev->last->next = NULL;
			}
			else {
				ipdev->last->next = ipdev->next;
				ipdev->next->last = ipdev->last;
			}

			/* Now put it at top of list */
			ipdev->next = ha->ipdev_db_top;
			ipdev->last = NULL;
			ha->ipdev_db_top->last = ipdev;
			ha->ipdev_db_top = ipdev;
		}
		return (TRUE);
	}

	/* Check for broadcast packet */
	if (!memcmp(packethdr->networkh.d.na.addr,
				hwbroadcast_addr, ETH_ALEN)) {
		/* Broadcast packet, return broadcast loop ID  */
		*loop_id = BROADCAST;

		/* Update destination NAA of header */
		packethdr->networkh.d.na.naa = NAA_IEEE_MAC_TYPE;
		packethdr->networkh.d.na.unused = 0;

		return (TRUE);
	}

	/* Check for multicast packet */
	if (packethdr->networkh.d.na.addr[0] & 0x01) {
		/* Use broadcast loop ID for multicast packets  */
		*loop_id = BROADCAST;

		/* Update destination NAA of header */
		packethdr->networkh.d.na.naa = NAA_IEEE_MAC_TYPE;
		packethdr->networkh.d.na.unused = 0;

		return (TRUE);
	}

	/* TODO */
	/* Try sending FARP IOCB to request login */

	DEBUG12(printk("%s: ID not found for "
			"XX XX %02x %02x %02x %02x %02x %02x\n",
			__func__,
			packethdr->networkh.d.na.addr[0],
			packethdr->networkh.d.na.addr[1],
			packethdr->networkh.d.na.addr[2],
			packethdr->networkh.d.na.addr[3],
			packethdr->networkh.d.na.addr[4],
			packethdr->networkh.d.na.addr[5]);)

	return (FALSE);
}

/**
 * qla2x00_reserve_loopid() - Reserve an unused public loop id.
 * @ha: SCSI driver HA context
 * @loop_id: loop id reserved
 *
 * Returns QL_STATUS_SUCCESS if a valid loop id is returned.
 */
static int
qla2x00_reserve_loopid(scsi_qla_host_t *ha, uint16_t *loop_id)
{
	int i;

	/* Look for unused loop ID */
	for (i = ha->min_external_loopid; i < ha->max_public_loop_ids; i++) {
		if (ha->fabricid[i].in_use)
			continue;

		/* Found free loop ID */
		ha->fabricid[i].in_use = TRUE;
		*loop_id = i;

		DEBUG12(printk("%s: assigned loop ID %x\n",
				__func__,
				*loop_id);)

		return (QL_STATUS_SUCCESS);
	}

	/* Out of loop IDs */
	*loop_id = ha->max_public_loop_ids + 1;     /* Set out of range */

	DEBUG12(printk("%s: out of loop IDs\n", __func__);)

	return (QL_STATUS_RESOURCE_ERROR);
}

/**
 * qla2x00_free_loopid() - Free a public loop id.
 * @ha: SCSI driver HA context
 * @loop_id: loop id to free
 */
static void
qla2x00_free_loopid(scsi_qla_host_t *ha, uint16_t loop_id)
{
	if (loop_id < ha->max_public_loop_ids) {
		ha->fabricid[loop_id].in_use = FALSE;
		DEBUG12(printk("%s: free loop ID %x\n",
				__func__,
				loop_id);)
	}
	else {
		DEBUG12(printk("%s: loop ID %x out of range\n",
				__func__,
				loop_id);)
	}
}


/**
 * qla2x00_add_new_ip_device() - Add a new IP capable device to the list.
 * @ha: SCSI driver HA context
 * @loop_id: loop id, if a private loop, of the new device
 * @port_id: port id of the new device
 * @port_name: port name of the new device
 * @force_add: should the function force the addition of the device
 * @ha_locked: Flag indicating if the function is called with the hardware lock
 *
 * Prior to RISC IP initialization, this routine, if necessary, will reset all
 * buffers in the receive buffer ring.
 *
 * Returns QL_STATUS_SUCCESS if there were no errors adding the device.
 */
static int
qla2x00_add_new_ip_device(scsi_qla_host_t *ha,
			     uint16_t loop_id,
			     uint8_t *port_id,
			     uint8_t *port_name,
			     int force_add,
			     uint32_t ha_locked)
{
	int	status;
	struct ip_device *ipdev;

	/* Get free IP device block */
	status = qla2x00_reserve_ip_block(ha, &ipdev);
	if (status == QL_STATUS_RESOURCE_ERROR) {
		if (!force_add)
			return (status);

		/*
		 * Out of IP blocks, bump public device at bottom of list
		 */
		DEBUG12(printk("%s: bump device from IP list\n", __func__);)

		for (ipdev = ha->ipdev_db_bottom; ipdev; ipdev = ipdev->last) {
			if (!(ipdev->flags & IP_DEV_FLAG_PUBLIC_DEVICE))
				continue;

			/* Do fabric logout and free loop ID */
			qla2x00_ip_send_logout_port_iocb(ha, ipdev, ha_locked);
			qla2x00_free_loopid(ha, ipdev->loop_id);

			/* Move device to top of list */
			qla2x00_free_ip_block(ha, ipdev);
			status = qla2x00_reserve_ip_block(ha, &ipdev);
			break;
		}
		if (status != QL_STATUS_SUCCESS)
			return (status);
	}

	/* Save IP port name */
	memcpy(ipdev->port_name, port_name, WWN_SIZE);

	if (loop_id != PUBLIC_LOOP_DEVICE) {
		/* Private loop device */
		ipdev->loop_id = loop_id;
		ipdev->flags = IP_DEV_FLAG_PRESENT;

		DEBUG12(printk("%s: WWN:%02x%02x%02x%02x%02x%02x%02x%02x, "
				"LoopID:%x\n",
				__func__,
				ipdev->port_name[0],
				ipdev->port_name[1],
				ipdev->port_name[2],
				ipdev->port_name[3],
				ipdev->port_name[4],
				ipdev->port_name[5],
				ipdev->port_name[6],
				ipdev->port_name[7],
				ipdev->loop_id);)
	}
	else {
		/* Public device */
		/* Reserve public loop ID, save it in database */
		status = qla2x00_reserve_loopid(ha, &ipdev->loop_id);
		if (status == QL_STATUS_RESOURCE_ERROR) {
			struct ip_device *ipdev_bump;

			if (!force_add) { 
				/* Failed to get loop ID */
				DEBUG12(printk("%s: failed to get loop ID\n",
						__func__);)
				qla2x00_free_ip_block(ha, ipdev);

				return (status);
			}

			/*
			 * Out of loop IDs, bump public device at bottom of
			 * list.
			 */
			DEBUG12(printk("%s: bump device from IP list\n",
					__func__);)

			for (ipdev_bump = ha->ipdev_db_bottom;
				ipdev_bump;
				ipdev_bump = ipdev_bump->last) {

				if (!(ipdev_bump->flags &
						IP_DEV_FLAG_PUBLIC_DEVICE))
					continue;

				/*
				 * Do fabric logout, steal loop ID, free bumped
				 * IP block.
				 */
				qla2x00_ip_send_logout_port_iocb(ha,
						ipdev_bump, ha_locked);
				ipdev->loop_id = ipdev_bump->loop_id;
				qla2x00_free_ip_block(ha, ipdev_bump);

				status = QL_STATUS_SUCCESS;
				break;
			}

			if (status != QL_STATUS_SUCCESS) {
				/* Failed to get loop ID */
				DEBUG12(printk("%s: failed to get loop ID\n",
						__func__);)
				qla2x00_free_ip_block(ha, ipdev);

				return (status);
			}
		}

		/* Save device data */
		ipdev->port_id[0] = port_id[0];
		ipdev->port_id[1] = port_id[1];
		ipdev->port_id[2] = port_id[2];
		ipdev->flags = IP_DEV_FLAG_PUBLIC_DEVICE;

		/* Login public device */
		status = qla2x00_ip_send_login_port_iocb(ha, ipdev, ha_locked);
		if (status == QL_STATUS_SUCCESS) {
			DEBUG12(printk("%s: "
					"WWN:%02x%02x%02x%02x%02x%02x%02x%02x, "
					"LoopID:%x, PortID:%x\n",
					__func__,
					ipdev->port_name[0],
					ipdev->port_name[1],
					ipdev->port_name[2],
					ipdev->port_name[3],
					ipdev->port_name[4],
					ipdev->port_name[5],
					ipdev->port_name[6],
					ipdev->port_name[7],
					ipdev->loop_id,
					ipdev->port_id[2]<<16 |
					ipdev->port_id[1]<<8 |
					ipdev->port_id[0]);)
		}
		else {
			/* Login failed, return resources */
			qla2x00_free_loopid(ha, ipdev->loop_id);
			qla2x00_free_ip_block(ha, ipdev);
		}
	}

	return (status);
}

/**
 * qla2x00_free_ip_block() - Remove an IP device from the active IP list.
 * @ha: SCSI driver HA context
 * @ipdev: IP device to remove
 */
static void
qla2x00_free_ip_block(scsi_qla_host_t *ha, struct ip_device *ipdev)
{
	/* Unhook IP device block from active list */
	if (ipdev->last == NULL)
		ha->ipdev_db_top = ipdev->next;
	else
		ipdev->last->next = ipdev->next;

	if (ipdev->next == NULL)
		ha->ipdev_db_bottom = ipdev->last;
	else
		ipdev->next->last = ipdev->last;

	/* Add IP device block to free list */
	ipdev->next = ha->ipdev_db_next_free;
	ha->ipdev_db_next_free = ipdev;
}

/**
 * qla2x00_reserve_ip_block() - Move an IP device to the IP device list.
 * @ha: SCSI driver HA context
 * @ipdevblk: reserved IP device to add 
 *
 * This routine will move the unused @ipdevblk from the free list to the top of
 * the active IP device list.
 *
 * Returns QL_STATUS_SUCCESS if the operation succeeded.
 */
static int
qla2x00_reserve_ip_block(scsi_qla_host_t *ha, struct ip_device **ipdevblk)
{
	struct ip_device *ipdev;

	/* Get free IP device block */
	ipdev = ha->ipdev_db_next_free;
	if (ipdev) { 
		/* Remove IP device block from free list */
		ha->ipdev_db_next_free = ipdev->next;

		/* Add IP device block to top of IP device list */
		ipdev->next = ha->ipdev_db_top;
		ipdev->last = NULL;
		if (ha->ipdev_db_top == NULL)
			ha->ipdev_db_bottom = ipdev;
		else
			ha->ipdev_db_top->last = ipdev;
		ha->ipdev_db_top = ipdev;

		*ipdevblk = ipdev;

		return (QL_STATUS_SUCCESS);
	}

	/* Out of IP blocks */
	DEBUG12(printk("%s: out of IP blocks\n", __func__);)

	return (QL_STATUS_RESOURCE_ERROR);
}

/**
 * qla2x00_update_ip_device_data() - Update IP device list with driver data.
 * @ha: SCSI driver HA context
 * @fcdev: SCSI driver FC device list
 *
 * This routine searchs for the device port name in the current IP database and
 * updates the IP device list.
 *
 * If device found:
 *	- Handle device movement between public and private loops
 *	- Mark device present
 *	- Log in device if necessary
 * If device not found and private loop device:
 *	- Insert the new entry in database
 * If device not found and public IP device:
 * 	- Ignore device until packet received from device
 *
 * Returns QL_STATUS_SUCCESS if the operation succeeded.
 */
static int
qla2x00_update_ip_device_data(scsi_qla_host_t *ha, fcdev_t *fcdev)
{
	int	status;
	struct ip_device *ipdev;

	status = 0;

	if (!ha->flags.enable_ip) {
		/* IP not enabled, just return */
		return (QL_STATUS_SUCCESS);
	}

	/* Scan list of IP devices for match */
	for (ipdev = ha->ipdev_db_top; ipdev; ipdev = ipdev->next) {
		if (memcmp(fcdev->wwn, ipdev->port_name, WWN_SIZE))
			continue;

		/* Found device in IP device list */
		DEBUG12(printk("%s: already in IP list, port ID: %x\n",
				__func__,
				ipdev->port_id[2] << 16 |
				 ipdev->port_id[1] << 8 |
				 ipdev->port_id[0]);)

		if (fcdev->flag != DEV_PUBLIC &&
			!(ipdev->flags & IP_DEV_FLAG_PUBLIC_DEVICE)) {
			/*
			 * Device on private loop now, was on private loop
			 * before.
			 */
			DEBUG12(printk("%s: was private loop, now "
					"private loop\n",
					__func__);)

			/* Update private loop ID in database */
			ipdev->loop_id = fcdev->loop_id;
			ipdev->flags |= IP_DEV_FLAG_PRESENT;
		}
		else if (fcdev->flag != DEV_PUBLIC &&
				(ipdev->flags & IP_DEV_FLAG_PUBLIC_DEVICE)) {
			/*
			 * Device on private loop now, was public device before.
			 */
			DEBUG12(printk("%s: was public, now private loop\n",
					__func__);)

			/*
			 * If loop ID changed, logout device and free loop ID.
			 */
			if (fcdev->loop_id != ipdev->loop_id) { 
				qla2x00_ip_send_logout_port_iocb(ha, ipdev, 0);
				qla2x00_free_loopid(ha, ipdev->loop_id);

				/*
				 * Clear public device flag and save private
				 * loop ID in database.
				 */
				ipdev->flags &= ~IP_DEV_FLAG_PUBLIC_DEVICE;
				ipdev->loop_id = fcdev->loop_id;
			}
			ipdev->flags |= IP_DEV_FLAG_PRESENT;
		}
		else if (fcdev->flag == DEV_PUBLIC &&
				!(ipdev->flags & IP_DEV_FLAG_PUBLIC_DEVICE)) {
			/*
			 * Device public now, was on private loop before.
			 */
			DEBUG12(printk("%s: was private loop, now public\n",
					__func__);)

			/*
			 * Reserve public loop ID, save it in database.
			 */
			status = qla2x00_reserve_loopid(ha, &ipdev->loop_id);
			if (status == QL_STATUS_SUCCESS) { 
				/*
				 * Save port ID and set public device flag.
				 */
				ipdev->port_id[0] = fcdev->d_id.r.d_id[0];
				ipdev->port_id[1] = fcdev->d_id.r.d_id[1];
				ipdev->port_id[2] = fcdev->d_id.r.d_id[2];
				ipdev->flags |= IP_DEV_FLAG_PUBLIC_DEVICE;

				/* Login public device */
				status = qla2x00_ip_send_login_port_iocb(ha,
							ipdev, 0);
			}
			if (status == QL_STATUS_RESOURCE_ERROR) {
				/* Out of loop IDs */
				ipdev->flags &= ~IP_DEV_FLAG_PUBLIC_DEVICE;
			}
		}
		else {
			/*
			 * Device public now, was public device before.
			 */
			DEBUG12(printk("%s: was public, now public\n",
					__func__);)

			/* Check if port ID changed */
			if (ipdev->port_id[0] != fcdev->d_id.r.d_id[0] ||
				ipdev->port_id[1] != fcdev->d_id.r.d_id[1] ||
				ipdev->port_id[2] != fcdev->d_id.r.d_id[2]) {

				/* Save new port ID */
				ipdev->port_id[0] = fcdev->d_id.r.d_id[0];
				ipdev->port_id[1] = fcdev->d_id.r.d_id[1];
				ipdev->port_id[2] = fcdev->d_id.r.d_id[2];

				DEBUG12(printk("%s: Port ID changed\n",
						__func__);)

				/* Logout public device */
				qla2x00_ip_send_logout_port_iocb(ha, ipdev, 0);
			}

			/* Login public device */
			status = qla2x00_ip_send_login_port_iocb(ha, ipdev, 0);
			if (status == QL_STATUS_RESOURCE_ERROR) {
				/* Out of loop IDs */
				ipdev->flags &= ~IP_DEV_FLAG_PUBLIC_DEVICE;
			}
		}
		return (status);
	}

	/* Device not found in database */
	DEBUG12(printk("%s: device NOT in list\n", __func__);)

	/* If private loop device, add device to IP list */
	/* Public devices will be added as needed when packet received */
	if (fcdev->flag != DEV_PUBLIC) {
		/* Add (force) new private loop device to IP list */
		status = qla2x00_add_new_ip_device(ha,
					fcdev->loop_id,
					NULL,
					fcdev->wwn,
					TRUE,
					0);
	}

	/* The following code is temporary, until FARP supported */
	/* Login all IP public devices for now */
	if (fcdev->flag == DEV_PUBLIC) {
		/* Add (don't force) new public device to IP list */
		status = qla2x00_add_new_ip_device(ha,
					PUBLIC_LOOP_DEVICE,
					(uint8_t *)&fcdev->d_id,
					fcdev->wwn,
					FALSE,
					0);
	}

	return (status);
}

/**
 * qla2x00_ip_send_login_port_iocb() - Login to an IP device.
 * @ha: SCSI driver HA context
 * @ipdev: IP device to login to
 * @ha_locked: Flag indicating if the function is called with the hardware lock
 *
 * This routine will build and send a mailbox IOCB to login to a fabric port.
 *
 * The qla2x00_ip_mailbox_iocb_done() routine will be called upon IOCB
 * completion, where further processing is performed.
 *
 * Returns QL_STATUS_SUCCESS if the operation succeeded.
 */
static int
qla2x00_ip_send_login_port_iocb(scsi_qla_host_t *ha,
				struct ip_device *ipdev, uint32_t ha_locked)
{
	unsigned long	flags = 0;
	struct mbx_entry *mbxentry;

	DEBUG12(printk("%s: port ID: %x\n",
			__func__,
			ipdev->port_id[2]<<16 |
			ipdev->port_id[1]<<8 |
			ipdev->port_id[0]);)

	/* Send marker if required */
	if (ha->marker_needed != 0) {
		if (ha_locked) {
			if(__qla2x00_marker(ha,
					0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS)
				return (QL_STATUS_ERROR);
		}
		else {
			if(qla2x00_marker(ha,
					0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS)
				return (QL_STATUS_ERROR);
		}
		ha->marker_needed = 0;
	}

	if (!ha_locked)
		spin_lock_irqsave(&ha->hardware_lock, flags);

	mbxentry = (struct mbx_entry *)qla2x00_req_pkt(ha);
	if (mbxentry == NULL) {
		DEBUG12(printk("%s: failed\n", __func__);)

		if (!ha_locked)
			spin_unlock_irqrestore(&ha->hardware_lock, flags);

		return (QL_STATUS_ERROR);
	}

	/* Build fabric login MBX IOCB */
	mbxentry->entry_type = ET_MAILBOX_COMMAND;
	mbxentry->entry_count = 1;
	mbxentry->sys_define1 = SOURCE_IP;
	mbxentry->entry_status = 0;
	mbxentry->handle = cpu_to_le32(ipdev->index |
				(MBC_LOGIN_FABRIC_PORT << 16));
	mbxentry->loop_id = ipdev->loop_id;
	mbxentry->mb0 = __constant_cpu_to_le16(MBC_LOGIN_FABRIC_PORT);
	mbxentry->mb1 = cpu_to_le16((ipdev->loop_id << 8) | 
				(MBC_NO_PROCESS_LOGIN |
				 MBC_NO_PLOGI_IF_LOGGED_IN));
	mbxentry->mb2 = cpu_to_le16(ipdev->port_id[2]);
	mbxentry->mb3 = cpu_to_le16((ipdev->port_id[1] << 8) |
				ipdev->port_id[0]);
	mbxentry->mb6 = __constant_cpu_to_le16(0);
	mbxentry->mb7 = __constant_cpu_to_le16(0);

	/* Issue command to ISP */
	qla2x00_isp_cmd(ha);

	if (!ha_locked)
		spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return (QL_STATUS_SUCCESS);
}

/**
 * qla2x00_ip_send_logout_port_iocb() - Logout an IP device.
 * @ha: SCSI driver HA context
 * @ipdev: IP device to logout
 * @ha_locked: Flag indicating if the function is called with the hardware lock
 *
 * This routine will build and send a mailbox IOCB to logout a fabric port.
 *
 * The qla2x00_ip_mailbox_iocb_done() routine will be called upon IOCB
 * completion, where further processing is performed.
 *
 * Returns QL_STATUS_SUCCESS if the operation succeeded.
 */
static int
qla2x00_ip_send_logout_port_iocb(scsi_qla_host_t *ha,
				struct ip_device *ipdev, uint32_t ha_locked)
{
	unsigned long	flags = 0;
	struct mbx_entry *mbxentry;

	DEBUG12(printk("%s: port ID: %x\n",
			__func__,
			ipdev->port_id[2]<<16 |
			ipdev->port_id[1]<<8 |
			ipdev->port_id[0]);)

	/* Send marker if required */
	if (ha->marker_needed != 0) {
		if (ha_locked) {
			if(__qla2x00_marker(ha,
					0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS)
				return (QL_STATUS_ERROR);
		}
		else {
			if(qla2x00_marker(ha,
					0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS)
				return (QL_STATUS_ERROR);
		}
		ha->marker_needed = 0;
	}

	if (!ha_locked)
		spin_lock_irqsave(&ha->hardware_lock, flags);

	mbxentry = (struct mbx_entry *)qla2x00_req_pkt(ha);
	if (mbxentry == NULL) {
		DEBUG12(printk("%s: failed\n", __func__);)

		if (!ha_locked)
			spin_unlock_irqrestore(&ha->hardware_lock, flags);

		return (QL_STATUS_ERROR);
	}

	/* Build fabric logout MBX IOCB */
	mbxentry->entry_type = ET_MAILBOX_COMMAND;
	mbxentry->entry_count = 1;
	mbxentry->sys_define1 = SOURCE_IP;
	mbxentry->entry_status = 0;
	mbxentry->handle = cpu_to_le32(ipdev->index |
				(MBC_LOGOUT_FABRIC_PORT << 16));
	mbxentry->loop_id = ipdev->loop_id;
	mbxentry->mb0 = __constant_cpu_to_le16(MBC_LOGOUT_FABRIC_PORT);
	mbxentry->mb1 = cpu_to_le16(ipdev->loop_id << 8);
	mbxentry->mb2 = __constant_cpu_to_le16(0);
	mbxentry->mb3 = __constant_cpu_to_le16(0);
	mbxentry->mb6 = __constant_cpu_to_le16(0);
	mbxentry->mb7 = __constant_cpu_to_le16(0);

	/* Issue command to ISP */
	qla2x00_isp_cmd(ha);

	if (!ha_locked)
		spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return (QL_STATUS_SUCCESS);
}

/**
 * qla2x00_ip_mailbox_iocb_done() - Process an mailbox IOCB completion.
 * @ha: SCSI driver HA context
 * @mbxentry: completed mailbox IOCB entry
 *
 * This routine is currently used for fabric login and logouts only.
 */
static void
qla2x00_ip_mailbox_iocb_done(scsi_qla_host_t *ha, struct mbx_entry *mbxentry)
{
	int		status;
	uint16_t	cmd;
	uint16_t	index;
	struct ip_device *ipdev;
// FIXME: endianess?
	/* Parse-out originating mailbox command */
	cmd = MSW(mbxentry->handle);

	DEBUG12(printk("%s: cmd %x, status %x, mb0 %x, mb1 %x, mb2 %x\n",
			__func__,
			cmd,
			mbxentry->status,
			mbxentry->mb0,
			mbxentry->mb1,
			mbxentry->mb2);)

	/* Get device block pointer */
	index = LSW(mbxentry->handle);
	if (index >= QLLAN_MAX_IP_DEVICES) {
		/* Bad handle from ISP */
		DEBUG12(printk("%s: bad handle from isp\n", __func__);)

		/* TODO: Cleanup??? */

		return;
	}

	ipdev = &ha->ipdev_db[index];

	if (cmd == MBC_LOGOUT_FABRIC_PORT) {
		/* Check fabric logout completion status */
		if (/*mbxentry->status == CS_COMPLETE && */
			mbxentry->mb0 == MBS_COMMAND_COMPLETE) {

			/* Logout successful -- do nothing */
		}
		else {
			DEBUG12(printk("%s: fabric logout failed\n", __func__);)
		}
	}
	else {
		/* Check fabric login completion status */
		/* Note: sometimes ISP returns Status=0x30 and MB0=0x4000 */
		/* Therefore, only check mb0 for now */
		if (/* mbxentry->status == CS_COMPLETE && */
			mbxentry->mb0 == MBS_COMMAND_COMPLETE) {

			/* Login successful */
			ipdev->flags |= IP_DEV_FLAG_PRESENT;
		}
		else if (mbxentry->mb0 == MBS_PORT_ID_IN_USE) {
			/* Different loop ID already assigned to port ID */
			/* Use the one that is already assigned */
			qla2x00_free_loopid(ha, ipdev->loop_id);
			ipdev->loop_id = mbxentry->mb1;

			/* Do logout first and then relogin */
			qla2x00_ip_send_logout_port_iocb(ha, ipdev, 1);
			qla2x00_ip_send_login_port_iocb(ha, ipdev, 1);
		}
		else if (mbxentry->mb0 == MBS_LOOP_ID_IN_USE) {
			/* Loop ID already used for different port ID */
			/* Get a new loop ID and reissue login request */
			status = qla2x00_reserve_loopid(ha, &ipdev->loop_id);
			if (status == QL_STATUS_SUCCESS) {
				qla2x00_ip_send_login_port_iocb(ha, ipdev, 1);
			}
			else {
				DEBUG12(printk("%s: out of loop IDs\n",
						__func__);)

				qla2x00_free_ip_block(ha, ipdev);
			}
		}
		else {
			/* Login failed, return resources */
			DEBUG12(printk("%s: fabric login failed\n", __func__);)

			qla2x00_free_loopid(ha, ipdev->loop_id);
			qla2x00_free_ip_block(ha, ipdev);
		}
	}
}


/**
 * qla2x00_ip_inquiry() - Discover IP-capable adapters.
 * @adapter_num: adapter number to check (instance)
 * @inq_data: return bd_inquiry data of the discovered adapter
 *
 * This routine is called by the IP driver to discover adapters that support IP
 * and to get adapter parameters from the SCSI driver.
 *
 * Returns TRUE if the specified adapter supports IP.
 */
#if defined (ISP2200)
int
qla2200_ip_inquiry(uint16_t adapter_num, struct bd_inquiry *inq_data)
#elif defined(ISP2300)
int
qla2300_ip_inquiry(uint16_t adapter_num, struct bd_inquiry *inq_data)
#endif
{
	scsi_qla_host_t	*ha;

	/* Verify structure size and version */
	if ((inq_data->length != BDI_LENGTH) ||
		(inq_data->version != BDI_VERSION)) {

		DEBUG12(printk("%s: incompatable structure\n", __func__);)
		return (FALSE);
	}

	/* Find the specified host adapter */
	for (ha = qla2x00_hostlist;
		ha && ha->instance != adapter_num;
		ha = ha->next);

	if (ha) {
		if (!ha->flags.online)
			return (FALSE);

		DEBUG12(printk("%s: found adapter %d\n",
				__func__,
				adapter_num);)

		/* Return inquiry data to backdoor IP driver */
		set_bit(BDI_IP_SUPPORT, &inq_data->options);
		if (ha->flags.enable_64bit_addressing)
			set_bit(BDI_64BIT_ADDRESSING, &inq_data->options);
		inq_data->ha = ha;                
		inq_data->risc_rec_q = ha->risc_rec_q;
		inq_data->risc_rec_q_size = IP_BUFFER_QUEUE_DEPTH;
		inq_data->link_speed = ha->current_speed;
		memcpy(inq_data->port_name, ha->ip_port_name, WWN_SIZE);
		inq_data->pdev = ha->pdev;
		inq_data->ip_enable_routine = qla2x00_ip_enable;
		inq_data->ip_disable_routine = qla2x00_ip_disable;
		inq_data->ip_add_buffers_routine = qla2x00_add_buffers;
		inq_data->ip_send_packet_routine = qla2x00_send_packet;
		inq_data->ip_tx_timeout_routine = qla2x00_tx_timeout;
		return (TRUE);
	}
	return (FALSE);
}

/**
 * qla2x00_ip_enable() - Create IP-driver/SCSI-driver IP connection.
 * @ha: SCSI driver HA context
 * @enable_data: bd_enable data describing the IP connection
 *
 * This routine is called by the IP driver to enable an IP connection to the
 * SCSI driver and to pass in IP driver parameters.
 *
 * The HA context is propagated with the specified @enable_data and the
 * Firmware is initialized for IP support.
 * 
 * Returns TRUE if the IP connection was successfully enabled.
 */
static int
qla2x00_ip_enable(scsi_qla_host_t *ha, struct bd_enable *enable_data)
{
	int status;

	DEBUG12(printk("%s: enable adapter %d\n", __func__, (int)ha->host_no);)

	status = FALSE;

	/* Verify structure size and version and adapter online */
	if (!(ha->flags.online) ||
		(enable_data->length != BDE_LENGTH) ||
		(enable_data->version != BDE_VERSION)) {

		DEBUG12(printk("%s: incompatable structure or offline\n",
				__func__);)
		return (status);
	}

	/* Save parameters from IP driver */
	ha->mtu = enable_data->mtu;
	ha->header_size = enable_data->header_size;
	ha->receive_buffers = enable_data->receive_buffers;
	ha->max_receive_buffers = enable_data->max_receive_buffers;
	ha->receive_buff_data_size = enable_data->receive_buff_data_size;
	if (test_bit(BDE_NOTIFY_ROUTINE, &enable_data->options)) {
		ha->notify_routine = enable_data->notify_routine;
		ha->notify_context = enable_data->notify_context;
	}
	ha->send_completion_routine = enable_data->send_completion_routine;
	ha->receive_packets_routine = enable_data->receive_packets_routine;
	ha->receive_packets_context = enable_data->receive_packets_context;

	/* Enable RISC IP support */
	status = qla2x00_ip_initialize(ha);
	if (!status) {
		DEBUG12(printk("%s: IP initialization failed", __func__);)
		ha->notify_routine = NULL;
	}
	return (status);
}

/**
 * qla2x00_ip_disable() - Remove IP-driver/SCSI-driver IP connection.
 * @ha: SCSI driver HA context
 *
 * This routine is called by the IP driver to disable a previously created IP
 * connection.
 *
 * A Firmware call to disable IP support is issued.
 */
static void
qla2x00_ip_disable(scsi_qla_host_t *ha)
{
	int	rval;
	static mbx_cmd_t mc;
	mbx_cmd_t *mcp = &mc;

	DEBUG12(printk("%s: disable adapter %d\n", __func__, (int)ha->host_no);)

	/* Wait for a ready state from the adapter */
	while (!ha->init_done || ha->dpc_active) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}

	/* Disable IP support */
	ha->flags.enable_ip = FALSE;

	mcp->mb[0] = MBC_DISABLE_IP;
	mcp->out_mb = MBX_0;
	mcp->in_mb = MBX_0;
	mcp->tov = 30;
	mcp->flags = 0;
	rval = qla2x00_mailbox_command(ha, mcp);
	if (rval == QL_STATUS_SUCCESS) {
		/* IP disabled successful */
		DEBUG12(printk(KERN_INFO
				"%s: successful\n", __func__);)
	}
	else {
		DEBUG12(printk(KERN_WARNING
				"%s: MBC_DISABLE_IP failed\n", __func__);)
	}

	/* Reset IP parameters */
	ha->rec_entries_in = 0;
	ha->rec_entries_out = 0;
	ha->notify_routine = NULL;
}

/**
 * qla2x00_add_buffers() - Adds buffers to the receive buffer queue.
 * @ha: SCSI driver HA context
 * @rec_count: The number of receive buffers to add to the queue
 * @ha_locked: Flag indicating if the function is called with the hardware lock
 *
 * This routine is called by the IP driver to pass new buffers to the receive
 * buffer queue.
 */
static void
qla2x00_add_buffers(scsi_qla_host_t *ha, uint16_t rec_count, int ha_locked)
{
	int		i;
	uint16_t	rec_in;
	uint16_t	handle;
	unsigned long	flags = 0;
	device_reg_t	*reg;
	struct risc_rec_entry *risc_rec_q;
	struct buffer_cb *bcbs;

	flags = 0;
	risc_rec_q = ha->risc_rec_q;
	rec_in = ha->rec_entries_in;
	bcbs = ha->receive_buffers;

	/* Set RISC owns buffer flag on new entries */
	for (i = 0; i < rec_count; i++) {
		handle = risc_rec_q[rec_in].handle;
		set_bit(BCB_RISC_OWNS_BUFFER, &(bcbs[handle].state));
		if (rec_in < IP_BUFFER_QUEUE_DEPTH - 1)
			rec_in++;
		else
			rec_in = 0;
	}
	
	/* Update RISC buffer pointer */
	if (!ha_locked)
		spin_lock_irqsave(&ha->hardware_lock, flags);

	reg = ha->iobase;
	WRT_REG_WORD(&reg->mailbox8, rec_in);
	ha->rec_entries_in = rec_in;

	if (!ha_locked)
		spin_unlock_irqrestore(&ha->hardware_lock, flags);
}

/**
 * qla2x00_send_packet() - Transmit a send_cb.
 * @ha: SCSI driver HA context
 * @scb: The send_cb structure to send
 *
 * This routine is called by the IP driver to pass @scb (IP packet) to the ISP
 * for transmission.
 *
 * Returns QL_STATUS_SUCCESS if @scb was sent, QL_STATUS_RESOURCE_ERROR if the
 * RISC was too busy to send, or QL_STATUS_ERROR.
 */
static int
qla2x00_send_packet(scsi_qla_host_t *ha, struct send_cb *scb)
{
	int		i;
	uint16_t	cnt;
	uint16_t	temp;
	uint32_t	handle;
	unsigned long	flags;
	struct ip_cmd_entry *ipcmd_entry;
	struct sk_buff	*skb;
	device_reg_t	*reg;

	DEBUG12(printk("%s: enter\n", __func__);)

	skb = scb->skb;
	reg = ha->iobase;

	/* Check adapter state */
	if (!ha->flags.online) {
		return (QL_STATUS_ERROR);
	}

	/* Send marker if required */
	if (ha->marker_needed != 0) {
		if(qla2x00_marker(ha, 0, 0, MK_SYNC_ALL) != QLA2X00_SUCCESS) {
			printk(KERN_WARNING
				"%s: Unable to issue marker.\n",
				__func__);
			return (QL_STATUS_ERROR);
		}
		ha->marker_needed = 0;
	}

	/* Acquire ring specific lock */
	spin_lock_irqsave(&ha->hardware_lock, flags);

	if (ha->req_q_cnt < 4) {
		/* Update number of free request entries */
#if defined(ISP2200)
		cnt = qla2x00_debounce_register(&reg->mailbox4);
#else
		cnt = qla2x00_debounce_register(&reg->req_q_out);
#endif
		if (ha->req_ring_index < cnt)
			ha->req_q_cnt = cnt - ha->req_ring_index;
		else
			ha->req_q_cnt = REQUEST_ENTRY_CNT -
						(ha->req_ring_index - cnt);
	}

	if (ha->req_q_cnt >= 4) {
		/* Get tag handle for command */
		handle = ha->current_scb_q_idx;
		for (i = 0; i < MAX_SEND_PACKETS; i++) {
			handle++;
			if (handle == MAX_SEND_PACKETS)
				handle = 0;
			if (ha->active_scb_q[handle] == NULL) {
				ha->current_scb_q_idx = handle;
				goto found_handle;
			}
		}
	}

	/* Low on resources, try again later */
	spin_unlock_irqrestore(&ha->hardware_lock, flags);
	printk(KERN_WARNING
		"%s: Low on resources, try again later...\n",
		__func__);

	return (QL_STATUS_RESOURCE_ERROR);

found_handle:

	/* Build ISP command packet */
	ipcmd_entry = (struct ip_cmd_entry *)ha->request_ring_ptr;

	/* OPTIMIZATION ??? */
	/* Throughput increases an additional 10 Mbps with the following code */
	*((uint32_t *)(&ipcmd_entry->entry_type)) = 
			 __constant_cpu_to_le32(ET_IP_COMMAND_64 | (1 << 8));
	//ipcmd_entry->entry_type = ET_IP_COMMAND_64;
	//ipcmd_entry->entry_count = 1;
	//ipcmd_entry->sys_define = 0;
	//ipcmd_entry->entry_status = 0;
	
	ipcmd_entry->handle = handle;
	ipcmd_entry->reserved_1 = 0;

	/* Get destination loop ID for packet */
	if (!qla2x00_get_ip_loopid(ha, scb->header, &ipcmd_entry->loop_id)) {
		/* Failed to get loop ID, convert packet to ARP */
		if (qla2x00_convert_to_arp(ha, scb)) {
			/* Broadcast ARP */
			ipcmd_entry->loop_id = BROADCAST;
		}
		else {
			/* Return packet */
			spin_unlock_irqrestore(&ha->hardware_lock, flags);
			printk(KERN_WARNING
				"%s: Unable to determine loop id for "
				"destination.\n",
				__func__);
			return (QL_STATUS_ERROR);
		}
	}

	/* Default five second firmware timeout */
	ipcmd_entry->timeout = __constant_cpu_to_le16(5);
	ipcmd_entry->control_flags = __constant_cpu_to_le16(CF_WRITE);
	ipcmd_entry->reserved_2 = 0;
	ipcmd_entry->service_class = __constant_cpu_to_le16(0);

	ipcmd_entry->data_seg_count = __constant_cpu_to_le16(2);
	ipcmd_entry->ds.data_segs64[0].address[0] =
			cpu_to_le32(LS_64BITS(scb->header_dma));
	ipcmd_entry->ds.data_segs64[0].address[1] =
			cpu_to_le32(MS_64BITS(scb->header_dma));
	ipcmd_entry->ds.data_segs64[0].length =
			__constant_cpu_to_le32(sizeof(struct packet_header));
	scb->skb_data_dma = pci_map_single(ha->pdev,
					skb->data, skb->len,
					PCI_DMA_TODEVICE);
	ipcmd_entry->ds.data_segs64[1].address[0] =
			cpu_to_le32(LS_64BITS(scb->skb_data_dma));
	ipcmd_entry->ds.data_segs64[1].address[1] =
			cpu_to_le32(MS_64BITS(scb->skb_data_dma));
	ipcmd_entry->ds.data_segs64[1].length = cpu_to_le32(skb->len);

	ipcmd_entry->byte_count =
			cpu_to_le32(skb->len + sizeof(struct packet_header));

	/* Adjust ring index. */
	ha->req_ring_index++;
	if (ha->req_ring_index == REQUEST_ENTRY_CNT) {
		ha->req_ring_index = 0;
		ha->request_ring_ptr = ha->request_ring;
	} else
		ha->request_ring_ptr++;

	ha->ipreq_cnt++;
	ha->req_q_cnt--;
	ha->active_scb_q[handle] = scb;

	/* Set chip new ring index. */
#if defined(ISP2200)
	/* Added from 64bit start */
	temp = CACHE_FLUSH(&reg->mailbox4);
	WRT_REG_WORD(&reg->mailbox4, ha->req_ring_index);
#else
	/* Added from 64bit start */
	temp = CACHE_FLUSH(&reg->req_q_in);
	WRT_REG_WORD(&reg->req_q_in, ha->req_ring_index);
#endif

	spin_unlock_irqrestore(&ha->hardware_lock, flags);

	return (QL_STATUS_SUCCESS);
}

/**
 * qla2x00_tx_timeout() - Handle transmission timeout.
 * @ha: SCSI driver HA context
 *
 * This routine is called by the IP driver to handle packet transmission
 * timeouts.
 *
 * Returns QL_STATUS_SUCCESS if timeout handling completed successfully.
 */
static int
qla2x00_tx_timeout(scsi_qla_host_t *ha)
{
	/* TODO: complete interface */

	/* Reset RISC firmware for basic recovery */
	printk(KERN_WARNING
		"%s: A transmission timeout occured - aborting ISP\n",
		__func__);
	set_bit(ISP_ABORT_NEEDED, &ha->dpc_flags);

	return (QL_STATUS_SUCCESS);
}
