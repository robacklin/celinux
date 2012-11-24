/*
 * BRIEF MODULE DESCRIPTION
 *	Au1000 USB Device-Side Driver
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2001 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#define USBDEV_REV 0x0110 // BCD
#define USBDEV_EP0_MAX_PACKET_SIZE 64

typedef enum {
	ATTACHED = 0,
	POWERED,
	DEFAULT,
	ADDRESS,
	CONFIGURED
} usbdev_state_t;

typedef enum {
	CB_NEW_STATE = 0,
	CB_PKT_COMPLETE
} usbdev_cb_type_t;


typedef struct usbdev_pkt {
	int                ep_addr;    // ep addr this packet routed to
	int                size;       // size of payload in bytes
	unsigned           status;     // packet status
	struct usbdev_pkt* next;       // function layer can't touch this
	u8                 payload[0]; // the payload
} usbdev_pkt_t;

#define PKT_STATUS_ACK  (1<<0)
#define PKT_STATUS_NAK  (1<<1)
#define PKT_STATUS_SU   (1<<2)

extern int usbdev_init(struct usb_device_descriptor* dev_desc,
		       struct usb_config_descriptor* config_desc,
		       struct usb_interface_descriptor* if_desc,
		       struct usb_endpoint_descriptor* ep_desc,
		       struct usb_string_descriptor* str_desc[],
		       void (*cb)(usbdev_cb_type_t, unsigned long, void *),
		       void* cb_data);

extern void usbdev_exit(void);

extern int usbdev_alloc_packet  (int ep_addr, int data_size,
				 usbdev_pkt_t** pkt);
extern int usbdev_send_packet   (int ep_addr, usbdev_pkt_t* pkt);
extern int usbdev_receive_packet(int ep_addr, usbdev_pkt_t** pkt);
extern int usbdev_get_byte_count(int ep_addr);
