/*
 * camera.h
 *
 * Definition of Camera object.
 *
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/videodev.h>

#define CAMERA_OV6630_DEV_ID  0xC0
#define CAMERA_SANYO_DEV_ID   0x78

#define QCIF_WIDTH  176
#define QCIF_HEIGHT 144
#define CIF_WIDTH   352
#define CIF_HEIGHT  288

struct camera_interface;

typedef struct camera {
	// interface in which this camera is attached
	struct camera_interface * camif;

	int imageWidth;
	int imageHeight;
	int bytes_per_pixel;
	
	int (*detect)(void); // detect this camera

	int (*init)(void);   // one-time setup
	void (*cleanup)(void);

	int (*open)(void);
	int (*close)(void);
	
	// called when image format changes
	int (*setup)(struct v4l2_pix_format* fmt);

	// frame period is .1 usec units, exclk returned in MHz
	int (*set_frame_period)(int fp, int* exclk, int test);
	int (*get_frame_period)(int* exclk);
	
	/* convert raw camera image to that specified by V4L2 layer */
	int (*convert_image)(u8* src, void* dest, int to_user,
			     int dest_stride,
			     struct v4l2_pix_format* fmt);

	int (*query_control)(struct v4l2_queryctrl *qc);
	int (*get_control)(struct v4l2_control *vc);
	int (*set_control)(struct v4l2_control *vc);
	int (*query_menu)(struct v4l2_querymenu *qm);
} camera_t;

// Implementations of camera_t
extern struct camera camera_ov6x30;
extern struct camera camera_sanyo;

