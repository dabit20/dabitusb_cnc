/* Quick & dirty driver module for the STM32F103 based USB motion controller.
 * This module can only run with PREEMPT_RT realtime kernels, not in an RTAI environment.
 *
 * Many things to make this production-ready code are missing, such as the ability to handle multiple devices, etc. 
 * 
 * Compile this module with 'sudo halcompile --install dabitusb.c
 */

#include <linux/input.h>
#include <linux/hidraw.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <endian.h>

#include <rtapi_slab.h>
#include <rtapi_ctype.h>
#include <rtapi_math64.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"

#include "hal.h"

/* Since HAL file compilation happens in a temporary directory, we must provide an absolute path */
#include "/home/linuxcnc/linuxcnc/dabitusb_module/commpackets.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bart van Hest");
MODULE_DESCRIPTION("DaBitUSB motion controller driver");
#ifdef MODULE_SUPPORTED_DEVICE
MODULE_SUPPORTED_DEVICE("DaBitUSB");
#endif

/* USB device to search for */
#define USBDEV_VID 0x0483
#define USBDEV_PID 0x5751

/* Number of joints */
#define NR_JOINTS 4

/* USB transmit and receive buffers */
static uint8_t xmitbuf[65], rcvbuf[65];

/* Component id */
static int comp_id;

/* USB device file descriptor */
static int usbfd=-1;

/* Communication packet to controller. Use machine-specific data due to size constraints 
   and the desire to transmit joint data every cycle
*/
#pragma pack(push,1)
struct __h2m_frame {
	sPkt_DaBitUSB_H2M_v1 h2m;
	sPkt_Endofcommand eoc;
};
static struct __h2m_frame h2m_frame;
#pragma pack(pop)
	
struct __comp_state {
    struct __comp_state *_next;
    hal_float_t *position_cmd_joint[NR_JOINTS];
    hal_float_t *position_fb_joint[NR_JOINTS];
    hal_u32_t *timestamp;
    hal_bit_t *enable;
    hal_bit_t *digout[4];
    hal_float_t *pwm[2];
    hal_float_t position_scale_joint[NR_JOINTS];
    hal_bit_t *digin[6];
    hal_float_t *analogin[4];
    hal_s32_t *handwheel_count[3];
    double joint_vel[NR_JOINTS];
    double joint_acc[NR_JOINTS];
    double prev_cmd_joint[NR_JOINTS];
    double prev_vel_joint[NR_JOINTS];
	uint64_t tscounter;
    uint32_t InitialiseDelay;
};

struct __comp_state *__comp_first_inst=0, *__comp_last_inst=0;

static void _(struct __comp_state *__comp_inst, long period);
static int __comp_get_data_size(void);
#undef TRUE
#define TRUE (1)
#undef FALSE
#define FALSE (0)
#undef true
#define true (1)
#undef false
#define false (0)
#undef fperiod
#define fperiod ((double)period * 1.0e-9)

static int export(char *prefix, long extra_arg) {
    char buf[HAL_NAME_LEN + 1];
    int r = 0,i;
    int sz = sizeof(struct __comp_state) + __comp_get_data_size();
    struct __comp_state *inst = hal_malloc(sz);
    memset(inst, 0, sz);
    for (i=0;i<NR_JOINTS;i++) {
		r = hal_pin_float_newf(HAL_IN, &(inst->position_cmd_joint[i]), comp_id,
		    "%s.position-cmd-joint%d", prefix,i);
		if(r != 0) return r;
		r = hal_pin_float_newf(HAL_OUT, &(inst->position_fb_joint[i]), comp_id,
		    "%s.position-fb-joint%d", prefix,i);
		if(r != 0) return r;
		r = hal_param_float_newf(HAL_RW, &(inst->position_scale_joint[i]), comp_id,
		    "%s.position-scale-joint%d", prefix,i);
	}
    r = hal_pin_u32_newf(HAL_OUT, &(inst->timestamp), comp_id,
        "%s.tslowres", prefix);
    if(r != 0) return r;
    r = hal_pin_bit_newf(HAL_IN, &(inst->enable), comp_id,
        "%s.enable", prefix);
    if(r != 0) return r;
    for (i=0;i<4;i++) {
    	r = hal_pin_bit_newf(HAL_IN, &(inst->digout[i]), comp_id,"%s.digout%d", prefix, i);
    	if(r != 0) return r;
    }
    for (i=0;i<2;i++) {
    	r = hal_pin_float_newf(HAL_IN, &(inst->pwm[i]), comp_id,"%s.pwm%d", prefix, i);
    	if(r != 0) return r;
    }
    for (i=0;i<6;i++) {
    	r = hal_pin_bit_newf(HAL_OUT, &(inst->digin[i]), comp_id,"%s.digitalin%d", prefix, i);
    	if(r != 0) return r;
    }
    for (i=0;i<4;i++) {
    	r = hal_pin_float_newf(HAL_OUT, &(inst->analogin[i]), comp_id,"%s.analogin%d", prefix, i);
    	if(r != 0) return r;
    }
    for (i=0;i<3;i++) {
    	r = hal_pin_s32_newf(HAL_OUT, &(inst->handwheel_count[i]), comp_id,"%s.handwheelcount%d", prefix, i);
    	if(r != 0) return r;
    }
		
    /* Setup defaults */
    inst->InitialiseDelay = 3;
    inst->tscounter = 0;
    *inst->enable = 0;
    for (i=0;i<NR_JOINTS;i++) {
    	inst->position_scale_joint[i] = 200.0;
    }
    
    rtapi_snprintf(buf, sizeof(buf), "%s", prefix);
    r = hal_export_funct(buf, (void(*)(void *inst, long))_, inst, 1, 0, comp_id);
    if(r != 0) return r;
    if(__comp_last_inst) __comp_last_inst->_next = inst;
    __comp_last_inst = inst;
    if(!__comp_first_inst) __comp_first_inst = inst;
    return 0;
}

static int __comp_get_data_size(void) { return 0; }

/* Brute-force open a couple of /dev/hidrawX devices and check if it is our motion controller */
static int FindAndOpenUSBdevice(void) {
	char buf[16];
	int i, fd, res;
	struct hidraw_devinfo info;
	for (i=0;i<16;i++) {
		rtapi_snprintf(buf, sizeof(buf), "/dev/hidraw%d", i);
		fd = open(buf, O_RDWR|O_NONBLOCK);
//		fd = open(buf, O_RDWR);	/* Note: do not rely on O_NONBLOCK; there is a kernel bug that prevents nonblock on disconnect */
		if (fd) {
			/* Opening the device succeeded. Extract VID/PID and compare */
			memset(&info, 0x0, sizeof(info));
			res = ioctl(fd, HIDIOCGRAWINFO, &info);
			if (res >= 0) {
				if (info.vendor == USBDEV_VID && info.product == USBDEV_PID) {
					/* We found our device. Return the fd. */
					rtapi_print("Found device at %s\n",buf);
					return fd;
				}
			}
			/* Not the correct device or we could not extract VID/PID. Close fd */
			close (fd);
		}
	}
	return -1;
}

int rtapi_app_main(void) {

    int ret,i;
    rtapi_print("loading DaBitUSB module\n");
    ret = hal_init("dabitusb");
    if (ret < 0)
        return ret;
    comp_id = ret;
    /* Open our USB device */
    usbfd = FindAndOpenUSBdevice();
    
	if (usbfd < 0) {
		rtapi_print("ERROR opening device, fd=%d\n", usbfd);
	} else {
		rtapi_print("SUCCESS opening device\n");
	}
	/* Initialize static packets members */
	h2m_frame.h2m.id = PKT_DABITUSB_H2M_V1;
	h2m_frame.h2m.len = sizeof(sPkt_DaBitUSB_H2M_v1);
	h2m_frame.eoc.id = PKT_ENDOFCOMMAND;
	h2m_frame.eoc.len = sizeof (sPkt_Endofcommand);
	
	/* Export pins and finalize HAL component creation */
	ret = export("dabitusb", 0);
    if(ret) {
        hal_exit(comp_id);
    } else {
        hal_ready(comp_id);
    }
    return ret;
}

void rtapi_app_exit(void) {
	if (usbfd > 0) {
		close (usbfd);
	}
    hal_exit(comp_id);
    rtapi_print("DaBitUSB module unloaded\n");
}

static void _(struct __comp_state *__comp_inst, long period) {
	int ret,i;
	uint8_t *data;
	uint8_t idx=0, jointnr;
	uint8_t *xmitptr;
	if (!(*__comp_inst->enable)) {
		/* After the enable, we want to dry-run for 3 cycles before we actually start transmitting data
		   This prevents weird acceleration and velocity values 
		*/
		__comp_inst->InitialiseDelay = 3;
	}
	/* Calculate joint velocities and accelerations */
	for (i=0;i<NR_JOINTS;i++) {
		__comp_inst->joint_vel[i] = ((*__comp_inst->position_cmd_joint[i]) - __comp_inst->prev_cmd_joint[i]) / fperiod;
		__comp_inst->joint_acc[i] = (__comp_inst->joint_vel[i] - __comp_inst->prev_vel_joint[i]) / fperiod;
		__comp_inst->prev_cmd_joint[i] = (double)*__comp_inst->position_cmd_joint[i];
		__comp_inst->prev_vel_joint[i] = __comp_inst->joint_vel[i];
	}			
	/* Update usec-resolution timestamp counter. We use LinuxCNC's period for this. Maybe we should use CLOCK_MONOTONIC? */
	__comp_inst->tscounter += (uint64_t)(period/1000);
	
	/* Do init if we're not initialized yet */
	if (__comp_inst->InitialiseDelay == 0) {
#if 1		
		/* Assemble frame. 
		 * Note the conversion to little-endian; unneeded on x86 but x86's do not rule the world anymore */
		h2m_frame.h2m.tslowres = htole32((int32_t)(__comp_inst->tscounter>>6));
		for (i=0;i<NR_JOINTS;i++) {
			h2m_frame.h2m.position[i] = htole32(((int32_t)(*__comp_inst->position_cmd_joint[i] * __comp_inst->position_scale_joint[i] * 256.0)));
			h2m_frame.h2m.velocity[i] = htole32(((int32_t)(__comp_inst->joint_vel[i] * __comp_inst->position_scale_joint[i] * 256.0)));
			h2m_frame.h2m.acceleration[i] = htole32(((int32_t)(__comp_inst->joint_acc[i] * __comp_inst->position_scale_joint[i] * 256.0)));
			//rtapi_print("pos:%d  vel:%d  acc:%d (%f)\n", le32toh(jointpkt[i].position), le32toh(jointpkt[i].velocity), le32toh(jointpkt[i].acceleration), __comp_inst->joint_acc[i]);
		}
		h2m_frame.h2m.digout_a0 =  (*__comp_inst->digout[0] ? 1 : 0 | 
		                  			*__comp_inst->digout[1] ? 2 : 0 | 
		                  			*__comp_inst->digout[2] ? 4 : 0 | 
		                  			*__comp_inst->digout[3] ? 8 : 0);
		for (i=0;i<2;i++) {
			if (*__comp_inst->pwm[i] < 0.0)
				h2m_frame.h2m.pwmout[i] = 0;
			else if (*__comp_inst->pwm[i] > 1.0)
				h2m_frame.h2m.pwmout[i] = 0;
			else
				h2m_frame.h2m.pwmout[i] = (uint16_t)(*__comp_inst->pwm[i] * 65535.0);
		}
		/* Copy frame in transmit buffer */
		xmitbuf[0] = 0;
		xmitptr = xmitbuf+1;
		memcpy (xmitptr, &h2m_frame, sizeof(sPkt_DaBitUSB_H2M_v1)); xmitptr+=sizeof(sPkt_DaBitUSB_H2M_v1);
		
		/* Transmit frame to USB device */
		ret=write(usbfd, xmitbuf, 65);
		if (ret != 65) {
			rtapi_print("USB transmit fail: %d bytes transmitted\n", ret);
		}
		/* See if there is data available */
		ret = read(usbfd, xmitbuf, 64);
		idx=0;
		if (ret == 64) {
			while (idx<64) {
				data = &xmitbuf[idx];
				/* Determine packet type and process the packet */
				switch (data[1]) {
					/* Timestamp? */
					case PKT_TIMESTAMP_USEC_ID:
						break;
					/* Joint motor feedback? */
					case PKT_JOINTMOTOR_ID:
						jointnr = ((sPkt_JointMotor *)data)->jointnr;
						if (jointnr >= NR_JOINTS)
							break;	
						*__comp_inst->position_fb_joint[jointnr] = (hal_float_t)((sPkt_JointMotor *)data)->position / __comp_inst->position_scale_joint[jointnr];
						//rtapi_print("Joint %d pos %f\n\n", jointnr, *__comp_inst->position_fb_joint[jointnr]);
						break;
					/* Complete DaBitUSB machine-to-host packet? */
					case PKT_DABITUSB_M2H_V1:
						for (i=0;i<NR_JOINTS;i++) {
							*__comp_inst->position_fb_joint[i] = (hal_float_t)((sPkt_DaBitUSB_M2H_v1 *)data)->position[i] / __comp_inst->position_scale_joint[i];
						}
						for (i=0;i<4;i++) {
							*__comp_inst->analogin[i] = (hal_float_t)((sPkt_DaBitUSB_M2H_v1 *)data)->analogin[i] / (65535.0/5.0);
						}
						for (i=0;i<6;i++) {
							*__comp_inst->digin[i] = (((sPkt_DaBitUSB_M2H_v1 *)data)->digin & (1<<i)) != 0;
						}
						for (i=0;i<3;i++) {
							*__comp_inst->handwheel_count[i] = ((sPkt_DaBitUSB_M2H_v1 *)data)->handwheel_count[i];
						}
						break;
					
					/* End of data frame command */
					case PKT_ENDOFCOMMAND:
						idx = 64;
						break;
				}
				/* go to next packet (or next byte if there was none) */
				if (data[0])
					idx += data[0];
				else
					idx++;
			}
		}
			
#endif
	} else {
		__comp_inst->InitialiseDelay--;
	}
}


