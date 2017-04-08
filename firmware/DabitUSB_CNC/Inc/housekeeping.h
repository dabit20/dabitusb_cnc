/*
 * housekeeping.h
 *
 *  Created on: Oct 14, 2016
 *      Author: dabit
 */

#ifndef HOUSEKEEPING_H_
#define HOUSEKEEPING_H_

#define NR_JOINTS 4
#define TICKS_PER_SEC 30000								/* Timer ISR rate divided by 2 */
#define ACCUMULATOR_MAX 1048575							/* 2^20-1 */
#define COMM_TIMEOUT (int32_t)(0.05 * TICKS_PER_SEC) 	/* Communication timeout 50msec in timer ticks */

#define NR_ADC_CHANNELS 5								/* Number of enabled ADC channels */

// PI controller coefficients
//#define P_COEFF ((int64_t)(ACCUMULATOR_MAX/32))		/* A 16-step error would result in maximum velocity */
//#define I_COEFF ((int64_t)(ACCUMULATOR_MAX/2048))	/* A 32-step integrated error would result in maximum velocity */
#define P_COEFF 1000
#define I_COEFF 10
#define I_LIMIT (TICKS_PER_SEC/10)

#ifndef bool
typedef uint8_t bool;
#endif
#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif


/* Joint administration */
typedef struct {
	/* Step generator working variables */
	int32_t accumulator;			/* NCO accumulator. Runs up to 2^20-1, then resets */
	bool StepPhase;					/* 0 or 1 */
	int32_t acc_increment;			/* Accumulator increment per cycle */
	/* Joint motion values */
	int32_t commanded_position;		/* commanded position in steps */
	int32_t actual_position;		/* actual position in steps */
	int32_t commanded_velocity;		/* Commanded velocity in steps/sec */
	int32_t commanded_acceleration;	/* Commanded acceleration in steps/sec^2 */
	/* Acceleration limiting */
	int64_t prev_joint_velocity;	/* Previous joint velocity */
	int32_t sg_max_acceleration;	/* Maximum acceleration for the stepgen. Defaults to 125% of the largest commanded acceleration */
	int64_t sg_maxvelincrease;		/* Same as maximum acceleration, but corrected for stepgen timer rate */
	/* PI controller */
	int32_t integrator;
	/* I/O pins. Note: we assume all joint i/o is on the I/O port as the joint 0 STEP pin */
	uint32_t step_pin;
	uint32_t dir_pin;
} sJointAdmin;

/* Instantiations are in main.c */
extern volatile sJointAdmin ja[NR_JOINTS];		/* Housekeeping data for the joints */
extern volatile uint64_t sender_timestamp;	/* Last host packet timestamp */
extern volatile bool bGotData;				/* Set to true when we received a packet from the host */
extern volatile int32_t CommTimeoutCtr;		/* Communications timeout downcounter */


#endif /* HOUSEKEEPING_H_ */
