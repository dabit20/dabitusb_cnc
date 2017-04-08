/*
 * commpackets.h
 *
 *  Created on: Oct 14, 2016
 *      Author: dabit
 */

#ifndef COMMPACKETS_H_
#define COMMPACKETS_H_

/* communication packets. Note: all data transferred in little endian */
#define PKT_TIMESTAMP_USEC_ID 10
#pragma pack(push,1)
typedef struct {
	uint8_t len;			// packet length in bytes, including this one = 10 bytes
	uint8_t id;				// id=PKT_TIMESTAMP_USEC_ID
	uint64_t timestamp;		// Monotonic timestamp, 1usec resolution.
} sPkt_Timestamp;

/* Communicate a joint position */
#define PKT_JOINTMOTOR_ID 20
typedef struct {
	uint8_t len;			// packet length in bytes, including this one = 15 bytes
	uint8_t id;				// id=PKT_JOINTMOTOR_ID
	uint8_t jointnr;		// Joint number (following LinuxCNC conventions)
	int32_t position;		// Joint Position at time <timestamp> in hardware units (usually steps), 24.8 fixed point
	int32_t velocity;		// Joint velocity at time <timestamp> in hardware units per second, 24.8 fixed point
	int32_t acceleration;	// Joint acceleration at time <timestamp> in hardware units per second^2, 24.8 fixed point
} sPkt_JointMotor;

/* Communicate a block of 8 digital values */
#define PKT_DIGITALIO_ID 30
typedef struct sPkt_DigIO8 {
	uint8_t len;			// packet length in bytes, including this one = 4 bytes
	uint8_t id;				// id=PKT_DIGITALIO_ID
	uint8_t address;		// Digital I/O address. If for example 32 outputs are present this ranges from 0 to 3, giving 4x8=32 bits
	uint8_t value;			// I/O values
} sPkt_DigIO8;

/* Communicate an analog value */
#define PKT_ANALOGIO_ID 40
typedef struct {
	uint8_t len;			// packet length in bytes, including this one = 8 bytes
	uint8_t id;				// id=PKT_DIGITALIO_ID
	uint8_t address;		// Analog I/O address. If for example 32 outputs are present this ranges from 0 to 3, giving 4x8=32 bits
	int32_t value;			// I/O value
}  sPkt_AnIO1;

/* Packet types containing often used data. Used to get frame size within 64 bytes */
/* DaBitUSB V1 Host-2-machine communicates the I/O for the DaBitUSB controller v1 */
#define PKT_DABITUSB_H2M_V1 50
typedef struct {
	uint8_t len;				// packet length in bytes, including this one = 4 bytes
	uint8_t id;					// id=PKT_DABITUSB_H2M_V1
	uint32_t tslowres;			// Monotonic timestamp, 1 count=32usec resolution. Lower than original timestamp due to space constraints
	// 6 bytes
	int32_t position[4];		// Joint Position at time <timestamp> in hardware units (usually steps), 24.8 fixed point
	int32_t velocity[4];		// Joint velocity at time <timestamp> in hardware units per second, 24.8 fixed point
	int32_t acceleration[4];	// Joint acceleration at time <timestamp> in hardware units per second^2, 24.8 fixed point
	// 54 bytes
	uint8_t digout_a0;			// Digital output bits for address 0
	uint16_t pwmout[2];			// PWM 1 and 2 output values, range 0-0xffff
	// 59 bytes
} sPkt_DaBitUSB_H2M_v1;

/* DaBitUSB V1 Machine to Host communicates the I/O for the DaBitUSB controller v1 */
#define PKT_DABITUSB_M2H_V1 60
typedef struct {
	uint8_t len;				// packet length in bytes, including this one = 4 bytes
	uint8_t id;					// id=PKT_DABITUSB_H2M_V1
	int32_t position[4];		// Joint Position at time <timestamp> in hardware units (usually steps), 24.8 fixed point
	// 14 bytes
	int32_t handwheel_count[3];	// Handwheel 1, 2 and 3 counts.
	uint8_t digin;				// Digital inputs
	uint16_t analogin[4];		// Analog inputs 0-3, range 0-0xffff equals 0.00-5.00V
	// 39 bytes
} sPkt_DaBitUSB_M2H_v1;

/* Signal end of command packets. Note that no filler bytes are needed  */
#define PKT_ENDOFCOMMAND 0xff
typedef struct {
	uint8_t len;			// packet length in bytes, including this one = 2
	uint8_t id;				// id=PKT_ENDOFCOMMAND
} sPkt_Endofcommand;
#pragma pack(pop)


#endif /* COMMPACKETS_H_ */
