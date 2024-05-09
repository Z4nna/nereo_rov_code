#ifndef __AHRSREG_H
#define __AHRSREG_H

#ifdef __cplusplus
extern "C" {
#endif
#define REGSIZE 0x69

/* KEY */
#define KEY_UNLOCK	0xB588

/* SAVE */
#define SAVE_PARAM	0x00
#define SAVE_SWRST	0x01

/* REG */
#define SAVE 		0x00
#define CALSW 		0x01
#define RSW 		0x02
#define RRATE		0x03
#define BAUD 		0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08 //angular velocity
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD	    0x1c
#define YYMM		0x30
#define DDHH		0x31
#define MMSS		0x32
#define MS			0x33
#define AX			0x34
#define AY			0x35
#define AZ			0x36
#define GX			0x37
#define GY			0x38
#define GZ			0x39
#define Roll		0x3d
#define Pitch		0x3e
#define Yaw			0x3f
#define TEMP		0x40
#define KEY         0x69

/* CALIBRATION */
#define NORMAL 0x00
#define CALGYROACC 0x01

/* ORIENT */
#define ORIENT_HERIZONE	0
#define ORIENT_VERTICLE	1



#ifdef __cplusplus
}
#endif

#endif
