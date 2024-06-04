#ifndef __WIT_61_P
#define __WIT_61_P

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include "wit_c_sdk.h"
#include <stdio.h>

#define WT_TIMEOUT	3
#define RETRY	3

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static void AutoScanSensor(void);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len);
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len);
int WT61P_begin(char* dev, uint8_t addr);
void WT61P_read_angle();
void WT61P_read_acc();
void WT61P_read_angular_vel();
float WT61P_get_roll();
float WT61P_get_pitch();
float WT61P_get_yaw();
float WT61P_get_acc_x();
float WT61P_get_acc_y();
float WT61P_get_acc_z();
float WT61P_get_angular_vel_x();
float WT61P_get_angular_vel_y();
float WT61P_get_angular_vel_z();

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */