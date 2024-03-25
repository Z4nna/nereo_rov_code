#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"

#define WT61P_ADDRESS 0x50 // I2C address found with I2CScanner by Luis

int32_t WitWriteReg(uint8_t uiReg, uint16_t usData);
int32_t WitReadReg(uint8_t uiReg, uint8_t uiReadNum, int32_t* data32);
int32_t WitInit();
int32_t getTEMP(float *T);
int32_t getAcc(float *A);
int32_t getAngle(float *A);
int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);lam
int32_t WitSetOffset(uint8_t ureg, float offset);
int32_t WitAccCali(void);
int32_t getAngVel(float *A);

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
