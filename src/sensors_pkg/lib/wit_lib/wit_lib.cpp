#include "wit_lib.h"

#include <thread>
#include <chrono>


int fd; // File descriptor for I2C communication

void delayFor(int ms){ // I made this function because I don't want to write std::... every time
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int32_t WitWriteReg(uint8_t uiReg, uint16_t usData)
{
    uint8_t ucBuff[2];
    if (uiReg >= REGSIZE)
        return 0;

    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;

    return IICwriteBytes(WT61P_ADDRESS, uiReg, ucBuff, 2);
}

int32_t WitReadReg(uint8_t uiReg, uint8_t uiReadNum, int32_t* data32)
{
    uint8_t ucBuff[12]; //max 6 reg
    uint8_t i;

    if(!IICreadBytes(WT61P_ADDRESS, uiReg, ucBuff, uiReadNum*2))
        return 0;
    for(i=0;i<uiReadNum; i++)
        data32[i] = (ucBuff[(i*2)+1] << 8) | ucBuff[i*2];

    return 1;
}

// FUNZIONI MODIFICATE PER RASPBERRY PI (wiringPiI2C.h) ==================================

int32_t WitInit()
{
  fd = wiringPiI2CSetup(WT61P_ADDRESS); // WT61P_ADDRESS is slave address
  if (fd == -1){
    return -1; // Failed to open I2C device
  }
    return 0;
}

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    if (fd == -1){
        return 0;
    }

    for (uint32_t i = 0; i < length; ++i){
        int byte = wiringPiI2CReadReg8(fd, reg + i);
        if (byte == -1){
            return 0;
        }
        data[i] = static_cast<uint8_t>(byte);
    }
    return 1;
}

int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    if (fd == -1){
        return 0;
    }

    for (uint32_t i = 0; i < length; ++i){
        int result = wiringPiI2CWriteReg8(fd, reg + i, data[i]);
        if (result == -1){
            return 0;
        }
    }
    return 1;
}

//=========================================================================================

int32_t getTEMP(float *T){
    int32_t data32;
    if(!WitReadReg(TEMP, 1, &data32))
        return 0;
    *T = data32/100.0f;
    return 1;
}

int32_t getAcc(float *A){
    int32_t data32[3];
    uint8_t i;
    if(!WitReadReg(AX, 3, data32))
        return 0;
    for(i=0; i<3; i++)
        A[i] = data32[i] * 0.00478516f; //  32768.0 / (16 * 9.8)
    return 1;
}

int32_t getAngle(float *A){
    int32_t data32[3];
    uint8_t i;
    if(!WitReadReg(Roll, 3, data32))
        return 0;
    for(i=0; i<3; i++)
        A[i] = data32[i] / 32768.0f * 180.0f;
    return 1;
}

int32_t getAngVel(float *A){
    int32_t data32[3];
    uint8_t i;
    if(!WitReadReg(GX, 3, data32))
        return 0;
    for(i=0; i<3; i++)
        A[i] = (float)data32[i] /32768 *2000;
    return 1;
}

int32_t WitSetOffset(uint8_t ureg, float offset){
    int16_t off = offset * 32768.0 / (16 * 9.8);  //error here
    uint8_t ucBuff[2];      //set offset, work!
    uint16_t usData=KEY_UNLOCK;
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS,KEY, ucBuff, 2))
        return 0;
    usData=off;
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS, AXOFFSET, ucBuff, 2))
        return 0;
    usData=0x0000;    // 0x01 to reset
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS, SAVE, ucBuff, 2))
        return 0;
    return 1;    
}

int32_t WitAccCali(void){
    uint8_t ucBuff[2];      //set offset, work!
    uint16_t usData=KEY_UNLOCK;
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS,KEY, ucBuff, 2))
        return 0;
    usData=CALGYROACC;
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS, CALSW, ucBuff, 2))
        return 0;

    delayFor(5000);
    usData=NORMAL;
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS, CALSW, ucBuff, 2))
        return 0;
    usData=0x0000;    // 0x01 to reset
    ucBuff[0] = usData & 0xff;
    ucBuff[1] = usData >> 8;
    if(!IICwriteBytes(WT61P_ADDRESS, SAVE, ucBuff, 2))
        return 0;
    return 1;
}