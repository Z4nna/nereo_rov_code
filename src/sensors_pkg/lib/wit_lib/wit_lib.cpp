#include "wit_lib.h"
#include <Wire.h>
#include <Arduino.h>


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
int32_t WitInit()
{
  Wire.begin();
  delay(10);
  Wire.beginTransmission(WT61P_ADDRESS);
  Wire.endTransmission();
  return 0;
}

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    int val;
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission(0); // endTransmission but keep the connection active

    val = Wire.requestFrom(dev, length); // Ask for bytes, once done, bus is released by default

    if (val == 0)
        return 0;
    while (Wire.available() < length) // Hang out until we get the # of bytes we expect
    {
        if (Wire.getWireTimeoutFlag())
        {
            Wire.clearWireTimeoutFlag();
            return 0;
        }
    }

    for (int x = 0; x < length; x++)
        data[x] = Wire.read();

    return 1;
}

int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(data, length);
    if (Wire.getWireTimeoutFlag())
    {
        Wire.clearWireTimeoutFlag();
        return 0;
    }
    Wire.endTransmission(); // Stop transmitting

    return 1;
}

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
    delay(5000);
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