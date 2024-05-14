#ifndef IMU_PUB_H
#define IMU_PUB_H

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <iostream>

#include "imu_libs/WT61P.h"

#define MAXN 20
#define WT61P_IIC_ADDR 0x50

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

typedef double float64;

void calcCovMatrix(std::queue<Vec3> window, float64 *matrix);

enum status {OK, WARN, ERROR, STALE};

#endif