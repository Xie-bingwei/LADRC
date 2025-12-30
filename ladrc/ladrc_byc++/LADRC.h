/**
 * @file          LADRC.cpp/.c
 * @brief         Linear Active Disturbance Rejection Control(LADRC) implementation
 * @author        Xie Bingwei
 * @date          2025.12.30
 * @version       v1.3.0
 * @copyright     MIT License
 */
#ifndef LADRC_LADRC_H
#define LADRC_LADRC_H

#include <iostream>
#include <cmath>
using namespace std;

typedef struct {
    float wc;   // 控制器带宽
    float b0;   // 控制增益
    float w0;   // 观测器带宽
    float h;    // 采样周期
    float r;    // 跟踪微分器参数

    float v1;   // 跟踪信号
    float v2;   // 微分信号

    float l1, l2, l3;  // 观测器增益
    float x1, x2, x3;  // 状态估计

    float kp, kd;      // PD参数

    float u;    // 最终控制量
    float u0;   // 中间控制量
} LADRC;

void LADRC_Init(LADRC *ladrc, float wc, float b0, float w0, float h, float r);
void LADRC_TD(LADRC *ladrc, float target);
void LADRC_LESO(LADRC *ladrc, float actual_output);
void LADRC_PD(LADRC *ladrc);
void LADRC_Update(LADRC *ladrc, float target, float actual_output);

#endif
