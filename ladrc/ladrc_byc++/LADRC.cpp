/**
 * @file          LADRC.cpp/.c
 * @brief         Linear Active Disturbance Rejection Control(LADRC) implementation
 * @author        Xie Bingwei
 * @date          2025.12.30
 * @version       v1.3.0
 * @copyright     MIT License
 */
#include "LADRC.h"

/**
 * @brief         Initialize LADRC controller
 * @param[in,out] ladrc  Pointer to LADRC controller structure
 * @param[in]     wc     Controller bandwidth (rad/s)
 * @param[in]     b0     System parameter estimate
 * @param[in]     w0     Observer bandwidth (rad/s)
 * @param[in]     h      Sampling period (seconds)
 * @param[in]     r      TD speed factor
 */
void LADRC_Init(LADRC *ladrc, float wc, float b0, float w0, float h, float r)
{
    ladrc->wc = wc;
    ladrc->b0 = b0;
    ladrc->w0 = w0;
    ladrc->h = h;
    ladrc->r = r;

    ladrc->v1 = ladrc->v2 = 0;
    ladrc->x1 = ladrc->x2 = ladrc->x3 = 0;
    ladrc->u = ladrc->u0 = 0;
    ladrc->kp = ladrc->kd = 0;
}

/**
 * @brief         Linear Active Disturbance Rejection Controller Tracking Differentiator(LADRC-TD)
 * @param[in,out] ladrc  Pointer to LADRC controller structure containing state and parameters
 * @param[in]     target Target reference value (setpoint) to track
 * @return        void
 * @par Algorithm
 * Discrete-time TD with fhan (fast optimal control synthesis function):
 *  1. Calculate intermediate variables: d = r * h, y = v1 - v + v2 * h
 *  2. Compute a0 = sqrt(d^2 + 8 * r * |y|)
 *  3. Compute a = v2 + sign(y) * (a0 - d) / 2
 *  4. Compute fh = -r * (a / d) - r * sign(a)
 *  5. Update states: v1 = v1 + h * v2, v2 = v2 + h * fh
 */
void LADRC_TD(LADRC *ladrc, float target)
{
//    float fh = -ladrc->r * ladrc->r * (ladrc->v1 - target) - 2 * ladrc->r * ladrc->v2;
//    ladrc->v1 += ladrc->v2 * ladrc->h;
//    ladrc->v2 += fh * ladrc->h;
    float d = ladrc->r * ladrc->h;
//    float d0 = ladrc->h * d;
    float y = ladrc->v1 - target + ladrc->v2 * ladrc->h;

    float a0 = sqrt(d*d + 8*ladrc->r * fabs(y));
    float a = ladrc->v2 + (float)(y > 0 ? 1 : -1) * (a0 - d) / 2;

    float fh = -ladrc->r * (a/d) - ladrc->r * (float)(a > 0 ? 1 : -1);

    ladrc->v1 += ladrc->v2 * ladrc->h;
    ladrc->v2 += fh * ladrc->h;
}

/**
 * @brief          Linear Extended State Observer (LESO) for LADRC
 * @param[in,out]  ladrc           Pointer to LADRC controller structure
 * @param[in]      actual_output   Actual system output(measured value)
 * @return         void
 * @par Observer Gain Calculation
 *  For a third-order LESO (for second-order system with extended state),
 *  the characteristic equation is: s³ + l1*s² + l2*s + l3 = (s + w0)³
 *  Therefore: l1 = 3*w0, l2 = 3*w0², l3 = w0³
 */
void LADRC_LESO(LADRC *ladrc, float actual_output)
{
    // 计算观测器增益
    ladrc->l1 = 3 * ladrc->w0;
    ladrc->l2 = 3 * ladrc->w0 * ladrc->w0;
    ladrc->l3 = ladrc->w0 * ladrc->w0 * ladrc->w0;

    float err = ladrc->x1 - actual_output;

    // 更新状态估计(欧拉法)
    ladrc->x1 += (ladrc->x2 - ladrc->l1 * err) * ladrc->h;
    ladrc->x2 += (ladrc->x3 - ladrc->l2 * err + ladrc->b0 * ladrc->u) * ladrc->h;
    ladrc->x3 += -ladrc->l3 * err * ladrc->h;
}

/**
 * @brief          PD Controller for LADRC
 * @param[in,out]  ladrc   Pointer to LADRC controller structure
 * @return         void
 * @sideeffect     Updates the following fields in the LADRC structure:
 *                 -ladrc->kp: Proportional gain (wc²)
 *                 -ladrc->kd: Derivative gain (2*wc)
 *                 -ladrc->u0: Nominal PD control output
 *                 -ladrc->u:  Final control output with disturbance compensation
 */
void LADRC_PD(LADRC *ladrc)
{
    // 计算PD参数
    ladrc->kp = ladrc->wc * ladrc->wc;
    ladrc->kd = 2 * ladrc->wc;

    // 计算控制量
    float e1 = ladrc->v1 - ladrc->x1;
    float e2 = 0 - ladrc->x2;
    ladrc->u0 = ladrc->kp * e1 + ladrc->kd * e2;
    ladrc->u = (ladrc->u0 - ladrc->x3) / ladrc->b0;
}

void LADRC_Update(LADRC *ladrc, float target, float actual_output)
{
    LADRC_TD(ladrc, target);
    LADRC_LESO(ladrc, actual_output);
    LADRC_PD(ladrc);
}
