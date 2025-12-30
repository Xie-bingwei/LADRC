/**
 * @file           System_Model.c
 * @brief          Physical system simulation model for controller testing
 * @author         Xie Bingwei
 * @date           2025-12-30
 * @version        v1.3.0
 * @copyright      MIT License
 * @details        This module implements a simplified physical system model for
 *                  testing and validating control algorithms.
 * @note           This is a SIMULATION ONLY module. It is NOT part of the actual
 *                  controller and should NOT be deployed on embedded targets.
 *                  It exists solely for testing and demonstration purposes.
 * @par System Dynamics
 *  The system is modeled as a point mass with viscous damping:
 *     m * ẍ + c * ẋ = u
 * Where:
 *     m: Mass (kg)
 *     c: Damping coefficient (N·s/m)
 *     u: Control input force (N)
 *     x: Position (m)
 *     ẋ: Velocity (m/s)
 *     ẍ: Acceleration (m/s²)
 * @par Discretization
 * The continuous-time dynamics are discretized using Euler integration:
 *     1. a[k] = (u[k] - c * v[k]) / m
 *     2. v[k+1] = v[k] + a[k] * dt
 *     3. x[k+1] = x[k] + v[k] * dt
 */

#include "System_Model.h"
/**
 * @brief           Initialize System model
 * @details         Represents a second-order mechanical system with
 *                   mass, damping, and actuator saturation limits.
 */
void System_Init(SystemModel *sys, float mass, float damping, float limit) {
    sys->position = 0;
    sys->velocity = 0;
    sys->mass = mass;
    sys->damping = damping;
    sys->limit = limit;
}

/**
 * @brief          Simulate one time step of physical system dynamics
 * @details        The continuous-time dynamics are:
 *                  m * dv/dt + c * v = u
 *                  dx/dt = v
 * @param[in,out]  sys             Pointer to system model structure
 * @param[in]      control_input   Control force to apply (N)
 *                                  Can be positive or negative
 * @param[in]      dt              Simulation time step (seconds)
 *                                  Must match controller sampling time
 * @return         float           Current system position after update (m)
 */
float System_Update(SystemModel *sys, float control_input, float dt) {
    // 限幅控制输入
    if (control_input > sys->limit) control_input = sys->limit;
    if (control_input < -sys->limit) control_input = -sys->limit;

    // 物理方程: F = m*a, 考虑阻尼力
    float acceleration = (control_input - sys->damping * sys->velocity) / sys->mass;

    // 积分得到速度和位置
    sys->velocity += acceleration * dt;
    sys->position += sys->velocity * dt;

    return sys->position;  // 位置作为实际输出
}
