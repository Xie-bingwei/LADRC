/**
 * @file          System_Model.h
 * @brief         Physical system simulation model for controller testing
 * @author        Xie Bingwei
 * @date          2025.12.30
 * @version       v1.3.0
 * @copyright     MIT License
 */
#ifndef LADRC_SYSTEM_MODEL_H
#define LADRC_SYSTEM_MODEL_H
typedef struct {
    float position;     // 位置状态
    float velocity;     // 速度状态
    float mass;         // 质量/惯性
    float damping;      // 阻尼
    float limit;        // 输出限幅
} SystemModel;

void System_Init(SystemModel *sys, float mass, float damping, float limit);
float System_Update(SystemModel *sys, float control_input, float dt);
#endif
