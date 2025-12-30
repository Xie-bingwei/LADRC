#include "LADRC.h"
#include "System_Model.h"

int main()
{
    LADRC ladrc;
    SystemModel system;
    // 初始化参数: wc=3.3, b0=1, w0=3.4, h=0.005, r=50
    LADRC_Init(&ladrc, 30.0f, 1.0f, 120.0f, 0.005f, 100.0f);
    System_Init(&system, 0.1f, 0.1f, 1000.0f);

    float target = 500.0f;
    float actual_output = 0.0f;

    for(int i = 0; i < 1000; i++) {
        LADRC_Update(&ladrc, target, actual_output);

        if(ladrc.u > 1000) ladrc.u = 1000;
        if(ladrc.u < -1000) ladrc.u = -1000;

        actual_output = System_Update(&system, ladrc.u, ladrc.h);

        float error = target - actual_output;
        printf("%d\t target: %f  actual_target: %f  u: %f  err: %f\n",i, target, actual_output, ladrc.u, error);

//        printf("target: %f, LADRC_OUT: %f, v1: %f\n", target, ladrc.u, ladrc.v1);
    }

    return 0;
}
