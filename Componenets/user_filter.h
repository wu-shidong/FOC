#ifndef __USER_FILTER_H
#define __USER_FILTER_H

typedef struct {
    float y_prev;       // 上一次的滤波结果
    float Tf;           // 滤波器的时间常数
		float alpha;				// 滤波器的平滑系数
} LowPassFilter;

extern void initLowPassFilter(LowPassFilter* filter, float Tf) ;
extern float lowPassFilter(LowPassFilter* filter, float x, float dt);
#endif
