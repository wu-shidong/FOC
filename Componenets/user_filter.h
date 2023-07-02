#ifndef __USER_FILTER_H
#define __USER_FILTER_H

typedef struct {
    float y_prev;       // ��һ�ε��˲����
    float Tf;           // �˲�����ʱ�䳣��
		float alpha;				// �˲�����ƽ��ϵ��
} LowPassFilter;

extern void initLowPassFilter(LowPassFilter* filter, float Tf) ;
extern float lowPassFilter(LowPassFilter* filter, float x, float dt);
#endif
