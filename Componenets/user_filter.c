#include "user_filter.h"

// ��ʼ����ͨ�˲���
void initLowPassFilter(LowPassFilter* filter, float Tf) 
{
    filter->y_prev = 0.0f;
    filter->Tf = Tf;
}
// �ٶȵ�ͨ�˲�����
float lowPassFilter(LowPassFilter* filter, float x, float dt)
{
    static float y,alpha;
    if (dt < 0.0f)
    {
        dt = 1e-3f;
    }
    else if (dt > 0.3f)
    {
        filter->y_prev = x;
        return x;
    }
    alpha = filter->Tf / (filter->Tf + dt);
    y = alpha * filter->y_prev + (1.0f - alpha) * x;
    filter->y_prev = y;
    return y;
}