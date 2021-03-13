
#include "device/IntelligentDecision.h"


IntelligentDecision::IntelligentDecision(std::vector<short> &data)
{
    if (data.empty()) return;

    _max_num = *(max_element(data.begin(), data.end()));  //找到数组中最大值

    std::vector<float> envelope(data.size());   //用来存储包络的临时数组

    envelope[0] = (float)data[0];
    for (size_t i = 1; i < data.size(); ++i)
    {
        float delta=(float)data[i]-envelope[i-1];

        if(delta<0)
            envelope[i]=(data[i]);
        else
            envelope[i]=envelope[i-1]+(float )_max_num/15000;//最大值/15000一个校准的参数

    }



    Smooth smooth(envelope,2000);   //滑动平均

    //TODO 识别频谱图信号位置,并且返回信号位置的频率。

}