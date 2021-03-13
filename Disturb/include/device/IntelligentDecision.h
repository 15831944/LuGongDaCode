/*****
 * 输入频点信息
 * 在频谱中找出信号所在的位置
 * */
#pragma once

#include <iostream>
#include <fstream>
#include <deque>
#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <glog/logging.h>

#include <common/GNUPlot.h>


/*智能干扰决策*/
class IntelligentDecision
{
public:
    IntelligentDecision(std::vector<short> &data);
private:
    void Envelope();
private:
    int _max_num = -10000;



};



/**************************************************************************************/

/*滑动平均窗*/
class Smooth
{
public:
    Smooth(std::vector<float> &data,uint win_len)
    {
        uint win_size = win_len;

        if(data.size()<win_size )
        {
            LOG(INFO)<<"data.size<win_size";
            return ;
        }
        if(win_size <2)
        {
            LOG(INFO)<<"win_size <2";
            return ;
        }
        if(win_size%2 != 0)
        {
            LOG(INFO)<<"win长度为奇数， win size -1";
            win_size = win_size-1;
        }


        int size = data.size();  // 输入数据的长度，保存
        std::deque<float> win;   //滑动平均窗
        float sum = 0;           //窗的累加值
        float avr ;              //为整个data数组的平均值

        avr = std::accumulate(data.begin(),data.end(),0)/data.size();//求累加和
        //先把初始化窗的前半数据为平均值
        for (int i = 0; i < win_size/2; ++i)
            win.push_back(avr);
        //窗的后半数据为data数组里的数据
        for (int i = 0; i < win_size/2; ++i)
            win.push_back(data[i]);
        //初始化完窗以后对窗求累加和
        for (short j : win)
            sum = sum + j;
        //给输入数组后部补一些值，理论上补0也行
        for (int k = 0; k < win_size/2 ; ++k)
            data.push_back(avr);

        //开始求滑动平均
        for (int n = 0; n <size ; ++n)
        {
            data[n] = sum/win_size;

            sum = sum - win.front();
            win.pop_front();

            win.push_back(data[n+1]);
            sum = sum + data[n+1];
        }

        data.resize(size);
    }

};




