#ifndef PARAM_HPP
#define PARAM_HPP

//PlayEngine 的参数
const uint32_t SpecLen=200; //频域点数  width
const uint32_t ObserveTime=200; //观察时间  height
const uint32_t ActionNum=8; //策略总数
const uint32_t STEP_OVER=10; //频率决策时间相基准时间的倍数

//DQN 的参数
const int Memory_Size=50000; //回放空间的大小
const int Memory_TH=100; //进行更新运算的最小回放步数
const double Gamma=0.8; //长期回报的折扣
const int Iter_Explore=5000;//多少迭代次数后，大概率选取贪婪策略
#endif



