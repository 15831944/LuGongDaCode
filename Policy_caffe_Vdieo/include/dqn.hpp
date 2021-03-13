#ifndef DQN_HPP
#define DQN_HPP

#include <memory>
#include <random>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <caffe/caffe.hpp>
#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>
#include "Param.hpp"
//#include "PlayEngine.hpp"
#include "Policy.hpp"
#include <caffe/layers/memory_data_layer.hpp>
namespace dqn{
	
constexpr auto kRawFrameHeight =ObserveTime;  //原始画面高度
constexpr auto kRawFrameWidth =SpecLen; //原始换面宽度

constexpr auto kFrameWidth = SpecLen; //压缩后的尺寸，如果200就是不用压缩 需要和solver 一致
constexpr auto kFrameHeight = ObserveTime;
constexpr auto kCroppedFrameDataSize =kFrameWidth * kFrameHeight;


constexpr auto kInputFrameCount = 1; //
constexpr auto kInputDataSize = kCroppedFrameDataSize * kInputFrameCount;
constexpr auto kMinibatchSize = 32;
constexpr auto kMinibatchDataSize = kInputDataSize * kMinibatchSize;
constexpr auto kGamma = 0.95f;

constexpr auto kOutputCount = ActionNum;



using FrameData = std::array<uint8_t, kCroppedFrameDataSize>; //Frame数据快  200×200
using FrameDataSp = std::shared_ptr<FrameData>; //FrameData 指针
//using InputFrames = std::array<FrameDataSp, kInputFrameCount>; //输入数据
using InputFrames = FrameDataSp; //输入数据就是一个 FrameDataSp 因为 kInputFrameCount=1

using Action=uint8_t;

//                              s           a        r            s'        
using Transition = std::tuple<InputFrames, Action, float, boost::optional<FrameDataSp>>;

using FramesLayerInputData = std::array<float, kMinibatchDataSize>;
using TargetLayerInputData = std::array<float, kMinibatchSize * kOutputCount>;
using FilterLayerInputData = std::array<float, kMinibatchSize * kOutputCount>;
	
	
	
	
	
	
/**
 * Deep Q-Network
 */
 
using ActionVect = std::vector<uint8_t>; //可选频率集合
using AQPair=std::pair<Action, float>; //action Qvalue

class DQN {
public:
	DQN(
		const ActionVect& legal_actions,
		const std::string& solver_param,
		const int replay_memory_capacity,
		const double gamma,
		bool train) :
		legal_actions_(legal_actions),
		solver_param_(solver_param),
		replay_memory_capacity_(replay_memory_capacity),
		gamma_(gamma),
        train(train),
		current_iter_(0),
		random_engine(0)
		{}



	//初始化模块
	void Initialize();

	//选择策略
	Action SelectAction(const InputFrames& input_frames, double epsilon);
	
	//获得一个输入的贪婪策略
	AQPair SelectActionGreedily(const InputFrames& last_frames);
	
	//获得一组输入的贪婪策略组
	std::vector<AQPair> SelectActionGreedilyBatch(const std::vector<InputFrames>& last_frames_batch);
	
	//增加回放记录
	void AddTransition(const Transition& transition);
	
	//更新网络
	void Update();
	
	//当前迭代次数，仅仅在update时会增加
	int current_iteration() const { return current_iter_; }
  
	//当前回放空间的大小
	int memory_size() const { return replay_memory_.size(); }

    bool train;

private:
	
	//主要参数
	const ActionVect legal_actions_;
	const std::string solver_param_;
	const int replay_memory_capacity_;
	const double gamma_;
	int current_iter_;
	std::mt19937 random_engine;

	
	
	//网络结构指针
	using SolverSp = std::shared_ptr<caffe::Solver<float>>;
	using NetSp = boost::shared_ptr<caffe::Net<float>>;
	using BlobSp = boost::shared_ptr<caffe::Blob<float>>;
	using MemoryDataLayerSp = boost::shared_ptr<caffe::MemoryDataLayer<float>>;
	
	//网络数据接口
	SolverSp solver_;
	NetSp net_;
	BlobSp q_values_blob_;
	MemoryDataLayerSp frames_input_layer_;
	MemoryDataLayerSp target_input_layer_;
	MemoryDataLayerSp filter_input_layer_;
	TargetLayerInputData dummy_input_data_;
	
	//回放记录
	std::deque<Transition> replay_memory_;
	
	
	//内部函数
	void InputDataIntoLayers(
		const FramesLayerInputData& frames_data,
		const TargetLayerInputData& target_data,
		const FilterLayerInputData& filter_data);


};


//FrameDataSp PreprocessScreen(std::deque<Spectrum_data> & Waterfall);

template <typename Dtype>
bool HasBlobSize(
	const caffe::Blob<Dtype>& blob,
    const int num,
    const int channels,
    const int height,
    const int width);

}

#endif /* DQN_HPP_ */
