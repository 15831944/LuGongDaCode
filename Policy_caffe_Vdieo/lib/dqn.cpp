#include "dqn.hpp"
#include <algorithm>
#include <iostream>
#include <cassert>
#include <sstream>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <glog/logging.h>
#include "prettyprint.hpp"

namespace dqn{


std::string PrintQValues(
    const std::vector<float>& q_values, const ActionVect& actions)
{
	assert(!q_values.empty());
	assert(!actions.empty());
	assert(q_values.size() == actions.size());
	std::ostringstream q_values_buf;

	for (auto i = 0; i < q_values.size(); ++i) {

		const auto q_str = std::to_string(q_values[i]);
		q_values_buf << q_str<<" ";
	}
	q_values_buf << std::endl;
	return q_values_buf.str();
}



void DQN::Initialize(){
	// Initialize net and solver
	caffe::SolverParameter solver_param;
	//caffe::ReadProtoFromTextFileOrDie(solver_param_, &solver_param);
	caffe::ReadSolverParamsFromTextFileOrDie(solver_param_, &solver_param);
	//solver_.reset(caffe::GetSolver<float>(solver_param));
    solver_.reset(::caffe::SolverRegistry<float>::CreateSolver(solver_param));

	std::cout<<((solver_param.solver_mode()==0) ? "CPU":"GPU")<<std::endl;

    //加载已经训练好的模型
//    caffe::Net<float> net("./dqn.prototxt",caffe::TRAIN);
//
//    net.CopyTrainedLayersFrom("./snapshort/train/dqn_solver_iter_12000.caffemodel");

    net_ = solver_->net();

    if(!train)
        //net_->CopyTrainedLayersFromBinaryProto("./snapshort/train/dqn_solver_iter_12000.caffemodel");
        net_->CopyTrainedLayersFrom("./snapshort/train/dqn_solver_iter_6000.caffemodel");


	// 或者Q值表
	q_values_blob_ = net_->blob_by_name("q_values");

	// dummy 数据初始化
	std::fill(dummy_input_data_.begin(), dummy_input_data_.end(), 0.0);

	// 获得输入口
	frames_input_layer_ =
		boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name("frames_input_layer"));
	assert(frames_input_layer_);

	//格式确认
	assert(HasBlobSize(*net_->blob_by_name("frames"),
		kMinibatchSize,
		kInputFrameCount,
		kFrameHeight,
		kFrameWidth));



	target_input_layer_ =
		boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name("target_input_layer"));
	assert(target_input_layer_);

	//格式确认
	assert(HasBlobSize(*net_->blob_by_name("target"), kMinibatchSize, kOutputCount, 1, 1));

	filter_input_layer_ =
		boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net_->layer_by_name("filter_input_layer"));
	assert(filter_input_layer_);

	//格式确认
	assert(HasBlobSize(
	*net_->blob_by_name("filter"), kMinibatchSize, kOutputCount, 1, 1));

}


//epsilon 随机选择策略还是贪婪选择策略的概率
Action DQN::SelectAction(const InputFrames& last_frames, const double epsilon) {
  assert(epsilon >= 0.0 && epsilon <= 1.0);

  Action action;


  if (std::uniform_real_distribution<>(0.0, 1.0)(random_engine) < epsilon) {
    //按照均匀分布随机选择策略
    const auto random_idx =std::uniform_int_distribution<int>(0, legal_actions_.size() - 1)(random_engine);
    action = legal_actions_[random_idx];
    //printf("random action=%d\n",action);

  } else {
	action = SelectActionGreedily(last_frames).first; //which return is <action reward>, first get action
    //printf("greedy action=%d\n",action);
  }

  //std::cout << " epsilon:" << epsilon << std::endl;
  return action;
}


AQPair DQN::SelectActionGreedily(const InputFrames& last_frames)
{
//	std::cout<<" SelectActionGreedily"<<std::endl;
	std::vector<InputFrames> last_frames_batch;

	last_frames_batch.push_back(last_frames); //只有一个帧，其他31个为空

	return SelectActionGreedilyBatch(last_frames_batch).front();
}


std::vector<AQPair> DQN::SelectActionGreedilyBatch(const std::vector<InputFrames>& last_frames_batch)
{
	assert(last_frames_batch.size() <= kMinibatchSize);
	std::array<float, kMinibatchDataSize> frames_input;

	//输入数据 batchsize *framedata
	for (auto i = 0; i < last_frames_batch.size(); ++i) {
		const auto& frame_data = last_frames_batch[i];
		std::copy(frame_data->begin(),frame_data->end(),frames_input.begin() + i * kInputDataSize);
	}

	//进行一次前向网络运算
	InputDataIntoLayers(frames_input, dummy_input_data_, dummy_input_data_);
	net_->ForwardPrefilled(nullptr);




	//记录输出结果
	std::vector<AQPair> AQVector;
	AQVector.reserve(last_frames_batch.size());

	//
	for (auto i = 0; i < last_frames_batch.size(); ++i)
	{
		std::vector<float> q_values(legal_actions_.size());
		const auto action_evaluator = [&](Action action) {
			const auto q = q_values_blob_->data_at(i, static_cast<int>(action), 0, 0);
			assert(!std::isnan(q));
			return q;
		};

		//
		std::transform(legal_actions_.begin(),legal_actions_.end(),q_values.begin(),action_evaluator);



		if (last_frames_batch.size() == 1) {
//			std::cout << PrintQValues(q_values, legal_actions_);
		}

		//找到最大值元素位置
		const auto max_idx =
        std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
        AQVector.emplace_back(legal_actions_[max_idx], q_values[max_idx]);

	}
	return AQVector;
}







void DQN::InputDataIntoLayers(
      const FramesLayerInputData& frames_input,
      const TargetLayerInputData& target_input,
      const FilterLayerInputData& filter_input)
{
	frames_input_layer_->Reset(
		const_cast<float*>(frames_input.data()),
		dummy_input_data_.data(),
		kMinibatchSize);
	target_input_layer_->Reset(
		const_cast<float*>(target_input.data()),
		dummy_input_data_.data(),
		kMinibatchSize);
	filter_input_layer_->Reset(
		const_cast<float*>(filter_input.data()),
		dummy_input_data_.data(),
		kMinibatchSize);
}


void DQN::AddTransition(const Transition& transition)
{
	if (replay_memory_.size() == replay_memory_capacity_) replay_memory_.pop_front();
	replay_memory_.push_back(transition);
}

//网络更新，整个DQN的核心部分，也是最难理解的部分
void DQN::Update()
{
	std::cout << "iteration: " << current_iter_++ << std::endl;

	//参与update的回放记录向量
	std::vector<int> TransSelectedVector;
	TransSelectedVector.reserve(kMinibatchSize);


	//随机选取回放记录库中的回放记录
	for (auto i = 0; i < kMinibatchSize; ++i)
	{
		const auto random_transition_idx =
			std::uniform_int_distribution<int>(0, replay_memory_.size() - 1)(
				random_engine);
		TransSelectedVector.push_back(random_transition_idx);
	}


	//构造下一步的输入 其实S' 这里就得到 S' batches
	std::vector<InputFrames> target_last_frames_batch;
	for (const auto idx : TransSelectedVector)
	{
		const auto& transition = replay_memory_[idx]; //获得对应的回放记录
		InputFrames target_last_frames=std::get<3>(transition).get(); // 获得 S'
		target_last_frames_batch.push_back(target_last_frames);
	}

	//获得S' 输入是的最大 Q(S',a')
	std::vector<AQPair> actions_and_values
		=SelectActionGreedilyBatch(target_last_frames_batch);


	//构建训练网的输入输出来更新网络参数
	auto frames_input= std::make_shared<FramesLayerInputData>();
	TargetLayerInputData target_input;
	FilterLayerInputData filter_input;
	std::fill(target_input.begin(), target_input.end(), 0.0f);
	std::fill(filter_input.begin(), filter_input.end(), 0.0f);

	auto target_value_idx = 0;


	for (auto i = 0; i < kMinibatchSize; ++i) {
		const auto& transition = replay_memory_[TransSelectedVector[i]]; //获得回放记录条目 s a r s'
		const auto action = std::get<1>(transition); //获取 a

		assert(static_cast<int>(action) < kOutputCount);
		const auto reward = std::get<2>(transition); //获取 r

		//assert(reward >= -1.0 && reward <= 1.0);

		//完成 Q(s,a)=r+gamma*Q(s',a')
		const auto target = reward + gamma_ * actions_and_values[target_value_idx++].second;


		assert(!std::isnan(target));

		//构造网络输出，filter 过滤掉无效的策略，在本例子中没有用到，暂时保留
		target_input[i * kOutputCount + static_cast<int>(action)] = target;
		filter_input[i * kOutputCount + static_cast<int>(action)] = 1;


		//构造网络输入

		const auto& frame_data = std::get<0>(transition);
		std::copy(
			frame_data->begin(),
			frame_data->end(),
			frames_input->begin() + i * kInputDataSize);


	}

	//进行一次网络训练
	InputDataIntoLayers(*frames_input, target_input, filter_input);
	solver_->Step(1);


}



template <typename Dtype>
bool HasBlobSize(
    const caffe::Blob<Dtype>& blob,
    const int num,
    const int channels,
    const int height,
    const int width)
{
  return blob.num() == num &&
      blob.channels() == channels &&
      blob.height() == height &&
      blob.width() == width;
}


//FrameDataSp PreprocessScreen(std::deque<Spectrum_data> & Waterfall) //1024*20 -> 1*20480
//{
//	auto screen = std::make_shared<FrameData>();
//
//	for (auto i = 0; i < kFrameHeight; ++i) {
//		for (auto j = 0; j < kFrameWidth; ++j){
//			(*screen)[i * kFrameWidth + j]=Waterfall[i][j];
//		}
//	}
//	return screen;
//}

}
