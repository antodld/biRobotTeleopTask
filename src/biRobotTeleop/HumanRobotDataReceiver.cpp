#include "../../include/biRobotTeleop/HumanRobotDataReceiver.h"
#include <chrono>

namespace biRobotTeleop
{


void HumanRobotDataReceiver::init(const std::string human_name, const std::string robot_name ,const std::string & pub, const std::string rec) 
{
    human_name_ = human_name;
    robot_name_ = robot_name;
    h_ = HumanPose(human_name_);
    robots_ = mc_rbdyn::Robots::make();
    robots_->load("initial",*mc_rbdyn::RobotLoader().get_robot_module({"JVRC1"}));
    connect(rec,pub);
    startConnection();
}

void HumanRobotDataReceiver::startConnection()
{
  run_ = true;
#ifndef MC_RTC_DISABLE_NETWORK
  sub_th_ = std::thread(
      [this]()
      {
        std::vector<char> buff(65536);
        auto t_last_received = std::chrono::system_clock::now();
        while(run_)
        {
          online_count_ +=1;
          run(buff, t_last_received);
          online_ = online_count_ < 1000;
          std::this_thread::sleep_for(std::chrono::microseconds(500));
          if(robot_thread_.joinable())
          {
            robot_thread_.join();
          }
        }
      });
#endif
}

void HumanRobotDataReceiver::robot(const mc_control::ElementId & id,const mc_control::RobotMsg & msg)
{
    if(id.category[0] == "Robots")
    {
        if(id.name == robot_name_)
        {
            if(robots_->robot(0).name() != robot_name_)
            {
                std::cout << "update robot name " << std::endl;
                robots_->load( robot_name_,*mc_rbdyn::RobotLoader().get_robot_module(msg.parameters) );
                robots_->removeRobot(0);

            }
            if(simulated_delay_ != 0)
            {
                robot_thread_ = std::thread(&HumanRobotDataReceiver::updateRobot,this,msg);
            }
            else
            {
                updateRobot(msg);
            }
        }   
    }
    for (auto & subs : subscribed_id_)
    {
        if(subs.second.checkId(id))
        {
            subscribed_data_[subs.first] = msg;
        }
    }

}

void HumanRobotDataReceiver::checkbox(const mc_control::ElementId & id, bool s)
{
    if(id.category[0] == "BiRobotTeleop" && id.name == "Online")
    {
        online_count_ = 0;
    }
    for (auto & subs : subscribed_id_)
    {
        if(subs.second.checkId(id))
        {
            subscribed_data_[subs.first] = s;
        }
    }
} 

void HumanRobotDataReceiver::transform(const mc_control::ElementId & id,
                        const mc_control::ElementId & /*requestId*/,
                        bool /*ro */,
                        const sva::PTransformd & X)
{
    if (id.category.size() == 4)
    {
        if(id.category[1] == "Human Pose" && id.category[2] == human_name_ && id.category[3] == "Transforms")
        {
            auto limb = str2Limb(id.name.substr(0,id.name.find("_transform")));
            h_.setPose(limb,X);
        }
    }
    for (auto & subs : subscribed_id_)
    {
        if(subs.second.checkId(id))
        {
            subscribed_data_[subs.first] = X;
        }
    }

}

void HumanRobotDataReceiver::label(const mc_control::ElementId & id , const std::string & label)
{
    // const mc_control::ElementId id_f(id.category,id.name.substr(0,id.name.find("_transform")))
    for (auto & subs : subscribed_id_)
    {
        if(subs.second.checkId(id))
        {
            subscribed_data_[subs.first] = label;
        }
    }
}

void HumanRobotDataReceiver::array_label(const mc_control::ElementId & id, const std::vector<std::string> &, const Eigen::VectorXd & data)
{

    for (auto & subs : subscribed_id_)
    {
        if(subs.second.checkId(id))
        {
            subscribed_data_[subs.first] = data;
        }
    }
}

void HumanRobotDataReceiver::updateRobot(const mc_control::RobotMsg & msg)
{
    const int dur = static_cast<int>(simulated_delay_ * 1e3);
    std::this_thread::sleep_for(std::chrono::milliseconds(dur));
    {
    std::lock_guard<std::mutex> lk_copy_state(mutex_robot_);
    auto & robot = robots_->robot(0);
    robot.mbc().q = msg.q;
    robot.mbc().alpha = msg.alpha;
    robot.posW(msg.posW);
    }
}


} //namespace{biRobotTeleop}