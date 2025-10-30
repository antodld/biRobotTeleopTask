#pragma once
#include <mc_control/ControllerClient.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/gui/RobotMsg.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/io_utils.h>

#include "HumanRobotPose.h"
#include <any>
#include <mutex>
#include <thread>

struct SubscridedbId
{
  SubscridedbId() = default;
  SubscridedbId(const mc_rtc::gui::Elements type, const std::vector<std::string> & category, const std::string & name)
  : type_(type), id_(category, name)
  {
  }
  mc_rtc::gui::Elements type_;
  mc_control::ElementId id_;

  bool checkId(const mc_control::ElementId id)
  {
    if(id.category.size() != id_.category.size())
    {
      return false;
    }
    if(id.name != id_.name)
    {
      return false;
    }
    for(int i = 0; i < id.category.size(); i++)
    {
      if(id.category[i] != id_.category[i])
      {
        return false;
      }
    }
    return true;
  }
};

template<typename T>
bool checkDataType(const SubscridedbId & sub_id)
{
  using Elements = mc_rtc::gui::Elements;
  switch(sub_id.type_)
  {
    case Elements::ArrayLabel:
      return typeid(T) == typeid(Eigen::VectorXd);
    case Elements::Label:
      return typeid(T) == typeid(std::string);
    case Elements::Transform:
      return typeid(T) == typeid(sva::PTransformd);
    case Elements::Checkbox:
      return typeid(T) == typeid(bool);
    case Elements::Robot:
      return typeid(T) == typeid(mc_rtc::gui::RobotMsgData);
    default:
      return true;
  }
}

namespace biRobotTeleop
{

class HumanRobotDataReceiver : public mc_control::ControllerClient
{

public:
  HumanRobotDataReceiver() : mc_control::ControllerClient() {}

  void deactivate()
  {
    stop();
  }

  void init(const std::string name,
            const std::string human_name,
            const std::string robot_name,
            const std::string & pub,
            const std::string & rec);

  void subsbscribe(const std::string & sub_name,
                   const mc_rtc::gui::Elements type,
                   const std::vector<std::string> & category,
                   const std::string & name)
  {
    subscribed_id_[sub_name] = SubscridedbId(type, category, name);
  }

  void unsubscribe(const std::string & sub_name)
  {
    subscribed_id_.erase(sub_name);
  }

  template<typename T>
  void getSubscribedData(T & data, const std::string & sub_name) const
  {
    if(subscribed_id_.find(sub_name) != subscribed_id_.end())
    {
      if(!checkDataType<T>(subscribed_id_.at(sub_name)))
      {
        mc_rtc::log::error("Incorrect required data type");
        return;
      }
      if(subscribed_data_.find(sub_name) != subscribed_data_.end())
      {
        data = std::any_cast<T>(subscribed_data_.at(sub_name));
      }
    }
    else
    {
      mc_rtc::log::error("sub name {} is not registered", sub_name);
    }
  }

  const HumanPose & getHumanPose() const noexcept
  {
    return h_;
  }

  const std::string robotName() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_robot_);
    return robots_->robot().name();
  }

  /**
   * Get a copy of the current robots
   *
   * This is inefficient, prefer using \ref updateRobot(mc_rbdyn::Robot & extRobot, bool acc) or \ref getRobotMsg
   * instead
   */
  const mc_rbdyn::RobotsPtr getRobots()
  {
    std::lock_guard<std::mutex> lock(this->mutex_robot_);
    auto robotsCtl = mc_rbdyn::Robots::make();
    robots_->copy(*robotsCtl);
    return robotsCtl;
  }

  /**
   * Get a copy of the last received robotMsg
   */
  const mc_rtc::gui::RobotMsgData getRobotMsg() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_robot_);
    return robotMsg;
  }

  /**
   * Update an external Robot instance from the received data. This is thread-safe
   */
  void updateRobot(mc_rbdyn::Robot & extRobot, bool updateFK = true, bool acc = false)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_robot_);
      const auto & robot = robots_->robot();

      // Ensure that the received robot module and the local robot module are the same
      if(extRobot.module().parameters() != robot.module().parameters())
      {
        mc_rtc::log::error_and_throw(
            "[{}] Received robot module parameters for robot {} ([{}]) do not match local robot module parameters "
            "([{}]). Fix either your mc_rtc.yaml configuration or BiRobotTeleoperation.yaml",
            name_, robot.name(), mc_rtc::io::to_string(robot.module().parameters()),
            mc_rtc::io::to_string(extRobot.module().parameters()));
      }

      const auto & msg = robotMsg;
      extRobot.mbc().q = rbd::vectorToParam(robot.mb(), msg.q);
      extRobot.mbc().alpha = rbd::vectorToDof(robot.mb(), msg.alpha);
      if(acc)
      {
        extRobot.mbc().alphaD = rbd::vectorToDof(robot.mb(), msg.alphaD);
      }
      extRobot.posW(msg.posW);
    }

    if(updateFK)
    {
      extRobot.forwardKinematics();
      extRobot.forwardVelocity();
      if(acc)
      {
        extRobot.forwardAcceleration();
      }
    }
  }

  const std::string name() const noexcept
  {
    return human_name_;
  }

  bool online() const noexcept
  {
    return online_;
  }

  void startConnection();

  void setSimulatedDelay(const double d)
  {
    simulated_delay_ = d;
  }

  void runAndUpdate();

  void update();

private:
  void form_string_input(const mc_control::ElementId & /*id*/,
                         const std::string & /*value*/,
                         bool /*required*/,
                         const std::string & /*default*/) override
  {
  }

  void transform(const mc_control::ElementId & id,
                 const mc_control::ElementId & /*requestId*/,
                 bool /*ro */,
                 const sva::PTransformd & X) override;

  void button(const mc_control::ElementId &) override {}

  void checkbox(const mc_control::ElementId & id, bool s) override;

  void form(const mc_control::ElementId &) override {}

  void force(const mc_control::ElementId &,
             const mc_control::ElementId & /*requestId*/,
             const sva::ForceVecd & /* force */,
             const sva::PTransformd & /* application point for the force */,
             const mc_rtc::gui::ForceConfig & /* forceConfig */,
             bool /* ro */) override
  {
  }

  inline void array_label(const mc_control::ElementId &,
                          const std::vector<std::string> &,
                          const Eigen::VectorXd &) override;

  inline void label(const mc_control::ElementId &, const std::string &) override;

  /** Called when a table starts */
  void table_start(const mc_control::ElementId &, const std::vector<std::string> & /*header*/) override {}

  /** Called for each element appearing in the table identified by \p id */
  void table_row(const mc_control::ElementId & /*id*/, const std::vector<std::string> & /*data*/) override {}

  /** Called to close a table identified by \p id */
  void table_end(const mc_control::ElementId & /*id*/) override {}

  void number_input(const mc_control::ElementId &, double /*data*/) override {}

  void array_input(const mc_control::ElementId &,
                   const std::vector<std::string> & /*labels*/,
                   const Eigen::VectorXd & /*data*/) override;

  void number_slider(const mc_control::ElementId &, double /*data*/, double /*min*/, double /*max*/) override {}

  void robot(const mc_control::ElementId & /*id*/,
             const std::vector<std::string> & /*parameters*/,
             const std::vector<std::vector<double>> & /*q*/,
             const sva::PTransformd & /*posW*/) override
  {
  }

  void robot_msg(const mc_control::ElementId & id, const mc_rtc::gui::RobotMsgData & msg) override;

  void schema(const mc_control::ElementId &, const std::string & /*schema*/) override {}

  void arrow(const mc_control::ElementId &,
             const mc_control::ElementId & /* requestId */,
             const Eigen::Vector3d & /* start */,
             const Eigen::Vector3d & /* end */,
             const mc_rtc::gui::ArrowConfig & /* config */,
             bool /* ro */) override
  {
  }

  void point3d(const mc_control::ElementId &,
               const mc_control::ElementId & /*requestId*/,
               bool /*ro */,
               const Eigen::Vector3d & /*pos*/,
               const mc_rtc::gui::PointConfig & /* config */) override
  {
  }

  void rotation(const mc_control::ElementId &,
                const mc_control::ElementId & /*requestId*/,
                bool /*ro */,
                const sva::PTransformd & /*pos*/) override
  {
  }

  void trajectory(const mc_control::ElementId &,
                  const std::vector<Eigen::Vector3d> & /* points */,
                  const mc_rtc::gui::LineConfig & /* config */) override
  {
  }

  void updateRobot(mc_rbdyn::Robot & robot, const mc_rtc::gui::RobotMsgData & msg);

  void updateRobot(mc_rbdyn::Robot & robot_target, const mc_rbdyn::Robot & robot_data);

  std::string human_name_;
  std::string robot_name_;
  std::string name_;
  HumanPose h_;
  HumanPose h_thread_;

  mc_rtc::gui::RobotMsgData robotMsg;
  mc_rtc::gui::RobotMsgData robotMsg_thread;

  std::map<std::string, std::any> subscribed_data_;
  std::map<std::string, SubscridedbId> subscribed_id_;

  mc_rbdyn::RobotsPtr robots_ = nullptr; // robots instance updated from the GUI thread
  int online_count_ = 1e4;
  bool online_ = false; // true if connected to server
  bool online_thread_ = false;

  double simulated_delay_ = 0.1;
  mutable std::mutex mutex_robot_;
  mutable std::mutex mutex_copy_;
  std::thread robot_thread_;
};

} // namespace biRobotTeleop
