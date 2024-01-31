#pragma once
#include <mc_control/ControllerClient.h>
#include "HumanRobotPose.h"
#include <mc_rtc/io_utils.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/RobotMsg.h>
#include <any>
#include <mutex>
#include <thread>


struct SubscridedbId
{
    SubscridedbId() = default;
    SubscridedbId( const mc_rtc::gui::Elements type,const std::vector<std::string> & category,const std::string & name)
    : type_(type) ,id_(category,name) {}
    mc_rtc::gui::Elements type_;
    mc_control::ElementId id_;

    bool checkId(const mc_control::ElementId id)
    {
        if(id.category.size() != id_.category.size()){return false;}
        if(id.name != id_.name){return false;}
        for (int i = 0 ; i < id.category.size() ; i++)
        {
            if (id.category[i] != id_.category[i]){return false;} 
        }
        return true;
    }
};


template <typename T>
bool checkDataType(const SubscridedbId & sub_id)
{
    using Elements = mc_rtc::gui::Elements;
    switch (sub_id.type_)
    {
    case Elements::ArrayLabel :
        return typeid(T) == typeid(Eigen::VectorXd);
    case Elements::Label :
        return typeid(T) == typeid(std::string);
    case Elements::Transform :
        return typeid(T) == typeid(sva::PTransformd);
    case Elements::Checkbox :
        return typeid(T) == typeid(bool);
    case Elements::Robot :
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

    HumanRobotDataReceiver() : mc_control::ControllerClient()
    {
        
    }

    void init(const std::string human_name, const std::string robot_name ,const std::string & pub, const std::string & rec);

    void subsbscribe(const std::string & sub_name, const mc_rtc::gui::Elements type,const std::vector<std::string> & category, const std::string & name)
    {
        subscribed_id_[sub_name] = SubscridedbId(type,category,name);
    }

    void unsubscribe(const std::string & sub_name)
    {
        subscribed_id_.erase(sub_name);
    }

    template <typename T>
    void getSubscribedData(T & data,const std::string & sub_name) const
    {
        if(subscribed_id_.find(sub_name) != subscribed_id_.end() )
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
            mc_rtc::log::error("sub name {} is not registered",sub_name);
        }
    }

    const HumanPose & getHumanPose()
    {
        std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
        return h_;
    }

    const mc_rbdyn::Robot & getRobot()
    {
        std::lock_guard<std::mutex> lk_copy_state_delay(mutex_robot_);
        std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
        return robots_->robot(0);
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


private:

    void form_string_input(const mc_control::ElementId & /*id*/,
                                    const std::string & /*value*/,
                                    bool /*required*/,
                                    const std::string & /*default*/) override {}

    void transform(const mc_control::ElementId & id,
                         const mc_control::ElementId & /*requestId*/,
                         bool /*ro */,
                         const sva::PTransformd & X) override ;

    void button(const mc_control::ElementId & ) override {}

    void checkbox(const mc_control::ElementId & id, bool s) override;

    void form(const mc_control::ElementId & ) override {}

    void force(const mc_control::ElementId & ,
                     const mc_control::ElementId & /*requestId*/,
                     const sva::ForceVecd & /* force */,
                     const sva::PTransformd & /* application point for the force */,
                     const mc_rtc::gui::ForceConfig & /* forceConfig */,
                     bool /* ro */) override
                {}
    
    inline void array_label(const mc_control::ElementId & , const std::vector<std::string> &, const Eigen::VectorXd &) override;

    inline void label(const mc_control::ElementId & , const std::string &) override;

    /** Called when a table starts */
    void table_start(const mc_control::ElementId & , const std::vector<std::string> & /*header*/) override {}

    /** Called for each element appearing in the table identified by \p id */
    void table_row(const mc_control::ElementId & /*id*/, const std::vector<std::string> & /*data*/) override {}

    /** Called to close a table identified by \p id */
    void table_end(const mc_control::ElementId & /*id*/) override {}

    void number_input(const mc_control::ElementId & , double /*data*/) override {}

    void array_input(const mc_control::ElementId & ,
                            const std::vector<std::string> & /*labels*/,
                            const Eigen::VectorXd & /*data*/) override;

    void number_slider(const mc_control::ElementId & , double /*data*/, double /*min*/, double /*max*/) override {}

    void robot(const mc_control::ElementId & id,
                     const std::vector<std::string> & /*parameters*/,
                     const std::vector<std::vector<double>> & /*q*/,
                     const sva::PTransformd & /*posW*/) override {}

    void robot_msg(const mc_control::ElementId & id, const mc_rtc::gui::RobotMsgData & msg) override;

    void schema(const mc_control::ElementId & , const std::string & /*schema*/) override {}

    void arrow(const mc_control::ElementId & ,
                        const mc_control::ElementId & /* requestId */,
                        const Eigen::Vector3d & /* start */,
                        const Eigen::Vector3d & /* end */,
                        const mc_rtc::gui::ArrowConfig & /* config */,
                        bool /* ro */) override {}

    void point3d(const mc_control::ElementId & ,
                        const mc_control::ElementId & /*requestId*/,
                        bool /*ro */,
                        const Eigen::Vector3d & /*pos*/,
                        const mc_rtc::gui::PointConfig & /* config */) override {}

    void rotation(const mc_control::ElementId & ,
                            const mc_control::ElementId & /*requestId*/,
                            bool /*ro */,
                            const sva::PTransformd & /*pos*/) override {}

    void trajectory(const mc_control::ElementId & ,
                            const std::vector<Eigen::Vector3d> & /* points */,
                            const mc_rtc::gui::LineConfig & /* config */) override {}
    

    void updateRobot(mc_rbdyn::Robot & robot, const mc_rtc::gui::RobotMsgData & msg);

    void updateRobot(mc_rbdyn::Robot & robot_target, const mc_rbdyn::Robot & robot_data);

    std::string human_name_;
    std::string robot_name_;
    HumanPose h_;
    HumanPose h_thread_;

    std::map<std::string,std::any> subscribed_data_;
    std::map<std::string,SubscridedbId> subscribed_id_;
    
    mc_rbdyn::RobotsPtr robots_ = nullptr;
    int online_count_ = 100;
    bool online_ = false; // true if connected to server

    double simulated_delay_ = 0.1;
    std::mutex mutex_robot_;
    std::mutex mutex_copy_;
    std::thread robot_thread_;


};

}