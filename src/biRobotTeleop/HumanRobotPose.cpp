#include "../../include/biRobotTeleop/HumanRobotPose.h"
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Transform.h>

namespace biRobotTeleop
{

void HumanPose::addDataToGUI(mc_rtc::gui::StateBuilder & gui)
{
    if (!gui.hasElement({"BiRobotTeleop","Human Pose",name_},"name"))
    {
        gui.addElement( {"BiRobotTeleop","Human Pose",name_},mc_rtc::gui::Label("name",[this]() -> const std::string & { return name_;} ));
        for (int i = 0 ; i <= Limbs::RightArm;i++)
        {
            const auto limb = static_cast<Limbs>(i);
            gui.addElement( {"BiRobotTeleop","Human Pose",name_,"Transforms"},
                            mc_rtc::gui::Transform(
                                limb2Str(limb),
                                [this,limb] () -> const sva::PTransformd &
                                {
                                    return getPose(limb);
                                } )
                        );
            gui.addElement( {"BiRobotTeleop","Human Pose",name_,"Velocity"},
                            mc_rtc::gui::ArrayLabel(
                                limb2Str(limb) + "_linear",{"vx","vy","vz"},
                                [this,limb] () -> const Eigen::Vector3d &
                                {
                                    return getVel(limb).linear();
                                } ),
                            mc_rtc::gui::ArrayLabel(
                                limb2Str(limb) + "_angular",{"wx","wy","wz"},
                                [this,limb] () -> const Eigen::Vector3d &
                                {
                                    return getVel(limb).angular();
                                } )
                        );
            gui.addElement( {"BiRobotTeleop","Human Pose",name_,"Acceleration"},
                            mc_rtc::gui::ArrayLabel(
                                limb2Str(limb) + "_linear",{"vx","vy","vz"},
                                [this,limb] () -> const Eigen::Vector3d &
                                {
                                    return getAcc(limb).linear();
                                } ),
                            mc_rtc::gui::ArrayLabel(
                                limb2Str(limb) + "_angular",{"wx","wy","wz"},
                                [this,limb] () -> const Eigen::Vector3d &
                                {
                                    return getAcc(limb).angular();
                                } )
                        );
        }
    }
}

void HumanPose::addPoseToGUI(mc_rtc::gui::StateBuilder & gui)
{
    if (!gui.hasElement({"BiRobotTeleop","Human Pose",name_},"name"))
    {
        gui.addElement( {"BiRobotTeleop","Human Pose",name_},mc_rtc::gui::Label("name",[this]() -> const std::string & { return name_;} ));
        for (int i = 0 ; i <= Limbs::RightArm;i++)
        {
            const auto limb = static_cast<Limbs>(i);
            gui.addElement( {"BiRobotTeleop","Human Pose",name_,"Transforms"},
                            mc_rtc::gui::Transform(
                                limb2Str(limb),
                                [this,limb] () -> const sva::PTransformd 
                                {
                                    return getOffset(limb) * getPose(limb);
                                } )
                        );
        }
    }
}

void HumanPose::addOffsetToGUI(mc_rtc::gui::StateBuilder & gui)
{
    for (int i = 0 ; i <= Limbs::RightArm;i++)
    {
        const auto limb = static_cast<Limbs>(i);
        gui.addElement(this, {"BiRobotTeleop","Human Pose", name_,"Offsets",limb2Str(limb)},

                    mc_rtc::gui::ArrayInput(
                        "translation [m]", {"x", "y", "z"},
                        [this,limb]() -> const Eigen::Vector3d & { return getOffset(limb).translation(); },
                        [this,limb](const Eigen::Vector3d & t) {
                            auto offset = getOffset(limb); 
                            offset.translation() = t;
                            setOffset(limb,offset);
                            }),
                    mc_rtc::gui::ArrayInput(
                        "rotation [deg]", {"r", "p", "y"},
                        [this,limb]() -> Eigen::Vector3d {
                        return mc_rbdyn::rpyFromMat(getOffset(limb).rotation()) * 180. / mc_rtc::constants::PI;
                        },
                        [this,limb](const Eigen::Vector3d & rpy) {
                            auto offset = getOffset(limb); 
                            offset.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.);
                            setOffset(limb,offset);
                        })
                        );
    }
}



}