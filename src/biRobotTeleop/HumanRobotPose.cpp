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
                                [this,limb] () -> const sva::PTransformd 
                                {
                                    return getOffset(limb) * getPose(limb);
                                } )
                        );
        }
    }
}



}