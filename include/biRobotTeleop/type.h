#pragma once
#include <iostream>
namespace biRobotTeleop
{
enum Limbs
{
  Head,
  Pelvis,
  LeftHand,
  LeftForearm,
  LeftArm,
  RightHand,
  RightForearm,
  RightArm
};

inline Limbs str2Limb(const std::string & name)
{

  if(name == "LeftHand")
  {
    return Limbs::LeftHand;
  }
  if(name == "RightHand")
  {
    return Limbs::RightHand;
  }
  if(name == "LeftArm")
  {
    return Limbs::LeftArm;
  }
  if(name == "RightArm")
  {
    return Limbs::RightArm;
  }
  if(name == "LeftForearm")
  {
    return Limbs::LeftForearm;
  }
  if(name == "RightForearm")
  {
    return Limbs::RightForearm;
  }
  if(name == "Pelvis")
  {
    return Limbs::Pelvis;
  }
  if(name == "Head")
  {
    return Limbs::Head;
  }

  std::cout << "[str2Limb] Link " << name << " not recognized : Available links :\n"
            << "LeftHand\nRightHand\nLeftArm\nRightArm\nLeftForearm\nRightForearm\nPelvis\nHead" << std::endl;
  return Limbs::LeftHand;
}

inline std::string limb2Str(const Limbs & limb)
{

  if(limb == LeftHand)
  {
    return "LeftHand";
  }
  if(limb == RightHand)
  {
    return "RightHand";
  }
  if(limb == LeftArm)
  {
    return "LeftArm";
  }
  if(limb == RightArm)
  {
    return "RightArm";
  }
  if(limb == RightForearm)
  {
    return "RightForearm";
  }
  if(limb == LeftForearm)
  {
    return "LeftForearm";
  }
  if(limb == Pelvis)
  {
    return "Pelvis";
  }
  if(limb == Head)
  {
    return "Head";
  }

  std::cout << "[limb2Str] Link " << limb << " not recognized : Available links :\n"
            << "LeftHand\nRightHand\nLeftArm\nRightArm\nLeftForearm\nRightForearm\nPelvis\nHead" << std::endl;
  return "LeftHand";
}

enum Type
{
  Human,
  Robot
};
enum Name
{
  A,
  B
};
} // namespace biRobotTeleop
