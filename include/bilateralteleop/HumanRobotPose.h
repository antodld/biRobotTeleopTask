#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/Configuration.h>
#include <bilateralTeleop_dataLink/type.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>

namespace bilateralTeleop
{

struct HumanPose
{
  sva::PTransformd X_0_Base = sva::PTransformd::Identity();
  sva::PTransformd X_0_LeftHand = sva::PTransformd::Identity();
  sva::PTransformd X_0_RightHand = sva::PTransformd::Identity();
  sva::PTransformd X_0_LeftArm = sva::PTransformd::Identity();
  sva::PTransformd X_0_RightArm = sva::PTransformd::Identity();
  sva::PTransformd X_0_LeftForeArm = sva::PTransformd::Identity();
  sva::PTransformd X_0_RightForeArm = sva::PTransformd::Identity();

  sva::MotionVecd V_Base = sva::MotionVecd::Zero();
  sva::MotionVecd V_LeftHand = sva::MotionVecd::Zero();
  sva::MotionVecd V_RightHand = sva::MotionVecd::Zero();
  sva::MotionVecd V_LeftArm = sva::MotionVecd::Zero();
  sva::MotionVecd V_RightArm = sva::MotionVecd::Zero();
  sva::MotionVecd V_LeftForeArm = sva::MotionVecd::Zero();
  sva::MotionVecd V_RightForeArm = sva::MotionVecd::Zero();

  sva::MotionVecd A_Base = sva::MotionVecd::Zero();
  sva::MotionVecd A_LeftHand = sva::MotionVecd::Zero();
  sva::MotionVecd A_RightHand = sva::MotionVecd::Zero();
  sva::MotionVecd A_LeftArm = sva::MotionVecd::Zero();
  sva::MotionVecd A_RightArm = sva::MotionVecd::Zero();
  sva::MotionVecd A_LeftForeArm = sva::MotionVecd::Zero();
  sva::MotionVecd A_RightForeArm = sva::MotionVecd::Zero();

  sch::S_Cylinder arm_cvx = sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1);
  sch::S_Cylinder forearm_cvx = sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1);
  sch::S_Cylinder hand_cvx = sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1);

  void setCvx(const mc_rtc::Configuration & config)
  {

    Eigen::Vector2d length = config("arm")("length");
    double radius = config("arm")("radius");
    Eigen::Vector3d axis = config("arm")("axis");
    arm_cvx = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

    length = config("forearm")("length");
    radius = config("forearm")("radius");
    axis = config("forearm")("axis");
    forearm_cvx = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) *  length.y(),radius);
    length = config("hand")("length");
    radius = config("hand")("radius");
    axis = config("hand")("axis");
    hand_cvx = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) *  length.y(),radius);

  }

  sch::S_Cylinder applyTransformation(const sch::S_Cylinder cvx,const sva::PTransformd & X_0_p)
  {

    sch::Point3 p1_sch = cvx.getP1();
    Eigen::Vector3d p1;
    p1 << p1_sch[0], p1_sch[1], p1_sch[2];
    sch::Point3 p2_sch = cvx.getP2();
    Eigen::Vector3d p2;
    p2 << p2_sch[0], p2_sch[1], p2_sch[2];     
    sva::PTransformd X_p_p1 = sva::PTransformd(Eigen::Matrix3d::Identity(),p1);     
    sva::PTransformd X_p_p2 = sva::PTransformd(Eigen::Matrix3d::Identity(),p2);     

    p1 = (X_p_p1 * X_0_p).translation();
    p2 = (X_p_p2 * X_0_p).translation();
    sch::Scalar r = cvx.getRadius(); 
    sch::S_Cylinder cvx_out = sch::S_Cylinder(sch::Point3(p1.x(),p1.y(),p1.z()),sch::Point3(p2.x(),p2.y(),p2.z()),r);
    
    return cvx_out;
  }

  sch::S_Cylinder getConvex(bilateralTeleop::Limbs limb)
  {

      if(limb == bilateralTeleop::Limbs::RightArm || limb == bilateralTeleop::Limbs::LeftArm)
      {
        return applyTransformation(arm_cvx,getPose(limb));
      }
      if(limb == bilateralTeleop::Limbs::RightForearm || limb == bilateralTeleop::Limbs::LeftForearm)
      {
        return applyTransformation(forearm_cvx,getPose(limb));
      }

      else
      {
        return applyTransformation(hand_cvx,getPose(limb));
      }
    
  }


  sva::PTransformd & getPose(bilateralTeleop::Limbs limb)
  {
    if (limb == bilateralTeleop::Limbs::LeftArm){return X_0_LeftArm; }
    if (limb == bilateralTeleop::Limbs::LeftForearm){return X_0_LeftForeArm ; }
    if (limb == bilateralTeleop::Limbs::LeftHand){return X_0_LeftHand ; }
    if (limb == bilateralTeleop::Limbs::RightArm){return X_0_RightArm ; }
    if (limb == bilateralTeleop::Limbs::RightForearm){return X_0_RightForeArm ; }
    if (limb == bilateralTeleop::Limbs::RightHand){return X_0_RightHand ; }
    if (limb == bilateralTeleop::Limbs::Pelvis){return X_0_Base ; }
    return X_0_Base;
  }
  sva::MotionVecd getVel(bilateralTeleop::Limbs limb, sva::PTransformd X_b_bOff = sva::PTransformd::Identity())
  {
    Eigen::Matrix3d R_0_b = getPose(limb).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    if (limb == bilateralTeleop::Limbs::LeftArm){return X_b_bOff0*V_LeftArm; }
    if (limb == bilateralTeleop::Limbs::LeftForearm){return X_b_bOff0*V_LeftForeArm ; }
    if (limb == bilateralTeleop::Limbs::LeftHand){return X_b_bOff0*V_LeftHand ; }
    if (limb == bilateralTeleop::Limbs::RightArm){return X_b_bOff0*V_RightArm ; }
    if (limb == bilateralTeleop::Limbs::RightForearm){return X_b_bOff0*V_RightForeArm ; }
    if (limb == bilateralTeleop::Limbs::RightHand){return X_b_bOff0*V_RightHand ; }
    return sva::MotionVecd::Zero();
  }
  sva::MotionVecd getAcc(bilateralTeleop::Limbs limb, sva::PTransformd X_b_bOff = sva::PTransformd::Identity())
  {
    Eigen::Matrix3d R_0_b = getPose(limb).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    if (limb == bilateralTeleop::Limbs::LeftArm){return X_b_bOff0 * A_LeftArm; }
    if (limb == bilateralTeleop::Limbs::LeftForearm){return X_b_bOff0 * A_LeftForeArm ; }
    if (limb == bilateralTeleop::Limbs::LeftHand){return X_b_bOff0 * A_LeftHand ; }
    if (limb == bilateralTeleop::Limbs::RightArm){return X_b_bOff0 * A_RightArm ; }
    if (limb == bilateralTeleop::Limbs::RightForearm){return X_b_bOff0 * A_RightForeArm ; }
    if (limb == bilateralTeleop::Limbs::RightHand){return X_b_bOff0 * A_RightHand ; }
    return sva::MotionVecd::Zero();
  }

 void setPose(const bilateralTeleop::Limbs limb, const sva::PTransformd & p)
  {
    
    if (limb == bilateralTeleop::Limbs::LeftArm){X_0_LeftArm = p; }
    if (limb == bilateralTeleop::Limbs::LeftForearm){X_0_LeftForeArm = p; }
    if (limb == bilateralTeleop::Limbs::LeftHand){X_0_LeftHand = p; }
    if (limb == bilateralTeleop::Limbs::RightArm){X_0_RightArm = p; }
    if (limb == bilateralTeleop::Limbs::RightForearm){X_0_RightForeArm = p; }
    if (limb == bilateralTeleop::Limbs::RightHand){X_0_RightHand = p; }
    if (limb == bilateralTeleop::Limbs::Pelvis){X_0_Base = p; }
   
  }
  void setVel(bilateralTeleop::Limbs limb, const sva::MotionVecd & vel)
  {
    if (limb == bilateralTeleop::Limbs::LeftArm){V_LeftArm = vel; }
    if (limb == bilateralTeleop::Limbs::LeftForearm){V_LeftForeArm = vel; }
    if (limb == bilateralTeleop::Limbs::LeftHand){V_LeftHand = vel; }
    if (limb == bilateralTeleop::Limbs::RightArm){V_RightArm = vel; }
    if (limb == bilateralTeleop::Limbs::RightForearm){V_RightForeArm = vel; }
    if (limb == bilateralTeleop::Limbs::RightHand){V_RightHand = vel; }
    if (limb == bilateralTeleop::Limbs::Pelvis){V_Base = vel; }

  }
  void setAcc(bilateralTeleop::Limbs limb, const sva::MotionVecd & acc)
  {
    if (limb == bilateralTeleop::Limbs::LeftArm){A_LeftArm = acc; }
    if (limb == bilateralTeleop::Limbs::LeftForearm){A_LeftForeArm = acc; }
    if (limb == bilateralTeleop::Limbs::LeftHand){A_LeftHand = acc; }
    if (limb == bilateralTeleop::Limbs::RightArm){A_RightArm = acc; }
    if (limb == bilateralTeleop::Limbs::RightForearm){A_RightForeArm = acc; }
    if (limb == bilateralTeleop::Limbs::RightHand){A_RightHand = acc; }
    if (limb == bilateralTeleop::Limbs::Pelvis){A_Base = acc; }

  }

};

struct RobotPose
{

  void load(const mc_rtc::Configuration & config)
  {
    config("left_hand",LeftHand);
    config("right_hand",RightHand);
    config("left_arm",LeftArm);
    config("right_arm",RightArm);
    config("left_forearm",LeftForeArm);
    config("right_forearm",RightForeArm);
    config("pelvis",Pelvis);
  }

  std::string get(const bilateralTeleop::Limbs part)
  {
    switch(part)
    {
      case bilateralTeleop::Limbs::LeftArm: return LeftArm;
      case bilateralTeleop::Limbs::LeftHand: return LeftHand;
      case bilateralTeleop::Limbs::RightArm: return RightArm;
      case bilateralTeleop::Limbs::RightHand: return RightHand;
      case bilateralTeleop::Limbs::RightForearm: return RightForeArm;
      case bilateralTeleop::Limbs::LeftForearm: return LeftForeArm;
      case bilateralTeleop::Limbs::Pelvis: return Pelvis;

    }
    return "";
  }

  std::string LeftHand = "";
  std::string RightHand = "";
  std::string LeftArm = "";
  std::string RightArm = "";
  std::string LeftForeArm = "";
  std::string RightForeArm = "";
  std::string Pelvis = "";

};

}