#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/Configuration.h>
#include <bilateralTeleop_dataLink/type.h>
#include <bilateralTeleop_dataLink/transformation.h>
#include <bilateralTeleop_dataLink/motion.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>

namespace bilateralTeleop
{

struct HumanPose
{

private:
    transformation pose_;
    motion vel_;
    motion acc_;


    sch::S_Cylinder arm_cvx_ = sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1);
    sch::S_Cylinder forearm_cvx_ = sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1);
    sch::S_Cylinder hand_cvx_ = sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1);

public:

  void setCvx(const mc_rtc::Configuration & config)
  {

    Eigen::Vector2d length = config("arm")("length");
    double radius = config("arm")("radius");
    Eigen::Vector3d axis = config("arm")("axis");
    arm_cvx_ = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

    length = config("forearm")("length");
    radius = config("forearm")("radius");
    axis = config("forearm")("axis");
    forearm_cvx_ = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) *  length.y(),radius);
    length = config("hand")("length");
    radius = config("hand")("radius");
    axis = config("hand")("axis");
    hand_cvx_ = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
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
        return applyTransformation(arm_cvx_,getPose(limb));
      }
      if(limb == bilateralTeleop::Limbs::RightForearm || limb == bilateralTeleop::Limbs::LeftForearm)
      {
        return applyTransformation(forearm_cvx_,getPose(limb));
      }

      else
      {
        return applyTransformation(hand_cvx_,getPose(limb));
      }
    
  }


  sva::PTransformd getPose(bilateralTeleop::Limbs limb)
  {
    return pose_.get(limb);
  }
  sva::MotionVecd getVel(bilateralTeleop::Limbs limb, sva::PTransformd X_b_bOff = sva::PTransformd::Identity())
  {
    Eigen::Matrix3d R_0_b = getPose(limb).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * vel_.get(limb);
  }
  sva::MotionVecd getAcc(bilateralTeleop::Limbs limb, sva::PTransformd X_b_bOff = sva::PTransformd::Identity())
  {
    Eigen::Matrix3d R_0_b = getPose(limb).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * acc_.get(limb);
  }

  void setPose(const bilateralTeleop::Limbs limb, const sva::PTransformd & p)
  {
    pose_.add(limb,p);
  }
  void setVel(bilateralTeleop::Limbs limb, const sva::MotionVecd & vel)
  {
    vel_.add(limb,vel);
  }
  void setAcc(bilateralTeleop::Limbs limb, const sva::MotionVecd & acc)
  {
    acc_.add(limb,acc);
  }

};

struct RobotPose
{
private:

    std::map<bilateralTeleop::Limbs, std::string> links_;

public:

  RobotPose()
  {
    for (int partInt = Limbs::Head ; partInt != Limbs::RightArm ; partInt++) {
      Limbs part = static_cast<Limbs>(partInt);
      links_[part] = "";
    }
  }

  void load(const mc_rtc::Configuration & config)
  {
    set(Limbs::LeftHand,config("left_hand"));
    set(Limbs::RightHand,config("right_hand"));
    set(Limbs::LeftArm,config("left_arm"));
    set(Limbs::RightArm,config("right_arm"));
    set(Limbs::LeftForearm,config("left_forearm"));
    set(Limbs::RightForearm,config("right_forearm"));
    set(Limbs::Pelvis,config("pelvis"));
  }

  void set(Limbs part,const std::string & name)
  {
    links_[part] = name; 
  }

  std::string get(const Limbs part)
  {
    return links_[part];
  }

};

}