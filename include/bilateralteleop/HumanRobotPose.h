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

    std::vector<sch::S_Cylinder> convex_;
    std::map<bilateralTeleop::Limbs, Eigen::Vector3d> axis_;


public:

  HumanPose()
  {
    convex_.clear();
    for (int partInt = bilateralTeleop::Limbs::Head ; partInt <= bilateralTeleop::Limbs::RightArm ; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      convex_.push_back(sch::S_Cylinder(sch::Point3(0,0,-1),sch::Point3(0,0,1),0.1));
      axis_[part] = Eigen::Vector3d{0,0,1.};
    }

  }

  void setCvx(const mc_rtc::Configuration & config)
  {
    Eigen::Vector2d length = config("arm")("length");
    double radius = config("arm")("radius");
    Eigen::Vector3d axis = config("arm")("axis")("left");
    axis_[LeftArm] = axis;
    convex_[LeftArm] = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                                       sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);
    axis = config("arm")("axis")("right");
    axis_[RightArm] = axis;
    convex_[RightArm] = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

    length = config("forearm")("length");
    radius = config("forearm")("radius");
    axis = config("forearm")("axis")("left");
    axis_[LeftForearm] = axis;
    convex_[LeftForearm] = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

    axis = config("forearm")("axis")("right");
    axis_[RightForearm] = axis;
    convex_[RightForearm] = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

    length = config("hand")("length");
    radius = config("hand")("radius");
    axis = config("hand")("axis")("left");
    axis_[LeftHand] = axis;
    convex_[LeftHand] = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

    axis = config("hand")("axis")("right");
    axis_[RightHand] = axis;  
    convex_[RightHand] = sch::S_Cylinder(sch::Point3(axis.x(),axis.y(),axis.z()) * - length.x(),
                              sch::Point3(axis.x(),axis.y(),axis.z()) * length.y(),radius);

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
    return applyTransformation(convex_[limb],getPose(limb)); 
  }
  Eigen::Vector3d getAxis(bilateralTeleop::Limbs limb)
  {
    return axis_[limb];
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

  void updateHumanState(HumanPose & human)
  {
    for (int partInt = bilateralTeleop::Limbs::Head ; partInt <= bilateralTeleop::Limbs::RightArm ; partInt++)
    {
      Limbs limb = static_cast<Limbs>(partInt);
      setPose(limb,human.getPose(limb));
      setVel(limb,human.getVel(limb));
      setAcc(limb,human.getAcc(limb));
    } 
  }

};

struct RobotPose
{
private:

    std::map<bilateralTeleop::Limbs, std::string> links_;
    std::map<bilateralTeleop::Limbs, Eigen::Vector3d> axis_;

public:

  RobotPose()
  {
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      links_[part] = "";
      axis_[part] = Eigen::Vector3d{0,0,1};
    }
  }

  void load(const mc_rtc::Configuration & config)
  {
    setName(Limbs::LeftHand,config("left_hand")("name"));
    setName(Limbs::RightHand,config("right_hand")("name"));
    setName(Limbs::LeftArm,config("left_arm")("name"));
    setName(Limbs::RightArm,config("right_arm")("name"));
    setName(Limbs::LeftForearm,config("left_forearm")("name"));
    setName(Limbs::RightForearm,config("right_forearm")("name"));
    setName(Limbs::Pelvis,config("pelvis")("name"));
    
    setAxis(Limbs::LeftHand,config("left_hand")("axis"));
    setAxis(Limbs::RightHand,config("right_hand")("axis"));
    setAxis(Limbs::LeftArm,config("left_arm")("axis"));
    setAxis(Limbs::RightArm,config("right_arm")("axis"));
    setAxis(Limbs::LeftForearm,config("left_forearm")("axis"));
    setAxis(Limbs::RightForearm,config("right_forearm")("axis"));
  }

  void setName(Limbs part,const std::string & name)
  {
    links_[part] = name; 
  }
  void setAxis(Limbs part,const Eigen::Vector3d & axis)
  {
    axis_[part] = axis; 
  }

  std::string getName(const Limbs part)
  {
    return links_[part];
  }

  Eigen::Vector3d getAxis(const Limbs part)
  {
    return axis_[part];
  }

};

}