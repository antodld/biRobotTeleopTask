#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/Configuration.h>
// #include <bilateralTeleop_dataType/type.h>
// #include <bilateralTeleop_dataType/transformation.h>
#include "type.h"
#include "transformation.h"
#include "motion.h"
#include "boost_serialization_eigen.h"
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>

namespace biRobotTeleop
{

struct HumanPose
{

private:
    transformation pose_; //From the world frame to the link frame
    motion vel_; //written in the body frame oriented as the world frame
    motion acc_; //written in the body frame oriented as the world frame

    //offset are made such as : when the arm are alongside the body, all the links frame orientation are matching the world frame,
    //the link frame position should be at the parent joint 
    transformation limbs_offset_; 
    std::map<Limbs, double> convex_radius_;
    std::map<Limbs, double> convex_length_;
    
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & pose_;
        ar & vel_;
        ar & acc_;
        ar & limbs_offset_;
        ar & convex_radius_;
        ar & convex_length_;
    }


public:

  HumanPose()
  {
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      convex_length_[part] = 1;
      convex_radius_[part] = 1;
      limbs_offset_.add(part,sva::PTransformd::Identity());
    }

  }

  void setCvx(const mc_rtc::Configuration & config)
  {
    double length = config("arm")("length");
    double radius = config("arm")("radius");
    convex_length_[LeftArm] = length;
    convex_radius_[LeftArm] = radius;
    convex_length_[RightArm] = length;
    convex_radius_[RightArm] = radius;
    sva::PTransformd offset = config("arm")("offset")("left");
    limbs_offset_.add(LeftArm,offset);


    offset = config("arm")("offset")("right");
    limbs_offset_.add(RightArm,offset);

    length = config("forearm")("length");
    radius = config("forearm")("radius");
    convex_length_[LeftForearm] = length;
    convex_radius_[LeftForearm] = radius;
    convex_length_[RightForearm] = length;
    convex_radius_[RightForearm] = radius;

    offset = config("forearm")("offset")("left");
    limbs_offset_.add(LeftForearm,offset);

    offset = config("forearm")("offset")("right");
    limbs_offset_.add(RightForearm,offset);

    length = config("hand")("length");
    radius = config("hand")("radius");
    convex_length_[LeftHand] = length;
    convex_radius_[LeftHand] = radius;
    convex_length_[RightHand] = length;
    convex_radius_[RightHand] = radius;

    offset = config("hand")("offset")("left");
    limbs_offset_.add(LeftHand,offset);

    offset = config("hand")("offset")("right");
    limbs_offset_.add(RightHand,offset);

  }

  sch::S_Cylinder applyTransformation(const Limbs limb,const sva::PTransformd & X_0_p)
  {
  
    sva::PTransformd X_p_p1 = sva::PTransformd::Identity();     
    sva::PTransformd X_p_p2 = sva::PTransformd(Eigen::Matrix3d::Identity(),Eigen::Vector3d{0,0,-convex_length_[limb]});     

    auto p1 = (X_p_p1 * X_0_p).translation();
    auto p2 = (X_p_p2 * X_0_p).translation();
    sch::Scalar r = convex_radius_[limb]; 
    sch::S_Cylinder cvx_out = sch::S_Cylinder(sch::Point3(p1.x(),p1.y(),p1.z()),sch::Point3(p2.x(),p2.y(),p2.z()),r);
    
    return cvx_out;
  }

  sch::S_Cylinder getConvex(Limbs limb)
  {
    return applyTransformation(limb,getOffset(limb) * getPose(limb)); 
  }

  sva::PTransformd getPose(Limbs limb)
  {
    return pose_.get(limb);
  }
  sva::PTransformd getOffset(const Limbs limb)
  {
    return limbs_offset_.get(limb);
  }

  transformation getOffset()
  {
    return limbs_offset_;
  }
  void setOffset(const transformation & limbs_offsets)
  {
    limbs_offset_ = limbs_offsets;
  }

  sva::MotionVecd getVel(Limbs limb, sva::PTransformd X_b_bOff = sva::PTransformd::Identity())
  {
    Eigen::Matrix3d R_0_b = getPose(limb).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * vel_.get(limb);
  }
  sva::MotionVecd getAcc(Limbs limb, sva::PTransformd X_b_bOff = sva::PTransformd::Identity())
  {
    Eigen::Matrix3d R_0_b = getPose(limb).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * acc_.get(limb);
  }

  void setPose(const Limbs limb, const sva::PTransformd & p)
  {
    pose_.add(limb,p);
  }
  void setVel(Limbs limb, const sva::MotionVecd & vel)
  {
    vel_.add(limb,vel);
  }
  void setAcc(Limbs limb, const sva::MotionVecd & acc)
  {
    acc_.add(limb,acc);
  }

  void updateHumanState(HumanPose & human)
  {
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
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

    std::map<Limbs, std::string> links_;
    //offset are made such as : when the arm are alongside the body, all the links frame orientation are matching the world frame,
    //the link frame position should be at the parent joint 
    transformation links_offsets_;
    std::string robot_name_;
    

public:

  RobotPose()
  {
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      links_[part] = "";
      links_offsets_.add(part ,sva::PTransformd::Identity());
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
    
    if(config("left_hand").has("offset")){setOffset(Limbs::LeftHand,config("left_hand")("offset"));}
    if(config("right_hand").has("offset")){setOffset(Limbs::RightHand,config("right_hand")("offset"));}
    if(config("left_arm").has("offset")){setOffset(Limbs::LeftArm,config("left_arm")("offset"));}
    if(config("right_arm").has("offset")){setOffset(Limbs::RightArm,config("right_arm")("offset"));}
    if(config("left_forearm").has("offset")){setOffset(Limbs::LeftForearm,config("left_forearm")("offset"));}
    if(config("right_forearm").has("offset")){setOffset(Limbs::RightForearm,config("right_forearm")("offset"));}
  }

  void setName(Limbs part,const std::string & name)
  {
    links_[part] = name; 
  }
  void setOffset(Limbs part, const sva::PTransformd & offset)
  {
    links_offsets_.add(part , offset);
  }

  std::string getName(const Limbs part)
  {
    return links_[part];
  }

  sva::PTransformd getOffset(const Limbs limb)
  {
    return links_offsets_.get(limb);
  }
  transformation getOffset()
  {
    return links_offsets_;
  }
  void setOffset(const transformation & links_offsets)
  {
    links_offsets_ = links_offsets;
  }

};

}