#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/Configuration.h>
#include "type.h"
#include "transformation.h"
#include "motion.h"
#include "boost_serialization_eigen.h"
#include <mc_rtc/gui.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>

namespace biRobotTeleop
{

/**
 * @brief Place holder for the human pose and motion datas.
 * The velocity and acceleration datas should be expressed in a body frame oriented as the world frame
 * The position data are stored as their raw value.
 * 
 * Offsets are included to account for a unified frame system
 * It is set such as when the human is standing straight with its arms alongside its body,
 * the frame are all oriented similarly as the world frame
 * 
 */
struct HumanPose 
{

private:
    transformation pose_; //From the world frame to the link frame (offset is not included in this data)
    motion vel_; //written in the body frame oriented as the world frame
    motion acc_; //written in the body frame oriented as the world frame

    std::string name_ = "";

    //transformation between the body Frame to an offsetted link body frame such as
    //when the arm are alongside the body, all the links frame orientation are matching the world frame,
    //the link frame position should be at the parent joint 
    transformation limbs_offset_; 
    std::map<Limbs, double> convex_radius_;
    std::map<Limbs, double> convex_length_;
    std::map<Limbs, bool> data_online_; //By default at false, can be used to set if a data has not been updated for a long time
    
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

  std::string contact_limbs = "None";

  HumanPose()
  {
    HumanPose("human");
  }
  
  HumanPose(std::string name) 
  {
    name_ = name;
    const auto I = sva::PTransformd::Identity();
    const auto v = sva::MotionVecd::Zero();
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      convex_length_[part] = 1;
      convex_radius_[part] = 1;
      data_online_[part] = false;
      pose_.add(part,I);
      limbs_offset_.add(part,I);
      vel_.add(part,v);
      acc_.add(part,v);
    }

  }

  void addDataToGUI(mc_rtc::gui::StateBuilder & gui);

  /**
   * @brief Only add human position value to gui with offsets
   * 
   * @param gui 
   */
  void addPoseToGUI(mc_rtc::gui::StateBuilder & gui,const bool use_offset = true);

  void addOffsetToGUI(mc_rtc::gui::StateBuilder & gui);

  const bool limbActive(const Limbs limb) const
  {
    return data_online_.at(limb);
  }

  void setLimbActiveState(const Limbs limb, const bool state) noexcept
  {
    data_online_[limb] = state;
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

    offset = config("pelvis")("offset");
    limbs_offset_.add(Pelvis,offset);

  }

  sch::S_Cylinder applyTransformation(const Limbs limb,const sva::PTransformd & X_0_p) const
  {
  
    sva::PTransformd X_p_p1 = sva::PTransformd::Identity();     
    sva::PTransformd X_p_p2 = sva::PTransformd(Eigen::Matrix3d::Identity(),Eigen::Vector3d{0,0,-convex_length_.at(limb)});     

    auto p1 = (X_p_p1 * X_0_p).translation();
    auto p2 = (X_p_p2 * X_0_p).translation();
    sch::Scalar r = convex_radius_.at(limb); 
    sch::S_Cylinder cvx_out = sch::S_Cylinder(sch::Point3(p1.x(),p1.y(),p1.z()),sch::Point3(p2.x(),p2.y(),p2.z()),r);
    
    return cvx_out;
  }

  sch::S_Cylinder getConvex(Limbs limb) const
  {
    return applyTransformation(limb,getOffset(limb) * getPose(limb)); 
  }

  const sva::PTransformd & getPose(Limbs limb) const
  {
    return pose_.get(limb);
  }
  
  const sva::PTransformd & getOffset(const Limbs limb) const
  {
    return limbs_offset_.get(limb);
  }

  const transformation & getOffset() const noexcept
  {
    return limbs_offset_;
  }

  void setOffset(const transformation & limbs_offsets)
  {
    limbs_offset_ = limbs_offsets;
  }

  void setOffset(Limbs limb, const sva::PTransformd & off)
  {
    limbs_offset_.add(limb,off);
  }

  /**
   * @brief Get the velocity at the limb, velocity is expressed at the limb oriented as the world frame
   * 
   * @param limb 
   * @param X_b_bOff 
   * @return sva::MotionVecd 
   */
  sva::MotionVecd getVel(Limbs limb, const sva::PTransformd & X_b_bOff) const
  {
    const Eigen::Matrix3d & R_0_b = (getOffset(limb) * getPose(limb)).rotation();
    sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * vel_.get(limb);
  }

  /**
   * @brief Get the velocity at the limb, velocity is expressed at the limb oriented as the world frame
   * 
   * @param limb 
   * @return sva::MotionVecd  
   */
  const sva::MotionVecd & getVel(Limbs limb) const
  {
    return vel_.get(limb);
  }

  sva::MotionVecd getAcc(Limbs limb,const sva::PTransformd & X_b_bOff) const
  {
    const Eigen::Matrix3d & R_0_b = (getOffset(limb) * getPose(limb)).rotation();
    const sva::PTransformd X_b_bOff0 = sva::PTransformd(Eigen::Matrix3d::Identity(),R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * acc_.get(limb);
  }

  /**
   * @brief Get the accel at the limb, accel is expressed at the limb oriented as the world frame
   * 
   */
  const sva::MotionVecd & getAcc(Limbs limb) const
  {
    return acc_.get(limb);
  }

  /**
   * @brief Update the human pose on one limb, this data should not necesserly be in the unified frame, if so the corresponding offset should be identity
   * 
   * @param limb 
   * @param p 
   */
  void setPose(const Limbs limb, const sva::PTransformd & p)
  {
    pose_.add(limb,p);
  }

  /**
   * @brief Set the limb velocity
   * The velocity is expressed at in the frame of the body oriented as the world frame
   * 
   * @param limb 
   * @param vel 
   */
  void setVel(Limbs limb, const sva::MotionVecd & vel)
  {
    vel_.add(limb,vel);
  }

  /**
   * @brief Set the limb acceleration
   * The velocity is expressed at in the frame of the body oriented as the world frame
   * 
   * @param limb 
   * @param vel 
   */
  void setAcc(Limbs limb, const sva::MotionVecd & acc)
  {
    acc_.add(limb,acc);
  }

  void name(const std::string & n)
  {
    name_ = n;
  }
  const std::string & name() const noexcept
  {
    return name_;
  }

  void updateHumanState(const HumanPose & human)
  {
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
    {
      Limbs limb = static_cast<Limbs>(partInt);
      setPose(limb,human.getPose(limb));
      setVel(limb,human.getVel(limb));
      setAcc(limb,human.getAcc(limb));
      setLimbActiveState(limb,human.limbActive(limb));
    } 
  }

};


/**
 * @brief RobotPose allows to generically maps a limb to a robot body.
 * It also includes offset such as this body frame can be expressed in the unified frame
 * 
 */
struct RobotPose
{
private:

  std::map<Limbs, std::string> links_;
  std::map<std::string,Limbs> limbs_;
  std::map<Limbs, std::string> convex_;
  
  //transformation between the body Frame to an offsetted link body frame such as
  //when the arm are alongside the body, all the links frame orientation are matching the world frame,
  //the link frame position should be at the parent joint 
  transformation links_offsets_;
  std::string robot_name_;

  void setLinksMap(const std::map<Limbs, std::string> & links_map)
  {
    links_ = links_map;
  }
  void setConvexMap(const std::map<Limbs, std::string> & convex)
  {
    convex_ = convex;
  }

  void setOffset(const transformation & links_offsets)
  {
    links_offsets_ = links_offsets;
  }

  void robotName(const std::string & name)
  {
    robot_name_ = name;
  }
    
public:

  RobotPose()
  {
    const auto I = sva::PTransformd::Identity();
    for (int partInt = Limbs::Head ; partInt <= Limbs::RightArm ; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      links_[part] = "";
      convex_[part] = "";
      links_offsets_.add(part,I);

    }
  }

  void load(const mc_rtc::Configuration & config)
  {

    setNameAndConvex(Limbs::LeftHand,config("left_hand"));
    setNameAndConvex(Limbs::RightHand,config("right_hand"));
    setNameAndConvex(Limbs::LeftArm,config("left_arm"));
    setNameAndConvex(Limbs::RightArm,config("right_arm"));
    setNameAndConvex(Limbs::LeftForearm,config("left_forearm"));
    setNameAndConvex(Limbs::RightForearm,config("right_forearm"));
    setNameAndConvex(Limbs::Pelvis,config("pelvis"));
    
    if(config("left_hand").has("offset")){setOffset(Limbs::LeftHand,config("left_hand")("offset"));}
    if(config("right_hand").has("offset")){setOffset(Limbs::RightHand,config("right_hand")("offset"));}
    if(config("left_arm").has("offset")){setOffset(Limbs::LeftArm,config("left_arm")("offset"));}
    if(config("right_arm").has("offset")){setOffset(Limbs::RightArm,config("right_arm")("offset"));}
    if(config("left_forearm").has("offset")){setOffset(Limbs::LeftForearm,config("left_forearm")("offset"));}
    if(config("right_forearm").has("offset")){setOffset(Limbs::RightForearm,config("right_forearm")("offset"));}
  }

  void load(const RobotPose & pose)
  {
    setLinksMap(pose.getLinksMap());
    setConvexMap(pose.getConvexesMap());
    setOffset(pose.getOffset());
    robotName(pose.robotName());
  }

  void setNameAndConvex(const Limbs part,const mc_rtc::Configuration & config)
  {
    std::string link;
    config("name",link);
    setName(part,link); 
    if(config.has("convex"))
    {
      config("convex",convex_[part]); 
    }
    else
    {
      convex_[part] = links_[part];
    }
  }
  void setName(const Limbs part, const std::string & name)
  {
    links_[part] = name;
    limbs_[name] = part;
  }
  void setOffset(const Limbs part, const sva::PTransformd & offset)
  {
    links_offsets_.add(part , offset);
  }

  const std::string & robotName() const noexcept
  {
    return robot_name_;
  }

  const std::string getName(const Limbs part) const
  {
    return links_.at(part);
  }

  const Limbs getLimb(const std::string frame) const
  {
    if(limbs_.find(frame) != limbs_.end())
    {
      return limbs_.at(frame);
    }
    std::cout << "frame not referenced" << std::endl;
    return Limbs::Head;
  }

  const std::string getConvexName(const Limbs part) const
  {
    return convex_.at(part);
  }

  sva::PTransformd getOffset(const Limbs limb) const
  {
    return links_offsets_.get(limb);
  }
  const transformation getOffset() const noexcept
  {
    return links_offsets_;
  }
  const std::map<Limbs, std::string> & getLinksMap() const noexcept
  {
    return links_;
  }
  const std::map<Limbs, std::string> & getConvexesMap() const noexcept
  {
    return convex_;
  }

};

}