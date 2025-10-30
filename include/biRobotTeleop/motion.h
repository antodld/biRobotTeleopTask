#pragma once
#include <mc_rtc/gui.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include "boost_serialization_eigen.h"
#include "type.h"
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

namespace biRobotTeleop
{
class motion
{
public:
  motion()
  {
    motion(Name::A, Type::Human);
  }
  motion(const Name & n, const Type & t)
  {
    name_ = n;
    type_ = t;
  }
  void add(const Limbs part, const sva::MotionVecd & motion)
  {
    motions_[part] = motion;
  }

  const sva::MotionVecd & get(const Limbs part) const
  {
    return motions_.at(part);
  }

  const Name & name() const noexcept
  {
    return name_;
  }

  void name(Name name)
  {
    name_ = name;
  }

  void type(Type t)
  {
    type_ = t;
  }

  const size_t size() const noexcept
  {
    return motions_.size();
  }

  // void addToGUI(mc_rtc::gui::StateBuilder & gui,const std::string & name);

private:
  std::map<Limbs, sva::MotionVecd> motions_;

  std::map<Limbs, Eigen::Vector6d> motionsAsVector_;

  Name name_;
  Type type_;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    for(auto & m : motions_)
    {
      motionsAsVector_[m.first] = m.second.vector();
    }
    ar & motionsAsVector_;
    ar & name_;
    ar & type_;
  }
};

} // namespace biRobotTeleop
