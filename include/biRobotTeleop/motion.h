#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include "boost_serialization_eigen.h"
#include <eigen3/Eigen/Dense>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <vector>
#include <map>
#include <mc_rtc/gui.h>
#include "type.h"

namespace biRobotTeleop
{
class motion
{
    public:
    motion()
    {
        motion(Name::A,Type::Human);
    }
    motion(const Name & n,const Type & t)
    {
        name_ = n;
        type_ = t;
    }
    void add(Limbs part, const sva::MotionVecd & motion)
    {
        motions_[part] = motion.vector();
    }

    sva::MotionVecd get(Limbs part) const
    {
        return sva::MotionVecd(motions_.at(part));
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

    size_t size() const noexcept
    {
        return motions_.size();
    }

    // void addToGUI(mc_rtc::gui::StateBuilder & gui,const std::string & name);

    private:

    std::map<Limbs, Eigen::Vector6d> motions_;
    
    Name name_;
    Type type_;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & motions_;
        ar & name_;
        ar & type_;
    }

};

}