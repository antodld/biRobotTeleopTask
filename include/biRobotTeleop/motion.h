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
        for (int partInt = Limbs::Head ; partInt != Limbs::RightArm ; partInt++) {
            Limbs part = static_cast<Limbs>(partInt);
            motions_[part] = Eigen::Vector6d::Zero();
        }
    }
    void add(Limbs part, const sva::MotionVecd & motion)
    {
        motions_[part] = motion.vector();
    }

    sva::MotionVecd get(Limbs part)
    {
        return sva::MotionVecd(motions_[part]);
    }

    Name name()
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