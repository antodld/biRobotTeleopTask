#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include "type.h"
#include "boost_serialization_eigen.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <mc_rtc/gui.h>

namespace biRobotTeleop
{
class transformation
{
public:
    transformation()
    {
        transformation(Name::A,Type::Human);
    }
    transformation(const Name & n,const Type & t)
    {
        name_ = n;
        type_ = t;

    }
    void add(Limbs part, const sva::PTransformd & transfo)
    {
        transformations_E_[part] = transfo.rotation();
        transformations_T_[part] = transfo.translation();
        transformations_[part] = sva::PTransformd(transformations_E_[part],transformations_T_[part]);

    }
    const sva::PTransformd & get(const Limbs part) const 
    {
        return transformations_.at(part);
    }

    const Name & name() const noexcept
    {
        return name_;
    }
    void name(Name name)
    {
        name_ = name;
    }
    const Type & type() const noexcept
    {
        return type_;
    }

    size_t size() const noexcept
    {
        return transformations_E_.size();
    }

    // void addToGUI(mc_rtc::gui::StateBuilder & gui,const std::string & name);

private:
    std::map<Limbs, Eigen::Matrix3d> transformations_E_;
    std::map<Limbs, Eigen::Vector3d> transformations_T_;
    std::map<Limbs, sva::PTransformd> transformations_;
    Name name_;
    Type type_;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & transformations_E_;
        ar & transformations_T_;
        ar & name_;
        ar & type_;
    }

};  

}