
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/boost_unordered_map.hpp>
#include <boost/serialization/string.hpp>
#include <SpaceVecAlg/SpaceVecAlg>

namespace boost
{
namespace serialization
{

template<class Archive>
void serialize(Archive & ar, sva::PTransformd & p, const unsigned int version)
{
    ar & p.rotation();
    ar & p.translation();
}


} //namespace serialization
} //namespace boost