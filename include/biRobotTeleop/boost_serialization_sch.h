#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/boost_unordered_map.hpp>
#include <boost/serialization/string.hpp>
#include <sch/S_Object/S_Cylinder.h>

namespace boost
{
namespace serialization
{

template<class Archive>
void serialize(Archive & ar, sch::S_Cylinder & cyl, const unsigned int version)
{
    ar & cyl.getP1();
    ar & cyl.getP2();
    ar & cyl.getRadius();
}


} //namespace serialization
} //namespace boost