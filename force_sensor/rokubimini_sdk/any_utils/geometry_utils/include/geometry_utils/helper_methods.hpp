/**
 * @authors     Marco Tranzatto, Gabriel Hottiger
 * @affiliation RSL, ANYbotics
 * @brief       Typedefs for geomerty_utils
 */

#include <cmath>
#include <limits>
#include <type_traits>

namespace geometry_utils
{
// From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
template <class T>
inline typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almostEqual(T x, T y, int ulp = 2)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::abs(x - y) <= std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp
         // unless the result is subnormal
         || std::abs(x - y) < std::numeric_limits<T>::min();
}

}  // namespace geometry_utils
