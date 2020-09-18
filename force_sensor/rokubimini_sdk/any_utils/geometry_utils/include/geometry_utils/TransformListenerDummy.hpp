/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Transform listener abstraction.
 */

#pragma once

#include <memory>

#include "geometry_utils/TransformListener.hpp"

namespace geometry_utils
{
class TransformListenerDummy : public TransformListener
{
public:
  //! Default Constructor
  TransformListenerDummy() = default;

  //! Default Destructor
  ~TransformListenerDummy() override = default;

  //! @copydoc TransformListener::canTransform
  bool canTransform(const std::string& /*targetFrame*/, const std::string& /*sourceFrame*/,
                    const Time& /*time*/) const override
  {
    return true;
  };

  //! @copydoc TransformListener::canTransform
  bool getTransformation(TransformStamped* transformStamped, const std::string& targetFrame,
                         const std::string& sourceFrame, const Time& time, double /*timeout*/) const override
  {
    transformStamped->stamp_ = time;
    transformStamped->frame_ = targetFrame;
    transformStamped->childFrame_ = sourceFrame;
    transformStamped->transform_.setIdentity();
    return true;
  }
};

using TransformListenerDummyPtr = std::shared_ptr<TransformListenerDummy>;

}  // namespace geometry_utils
