#pragma once

// std
#include <cstdint>

namespace rokubimini
{
namespace ethercat
{
namespace sdo
{
// Not sure if needed
struct RxSdoAccelerationRange
{
  uint8_t accelerationRange_{ 1 };
} __attribute__((packed));
}
}  // namespace ethercat
}  // namespace rokubimini
