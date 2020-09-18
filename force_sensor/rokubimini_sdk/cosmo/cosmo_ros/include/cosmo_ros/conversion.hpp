#pragma once

namespace cosmo_ros
{
template <typename Msg_, typename MsgRos_>
class ConversionTraits;

// Example for ConversionTraits:
// template<typename Msg_, typename MsgRos_>
// class ConversionTraits {
// public:
// inline static MsgRos_ convert(const Msg_& msg);
// inline static Msg_ convert(const MsgRos_& msg);
//};

// Tests for a valid implementation of the convert function
template <typename Msg_, typename MsgRos_, template <typename, typename> class Converter_>
class HasConvert
{
private:
  template <typename T, T>
  struct helper;
  template <typename T>
  static std::uint8_t check(helper<MsgRos_ (*)(const Msg_&), &T::convert>*);
  template <typename T>
  static std::uint16_t check(...);

public:
  static constexpr bool VALUE = sizeof(check<Converter_<Msg_, MsgRos_>>(nullptr)) == sizeof(std::uint8_t);
};
}  // namespace cosmo_ros
