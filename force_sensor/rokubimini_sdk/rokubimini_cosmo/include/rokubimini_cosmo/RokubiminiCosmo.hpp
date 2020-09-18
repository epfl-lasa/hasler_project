#pragma once

#include <memory>

#include <any_node/any_node.hpp>
#include <cosmo_ros/cosmo_ros.hpp>

#include <rokubimini/Reading.hpp>
#include <rokubimini/Command.hpp>
#include <rokubimini_manager/Manager.hpp>
#include <rokubimini_msgs/Command.h>
#include <rokubimini_msgs/Reading.h>
#include <rokubimini_ros/ConversionTraits.hpp>

#include <any_measurements/Imu.hpp>
#include <any_measurements/Wrench.hpp>
#include "any_measurements_ros/ConversionTraits.hpp"

#include <any_msgs/Toggle.h>

namespace rokubimini_cosmo
{
using namespace rokubimini;

class RokubiminiCosmo : public any_node::Node
{
public:
  using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;
  using RokubiminiReadingShm = Reading;
  using RokubiminiReadingRos = rokubimini_msgs::Reading;
  using RokubiminiWrenchShm = Reading::WrenchType;
  using RokubiminiWrenchRos = geometry_msgs::WrenchStamped;
  using RokubiminiImuShm = Reading::ImuType;
  using RokubiminiImuRos = sensor_msgs::Imu;
  template <typename Msg_, typename MsgRos_>
  using WrenchImuConversionTraits = any_measurements_ros::ConversionTraits<Msg_, MsgRos_>;

  RokubiminiCosmo() = delete;
  explicit RokubiminiCosmo(NodeHandlePtr nh);
  ~RokubiminiCosmo() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent& event);

private:
  // Rokubimini* rokubimini_;
  std::unique_ptr<RokubiminiManager> rokubiminiManager_;

  using RokubiminiPublisherData = struct RokubiminiPublisherData
  {
    Rokubimini* rokubimini_;
    cosmo_ros::PublisherRosPtr<RokubiminiReadingShm, RokubiminiReadingRos, rokubimini_ros::ConversionTraits>
        readingPublisher_;

    cosmo_ros::PublisherRosPtr<RokubiminiWrenchShm, RokubiminiWrenchRos, WrenchImuConversionTraits> wrenchPublisher_;

    cosmo_ros::PublisherRosPtr<RokubiminiImuShm, RokubiminiImuRos, WrenchImuConversionTraits> imuPublisher_;
    rokubimini::Command command_;
    rokubimini::Reading reading_;
  };
  std::vector<RokubiminiPublisherData> rokubiminiPublishers_;

  RokubiminiPublisherData initRokubiminiPublisher(Rokubimini* rokubimini);
};

}  // namespace rokubimini_cosmo