#include <rokubimini_cosmo/RokubiminiCosmo.hpp>

#include <message_logger/message_logger.hpp>

#include <string>
#include <utility>

namespace rokubimini_cosmo
{
RokubiminiCosmo::RokubiminiCosmo(any_node::Node::NodeHandlePtr nh) : any_node::Node(std::move(nh))
{
}

bool RokubiminiCosmo::init()
{
  const bool is_standalone = false;
  const bool install_signal_handler = false;
  const double time_step = param<double>("time_step", 0.01);
  const std::string setup_file = param<std::string>("rokubimini_setup_file", "");
  const std::string sensor_config_name = param<std::string>("sensor_config_name", "");

  rokubiminiManager_ = std::make_unique<RokubiminiManager>(is_standalone, install_signal_handler, time_step);

  if (!rokubiminiManager_->loadSetup(setup_file))
  {
    MELO_ERROR_STREAM("Could not load setup file");
  }

  if (!rokubiminiManager_->startup())
  {
    MELO_WARN_STREAM("Manager could not startup");
    return false;
  }

  rokubiminiPublishers_.clear();
  if (sensor_config_name.empty())
  {
    // No name specified. Load all available
    std::vector<Rokubimini*> rokubiminis = rokubiminiManager_->getRokubiminis();
    if (rokubiminis.empty())
    {
      MELO_ERROR_STREAM("No rokubimini available")
      return false;
    }
    for (auto rokubimini : rokubiminis)
    {
      rokubiminiPublishers_.emplace_back(initRokubiminiPublisher(rokubimini));
    }
  }
  else
  {
    Rokubimini* rokubimini = rokubiminiManager_->getRokubimini(sensor_config_name);
    if (rokubimini == nullptr)
    {
      MELO_ERROR_STREAM("No configuration found for Rokubimini with name '" << sensor_config_name << "'")
      return false;
    }
    rokubiminiPublishers_.emplace_back(initRokubiminiPublisher(rokubimini));
  }

  any_worker::WorkerOptions options;
  options.callback_ = std::bind(&RokubiminiCosmo::update, this, std::placeholders::_1);
  options.defaultPriority_ = 10;  // this has high priority
  options.name_ = "RokubiminiCosmo::updateWorker";

  int minimumft_sampling_rate = std::numeric_limits<int>::max();
  /* the following function has to be after setting the filter in order Sampling rate Object to be updated.
   * The filters are set inside the startup function above */
  for (auto& rokubimini_publisher : rokubiminiPublishers_)
  {
    int ft_sampling_rate;
    rokubimini_publisher.rokubimini_->getForceTorqueSamplingRate(ft_sampling_rate);
    minimumft_sampling_rate = std::min(minimumft_sampling_rate, ft_sampling_rate);
  }

  /* never compare with equality floats */
  if (time_step <= 0.000001)
  {
    options.timeStep_ = 1.0 / static_cast<float>(minimumft_sampling_rate);
    MELO_WARN_STREAM("Starting Worker at " << 1 / options.timeStep_ << " Hz, based on force/torque sampling rate.");
  }
  else
  {
    options.timeStep_ = time_step;
    MELO_WARN_STREAM("Starting Worker at " << 1 / options.timeStep_ << " Hz, based on selected time step.");
  }

  if (!this->addWorker(options))
  {
    MELO_ERROR_STREAM("[RokubiminiCosmo] Worker " << options.name_ << "could not be added!");
    return false;
  }
  MELO_DEBUG("Finished initializing cosmo");
  return true;
}

bool RokubiminiCosmo::update(const any_worker::WorkerEvent& event)
{
  MELO_DEBUG("cosmo:update called");
  rokubiminiManager_->updateCommunicationManagerReadMessages();
  rokubiminiManager_->updateProcessReadings();

  for (auto& rokubimini_publisher : rokubiminiPublishers_)
  {
    rokubimini_publisher.rokubimini_->stageCommand(rokubimini_publisher.command_);
  }

  rokubiminiManager_->updateSendStagedCommands();
  rokubiminiManager_->updateCommunicationManagerWriteMessages();

  for (auto& rokubimini_publisher : rokubiminiPublishers_)
  {
    rokubimini_publisher.rokubimini_->getReading(rokubimini_publisher.reading_);
    rokubimini_publisher.readingPublisher_->publish(rokubimini_publisher.reading_);
    rokubimini_publisher.wrenchPublisher_->publish(rokubimini_publisher.reading_.getWrench());
    rokubimini_publisher.imuPublisher_->publish(rokubimini_publisher.reading_.getImu());
  }
  return true;
}

void RokubiminiCosmo::cleanup()
{
  rokubiminiManager_->shutdown();
}

RokubiminiCosmo::RokubiminiPublisherData RokubiminiCosmo::initRokubiminiPublisher(Rokubimini* rokubimini)
{
  RokubiminiPublisherData rokubimini_publisher;
  rokubimini_publisher.rokubimini_ = rokubimini;

  unsigned int serial_number;
  rokubimini->getSerialNumber(serial_number);
  MELO_DEBUG_STREAM("Sensor with config name: '" << rokubimini->getName() << "' has S/N: " << serial_number);

  auto pub_options =
      std::make_shared<cosmo_ros::PublisherRosOptions>(rokubimini->getName() + "/ft_sensor_readings", getNodeHandle());
  pub_options->rosQueueSize_ = 10;
  pub_options->rosLatch_ = false;
  pub_options->autoPublishRos_ = true;
  rokubimini_publisher.readingPublisher_ =
      cosmo_ros::advertiseShmRos<RokubiminiReadingShm, RokubiminiReadingRos, rokubimini_ros::ConversionTraits>(
          rokubimini->getName() + "/ft_sensor_readings", pub_options);

  auto wrench_pub_options = std::make_shared<cosmo_ros::PublisherRosOptions>(
      rokubimini->getName() + "/ft_sensor_readings/wrench", getNodeHandle());
  wrench_pub_options->rosQueueSize_ = 10;
  wrench_pub_options->rosLatch_ = false;
  wrench_pub_options->autoPublishRos_ = true;
  rokubimini_publisher.wrenchPublisher_ =
      cosmo_ros::advertiseShmRos<RokubiminiWrenchShm, RokubiminiWrenchRos, WrenchImuConversionTraits>(
          rokubimini->getName() + "/ft_sensor_wrench", wrench_pub_options);

  auto imu_pub_options = std::make_shared<cosmo_ros::PublisherRosOptions>(
      rokubimini->getName() + "/ft_sensor_readings/imu", getNodeHandle());
  imu_pub_options->rosQueueSize_ = 10;
  imu_pub_options->rosLatch_ = false;
  imu_pub_options->autoPublishRos_ = true;
  rokubimini_publisher.imuPublisher_ =
      cosmo_ros::advertiseShmRos<RokubiminiImuShm, RokubiminiImuRos, WrenchImuConversionTraits>(
          rokubimini->getName() + "/ft_sensor_imu", imu_pub_options);

  return rokubimini_publisher;
}
}  // namespace rokubimini_cosmo
