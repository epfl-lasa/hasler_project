#include "multiAxisFrictionID.h"

int main(int argc, char **argv)
{
  multiAxisFrictionID::Platform_Name platform_id_;
  multiAxisFrictionID::Strategy strategy_;
  float tau_;
  int8_t whichAxis_;

  if (argc>=2)
  {
    if (std::string(argv[1])=="left") {platform_id_=multiAxisFrictionID::LEFT;}
    else if (std::string(argv[1])=="right") {platform_id_=multiAxisFrictionID::RIGHT;}
  }

  else {
    ROS_ERROR("You didn't enter a left|right + (steps|ramp + tau) ");
    return -1;
  }

  if (argc>=3)
  {
    if (std::string(argv[2])=="steps") {strategy_=multiAxisFrictionID::STEPS;}
    else if (std::string(argv[2])=="ramp") {strategy_=multiAxisFrictionID::RAMP;}
  }
  else {
    strategy_ = multiAxisFrictionID::STEPS;
  }

  if (argc >= 4) 
  {
    tau_=atof(argv[3]);
  }
  else
    {
        tau_=TAU_DEFAULT; //! time of completing one step
    }

  if (argc >= 5) {
      tau_ = atoi(argv[4]);
    } else {
      whichAxis_ = -1; //! time of completing one step
    }

    ros::init(argc, argv, std::string(argv[1]) + "FrictionID");

    ros::NodeHandle nh_friction_id;

    float frequency = 90.0f;

    multiAxisFrictionID multiAxisFrictionID(nh_friction_id, frequency,
                                            platform_id_, strategy_, tau_,whichAxis_);

    if (!multiAxisFrictionID.init()) {
      return -1;
  }
  else
  {
    multiAxisFrictionID.run();
  }
  return 0;
}
