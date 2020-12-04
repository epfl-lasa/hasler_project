#include "multiAxisFrictionID.h"

int main(int argc, char **argv)
{
  multiAxisFrictionID::Platform_Name platform_id_;
  multiAxisFrictionID::Strategy strategy_;
  float tau_;
  int whichAxis_;

  if (argc>=2)
  {
    if (std::string(argv[1])=="left") {platform_id_=multiAxisFrictionID::LEFT;}
    else if (std::string(argv[1])=="right") {platform_id_=multiAxisFrictionID::RIGHT;}
  }

  else {
    ROS_ERROR("You didn't enter a left|right + (steps|ramp + tau) + axis ");
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
    tau_= tau_>=0.02f ? tau_ : 0.02f;
  }
  else
  {
    tau_=TAU_DEFAULT; //! time of completing one step
  }

  if (argc >= 5)
  {
    
      if (std::string(argv[4]) == "x")
      {
         whichAxis_= X;
      }
      else if (std::string(argv[4]) == "y")
      {
        whichAxis_ = Y;
      }
      else if (std::string(argv[4]) == "pitch")
      {
        whichAxis_ = PITCH;
      }
      else if (std::string(argv[4]) == "roll")
      {
        whichAxis_ = ROLL;
      }
      else if (std::string(argv[4]) == "yaw")
      {
        whichAxis_ = YAW;
      }
      else if (std::string(argv[4]) == "all")
      {
        whichAxis_ = -1;
      }
    }
    else
    {
      whichAxis_ = X;
    }

    ros::init(argc, argv, std::string(argv[1]) + "FrictionID");

    ros::NodeHandle nh_friction_id;

    float frequency = 100.0f;

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
