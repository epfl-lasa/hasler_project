#include "multiAxisFrictionID.h"

int main(int argc, char **argv)
{
  multiAxisFrictionID::Platform_Name platform_id_;
  
  if (argc>=2)
  {
    if (std::string(argv[1])=="left") {platform_id_=multiAxisFrictionID::LEFT;}
    else if (std::string(argv[1])=="right") {platform_id_=multiAxisFrictionID::RIGHT;}
  }
  else
  {
    ROS_ERROR("You didn't enter a left or right + (the number of steps + tau)");
    return -1;
  }

  ros::init(argc, argv, std::string(argv[1])+"FrictionID");


  ros::NodeHandle nh_friction_id;

  float frequency = 90.0f;
  multiAxisFrictionID multiAxisFrictionID(nh_friction_id,frequency,platform_id_);  

    // if (argc >= 3) 
    // { //! To be changed by a dyn reconfig
    //   for (int k = 0; k<NB_AXIS; k++)
    //   {
    //     multiAxisFrictionID._nSteps[k] = (int) *argv[2];
    //   }
 
    // }

    // ROS_INFO(std::string(argv[2]) + " are going to be generated");

    // if (argc >= 4)
    // {
    //   for (int k = 0; k < NB_AXIS; k++)
    //   {
    //     multiAxisFrictionID._tau[k] = (int) *argv[3];
    //   }
    // }

    // ROS_INFO("The time constant to use is " + std::string(argv[3]) + "s");

  if (!multiAxisFrictionID.init()) 
  {
    return -1;
  }
  else
  {
    multiAxisFrictionID.run();
  }
  return 0;
}
