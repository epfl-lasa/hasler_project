#include "iostream"
#include <urdf/model.h>
#include "endoscopeModifier.h"

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "endoscope_modifier");

  float frequency = 100.0f;

  ros::NodeHandle nh_("~");
  std::string tool_name;

  nh_.getParam("publish_frequency", frequency);

  endoscopeModifier endoscopeFB(nh_,frequency);
  
  
   if (!endoscopeFB.init()) 
   {
     return -1;
   } 
   else 
   {
     endoscopeFB.run();
   }
  
  return 0;
}
