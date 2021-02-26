#include "iostream"
#include <urdf/model.h>
#include "sharedControlGrasp.h"

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "sharedControlGrasp");

  double frequency = 500.0f;

  ros::NodeHandle nh_("~");
  std::string tool_name="";

  nh_.getParam("toolID", tool_name);
  nh_.getParam("publish_frequency", frequency);

  sharedControlGrasp sharedControlGrasp(nh_, frequency,tool_name);
  
  
   if (!sharedControlGrasp.init()) 
   {
     return -1;
   } 
   else 
   {
     sharedControlGrasp.run();
   }
  
  return 0;
}
