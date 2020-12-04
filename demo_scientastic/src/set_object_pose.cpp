#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"

using namespace std;

int main(int argc,char **argv)
{
    std::string objectName;
    geometry_msgs::Point objectPosition;

    if(argc == 5)
    {
        objectName = std::string(argv[1]);
        objectPosition.x = atof(argv[2]);
        objectPosition.y = atof(argv[3]);
        objectPosition.z = atof(argv[4]);
    }
    else
    {
        return 0;
    }
    ros::init(argc,argv,"set_object_pose");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //orientation
    geometry_msgs::Quaternion objectOrientation;
    objectOrientation.x = 0.0;
    objectOrientation.y = 0.0;
    objectOrientation.z = 0.0;
    objectOrientation.w = 1.0;

    //pose (Pose + Orientation)
    geometry_msgs::Pose modelPose;
    modelPose.position = objectPosition;
    modelPose.orientation = objectOrientation;

    //ModelState
    gazebo_msgs::ModelState modelState;
    modelState.model_name = objectName;
    modelState.pose = modelPose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = modelState;

    if(client.call(srv))
    {
        ROS_INFO("Set object pose");
    }
    else
    {
        ROS_ERROR("Reset box pose! Error msg:%s",srv.response.status_message.c_str());
    }

    return 0;
}