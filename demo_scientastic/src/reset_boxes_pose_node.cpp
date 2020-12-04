#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"

using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"reset_boxes_pose");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //osition
    geometry_msgs::Point linkPosition;
    linkPosition.x = -3.0;
    linkPosition.y = -3.0;
    linkPosition.z = 0.5;
    //orientation
    geometry_msgs::Quaternion linkOrientation;
    linkOrientation.x = 0.0;
    linkOrientation.y = 0.0;
    linkOrientation.z = 0.0;
    linkOrientation.w = 1.0;

    //pose (Pose + Orientation)
    geometry_msgs::Pose modelPose;
    modelPose.position = linkPosition;
    modelPose.orientation = linkOrientation;

    //ModelState
    gazebo_msgs::ModelState modelState;
    modelState.model_name = (std::string) "box1";
    modelState.pose = modelPose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = modelState;

    if(client.call(srv))
    {
        ROS_INFO("Reset box pose success!!");
    }
    else
    {
        ROS_ERROR("Reset box pose! Error msg:%s",srv.response.status_message.c_str());
    }

    linkPosition.x = 0.0;
    linkPosition.y = -3.0;
    linkPosition.z = 0.5;
    linkOrientation.x = 0.0;
    linkOrientation.y = 0.0;
    linkOrientation.z = 0.0;
    linkOrientation.w = 1.0;
    modelPose.position = linkPosition;
    modelPose.orientation = linkOrientation;
    modelState.model_name = (std::string) "box2";
    modelState.pose = modelPose;
    srv.request.model_state = modelState;
    if(client.call(srv))
    {
        ROS_INFO("Reset box pose success!!");
    }
    else
    {
        ROS_ERROR("Reset box pose! Error msg:%s",srv.response.status_message.c_str());
    }

    linkPosition.x = 3.0;
    linkPosition.y = -3.0;
    linkPosition.z = 0.5;
    linkOrientation.x = 0.0;
    linkOrientation.y = 0.0;
    linkOrientation.z = 0.0;
    linkOrientation.w = 1.0;
    modelPose.position = linkPosition;
    modelPose.orientation = linkOrientation;
    modelState.model_name = (std::string) "box3";
    modelState.pose = modelPose;
    srv.request.model_state = modelState;
    if(client.call(srv))
    {
        ROS_INFO("Reset box pose success!!");
    }
    else
    {
        ROS_ERROR("Reset box pose! Error msg:%s",srv.response.status_message.c_str());
    }
    return 0;
}