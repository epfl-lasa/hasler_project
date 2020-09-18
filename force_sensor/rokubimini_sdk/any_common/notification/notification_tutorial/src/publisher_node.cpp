/**
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Christian Gehring
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*!
* @file     publisher_node.cpp
* @author   Christian Gehring
* @date     April 16, 2015
* @brief
*/

// ros
#include <ros/ros.h>

// notification
#include <notification/NotificationPublisher.hpp>

int main(int argc, char** argv)
{
  // Set up ROS node.
  ros::init(argc, argv, "notification_publisher");
  ros::NodeHandle nodeHandle("~");

  // Set up publishers.
  notification::NotificationPublisher screenPublisher("screen", nodeHandle, true);
  notification::NotificationPublisher screenAndAudioPublisher("screen_and_audio", nodeHandle, false);
  notification::NotificationPublisher broadcastPublisher("broadcast", nodeHandle, true);

  // Sleep to make sure the subscribers are set up.
  sleep(1);

  // Broadcast initial notification and wait some time.
  broadcastPublisher.notify(notification::Level::LEVEL_INFO, "Hello all!", "Welcome speech.");
  ros::spinOnce();
  sleep(2);

  // Send a second welcome speech to the screen only.
  screenPublisher.notify(notification::Level::LEVEL_INFO, "Hello again, screen!", "Welcome speech 2 on screen.");
  ros::spinOnce();
  sleep(2);

  // Start sending notifications continuously.
  ros::Rate rate(2);
  unsigned int counter = 0;
  while (ros::ok())
  {
    // Send a vital sign.
    std::stringstream description;
    description << "Vital sign, counter = " << counter;
    screenAndAudioPublisher.notify(notification::Level::LEVEL_INFO, "I'm alive!", description.str());
    counter++;

    if (counter % 10 == 0)
    {
      // Send a wake up notification.
      std::stringstream description2;
      description2 << "You are not allowed to sleep, counter = " << counter / 10;
      screenAndAudioPublisher.notify(notification::Level::LEVEL_WARN, "Wake up!", description2.str());
    }

    // The buffer of this publisher now contains one or two notifications, publish them at once.
    screenAndAudioPublisher.publish();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
