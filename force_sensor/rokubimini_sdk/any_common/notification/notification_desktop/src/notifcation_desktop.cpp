/*!
 * @file     notification_desktop.cpp
 * @author   Gabriel Hottiger
 * @date     May 18, 2018
 * @brief
 */

// ros
#include <ros/ros.h>

// notification
#include <notification/Level.hpp>
#include <notification/Notification.hpp>
#include <notification/NotificationSubscriber.hpp>

// libnotify
#include <libnotify/notify.h>

class DesktopNotification
{
public:
  DesktopNotification() : iconPath_{ "/usr/share/icons/gnome/32x32/status/" }
  {
    notify_init("DesktopNotification");
    notificationWidget_.reset(notify_notification_new("", "", 0));
  }

  void notificationCallback(const notification::Notification& notification)
  {
    // Select icon
    std::string iconImage;
    switch (notification.getLevel())
    {
      case notification::Level::LEVEL_ERROR:
      case notification::Level::LEVEL_FATAL:
        iconImage = "messagebox_critical.png";
        break;
      case notification::Level::LEVEL_WARN:
        iconImage = "messagebox_warning.png";
        break;
      case notification::Level::LEVEL_DEBUG:
      case notification::Level::LEVEL_INFO:
      default:
        iconImage = "messagebox_info.png";
        break;
    };

    notify_notification_close(notificationWidget_.get(), 0);
    notify_notification_update(notificationWidget_.get(), notification.name_.c_str(), notification.description_.c_str(),
                               (iconPath_ + iconImage).c_str());
    notify_notification_set_timeout(notificationWidget_.get(), 2000);  // 2 seconds
    notify_notification_show(notificationWidget_.get(), 0);
  }

private:
  const std::string iconPath_;
  std::unique_ptr<NotifyNotification> notificationWidget_;
};

int main(int argc, char** argv)
{
  // Set up ROS node.
  ros::init(argc, argv, "notification_desktop");
  ros::NodeHandle nodeHandle("~");
  std::string deviceName = nodeHandle.param<std::string>("device", "");

  // Add desktop notification subscriber
  DesktopNotification dn;
  auto cb = [&dn](const notification::Notification& notification) { dn.notificationCallback(notification); };
  notification::NotificationSubscriber sub = notification::NotificationSubscriber(deviceName, nodeHandle, cb);

  // Start spinning.
  ros::spin();

  return 0;
}
