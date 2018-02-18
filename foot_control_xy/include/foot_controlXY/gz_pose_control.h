
#ifndef GAZEBO_POSE_CONTROLLER_HH
#define GAZEBO_POSE_CONTROLLER_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "pid_controller.h"

namespace gazebo {

  class GazeboRos3DThruster : public ModelPlugin {

    public: 
      GazeboRos3DThruster();
      ~GazeboRos3DThruster();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();
      virtual void Reset();

    private:
      void ForceTorquePublisher();
      physics::ModelPtr parent_;
      physics::LinkPtr base_link_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_; //! nodehandle from gazebo
      ros::Subscriber joy_sub_th_;
      //ros::Subscriber vel_sub_;
      ros::Publisher thrusters_pub_;
      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string forcetorque_topic_;
      
      double force_torque_rate_;
      
      // Custom Callback Subscriber Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();
      
      //Message for force/torque in the ACS system
      	geometry_msgs::Twist thruster_msg_;
      
      // joystick message Callback

     // void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);	
      void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
      double l_scale_,a_scale_;
      int linearx_,lineary_,linearz_,angularx_,angulary_,angularz_;
      int bax_stop_o_;
      int b_x_stop_,b_y_stop_,b_z_stop_;
      int b_reset_;
      int p_breset_;
      int p_bxstop_,p_bystop_,p_bzstop_;
      int b_enable_,b_angle_co_,b_orig_rot_;
      int p_benable_,p_bangleco_;
      int p_borigrot_; int p_bax_stop_o_;
	
      double command_vel_x_;
      double command_vel_y_;
      double command_vel_z_;
      double linear_velocity_limit_;
      double command_pos_yaw_;
      double command_pos_roll_;
      double command_pos_pitch_;
      double command_vel_yaw_;
      double command_vel_roll_;
      double command_vel_pitch_;
      
      bool flag_STOPX_;
      bool flag_STOPY_;
      bool flag_STOPZ_;
      bool flag_STOPORI_;
      bool flag_ENABLED_; 
      bool flag_RESET_;
      bool flag_ANGLE_CONTROL_;
      bool flag_ORIGINAL_ROT_;
      
      bool alive_;
      double nominal_thrust_; //! [N]
      double nominal_reaction_torque_; //! [N]
      double dist_base_thruster_COM_; //! in [m]
      double joint_state_idle_sec_;
      double publisher_rate_;
	
      common::Time last_time_; //! for calculating dt
      common::Time last_thrust_publish_time_;
      math::Vector3 relative_force_;
      math::Vector3 relative_torque_;
      math::Vector3 linear_cmd_;
      math::Vector3 angular_cmd_; //ATTITUDE CONTROLLER ALWAYS ACTIVE!
      math::Vector3 available_thrust_;
      math::Vector3 available_torque_;
      struct Controller{
	PIDController co_linear_vel_x_;
	PIDController co_linear_vel_y_;
	PIDController co_linear_vel_z_;
	PIDController co_angular_vel_x_; // roll
	PIDController co_angular_vel_y_; //pítch
	PIDController co_angular_vel_z_; //yaw  
	PIDController co_angular_pos_x_; // roll
	PIDController co_angular_pos_y_; //pítch
	PIDController co_angular_pos_z_; //yaw    
      } controllers_;
  };
  
  
  

}

#endif /* end of include guard: GAZEBO_ROS_3D_THRUSTER_HH */
