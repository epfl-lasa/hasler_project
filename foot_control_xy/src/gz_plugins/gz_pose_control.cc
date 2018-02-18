

// Modifications copyright (C) 2017 <tohoku university/jacob hernandez>
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */



#include "chaser_control/gazebo_ros_3D_thruster.h"


namespace gazebo 
{

  GazeboRos3DThruster::GazeboRos3DThruster() {}

  GazeboRos3DThruster::~GazeboRos3DThruster() {}
	  
  // Load the controller
  void GazeboRos3DThruster::Load(physics::ModelPtr parent, 
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;
    base_link_ = parent_->GetChildLink("base_footprint");

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("3DThrusterPlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }


	forcetorque_topic_="thrustersFT";

	if (!sdf->HasElement("thrustersTFtopic")) 
    {
      ROS_WARN("3DThrusterPlugin (ns = %s) missing <thrustersTFTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), forcetorque_topic_.c_str());
    } 
    else 
    {
      forcetorque_topic_ = sdf->GetElement("thrustersTFtopic")->Get<std::string>();
    }

    command_topic_ = "joy";
    
    
    publisher_rate_=30;
    
    
    if (!sdf->HasElement("publisherRate")) 
     {
       ROS_WARN("3DThrusterPlugin (ns = %s) missing <publisherRate>, "
           "defaults to %f",
           robot_namespace_.c_str(), publisher_rate_);
     } 
     else 
     {
       publisher_rate_ = sdf->GetElement("publisherRate")->Get<double>();
     } 
  
    
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("3DThrusterPlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }
    
    /** PID controllers parameters **/
    
		controllers_.co_angular_vel_x_.Load(sdf, "rollpitchvel");
		controllers_.co_angular_vel_y_.Load(sdf, "rollpitchvel");
		controllers_.co_angular_vel_z_.Load(sdf, "yawvel");
		controllers_.co_angular_pos_x_.Load(sdf, "rollpitchpos");
		controllers_.co_angular_pos_y_.Load(sdf, "rollpitchpos");
		controllers_.co_angular_pos_z_.Load(sdf, "yawpos");
		controllers_.co_linear_vel_x_.Load(sdf, "velocityXY");
		controllers_.co_linear_vel_y_.Load(sdf, "velocityXY");
		controllers_.co_linear_vel_z_.Load(sdf, "velocityZ");
		
    /**                                                 **/
    
    //!Initialization of private variables;
    
		command_vel_x_ = 0.0;
		command_vel_y_ = 0.0;
		command_vel_z_ = 0.0;
		linear_velocity_limit_=l_scale_; //! assuming that the axis are normalized before to -1:1
		command_vel_yaw_= 0.0;
		command_vel_roll_= 0.0;
		command_vel_pitch_= 0.0;
		command_pos_yaw_= 0.0;
		command_pos_roll_= 0.0;
		command_pos_pitch_= 0.0;
		relative_force_ = math::Vector3(0.0,0.0,0.0);
		relative_torque_ = math::Vector3(0.0,0.0,0.0);
		linear_cmd_=math::Vector3(0.0,0.0,0.0);
		nominal_thrust_= 10; //! [N/s]
		available_thrust_=math::Vector3(20,20,20);
		dist_base_thruster_COM_=0.150; //![m]
		nominal_reaction_torque_=6; //! angular_impulse_ [N.m.s]
		available_torque_=math::Vector3(9,9,9);
		//!Initialization of axes and buttons of joystick
		linearx_=1; //! XBOX: Left Scroll up/down
		lineary_=0; //! XBOX: Left Scroll left/right
		linearz_=7; //! XBOX: Vertical Arrows
		angularx_=6; //! XBOX: Horizontral Arrows
		angulary_=4; //! XBOX: Right Scroll up/down
		angularz_=3;//! XBOX: Right Scroll left/right
		b_x_stop_=2; //! XBOX: X
		b_y_stop_=3; //! XBOX: Y
		b_z_stop_=1; //! XBOX: B
		bax_stop_o_=5; //! XBOX: Analog 2 (rear button right)
		b_enable_=8; //! XBOX: CENTRAL BUTTON
		b_reset_=7; //! XBOX: RIGHT BUTTON IN THE MIDDLE
		b_angle_co_=5; //! XBOX: RB
		b_orig_rot_=6;//! XBOSX: button back in the middle
		//!Initialization of persistent states for buttons
		
		p_bxstop_=0; p_bystop_=0;p_bzstop_=0;
		p_benable_=0;p_bangleco_=0;p_borigrot_=0;
		p_breset_=0; p_bax_stop_o_=0;
		//!Initialization of scale factors for axes 
		l_scale_=1;
		a_scale_=1;
		
		//! Initialization of flags;
		flag_STOPX_=true;
		flag_STOPY_=true;
		flag_STOPZ_=true;
		flag_STOPORI_=true;
		flag_ENABLED_=false;
		flag_ANGLE_CONTROL_=true;
		flag_ORIGINAL_ROT_=true;
		flag_RESET_=false;
		alive_ = true;
    
   
  // angular_cmd_=math::Vector3(0.0,0.0,0.0);
	
	
	//!Retrieve the joystick axes configuration from the SDF file
	if (sdf->HasElement("joyAxisXLinear"))
      (sdf->GetElement("joyAxisXLinear")->GetValue()->Get(linearx_));  
    if (sdf->HasElement("joyAxisYLinear"))
      (sdf->GetElement("joyAxisYLinear")->GetValue()->Get(lineary_));  
    if (sdf->HasElement("joyAxisZLinear"))
      (sdf->GetElement("joyAxisZLinear")->GetValue()->Get(linearz_));   
  
	if (sdf->HasElement("joyAxisXAngular"))
      (sdf->GetElement("joyAxisXAngular")->GetValue()->Get(angularx_));  
    if (sdf->HasElement("joyAxisYAngular"))
      (sdf->GetElement("joyAxisYAngular")->GetValue()->Get(angulary_));  
    if (sdf->HasElement("joyAxisZAngular"))
      (sdf->GetElement("joyAxisZAngular")->GetValue()->Get(angularz_));  
      
    if (sdf->HasElement("joyButtonStopX"))
      (sdf->GetElement("joyButtonStopX")->GetValue()->Get(b_x_stop_));  
    if (sdf->HasElement("joyButtonStopY"))
      (sdf->GetElement("joyButtonStopY")->GetValue()->Get(b_y_stop_));  
    if (sdf->HasElement("joyButtonStopZ"))
      (sdf->GetElement("joyButtonStopZ")->GetValue()->Get(b_z_stop_)); 
	if (sdf->HasElement("joyButtonEnable"))
      (sdf->GetElement("joyButtonEnable")->GetValue()->Get(b_enable_));  
    if (sdf->HasElement("joyButtonReset"))
      (sdf->GetElement("joyButtonReset")->GetValue()->Get(b_reset_));  
    if (sdf->HasElement("joyButtonAngleControl"))
      (sdf->GetElement("joyButtonAngleControl")->GetValue()->Get(b_angle_co_));
    if (sdf->HasElement("joyButtonWorldRotation"))
      (sdf->GetElement("joyButtonWorldRotation")->GetValue()->Get(b_orig_rot_));    
	//! Retrieve the scale factors for the angular and linear spans
	
	
	 if (sdf->HasElement("joyBAxisStopOri"))
      (sdf->GetElement("joyBAxisStopOri")->GetValue()->Get(bax_stop_o_));    
	
	
	if (sdf->HasElement("joyScaleAxisLinear"))
      (sdf->GetElement("joyScaleAxisLinear")->GetValue()->Get(l_scale_));  
    if (sdf->HasElement("joyScaleAxisAngular"))
      (sdf->GetElement("joyScaleAxisAngular")->GetValue()->Get(a_scale_));  	
	
	//! Retrieve Characteristics of the Navigation System i.e thrusters
	if (sdf->HasElement("velocityXYZLimit"))
      (sdf->GetElement("velocityXYZLimit")->GetValue()->Get(linear_velocity_limit_));  
    if (sdf->HasElement("nominalThrust"))
      (sdf->GetElement("nominalThrust")->GetValue()->Get(nominal_thrust_));  
    if (sdf->HasElement("distThrusterBaseCOM"))
      (sdf->GetElement("distThrusterBaseCOM")->GetValue()->Get(dist_base_thruster_COM_));   
    if (sdf->HasElement("nominalReactionTorque")){
		(sdf->GetElement("nominalReactionTorque")->GetValue()->Get(nominal_reaction_torque_));      
	}
	ROS_INFO("3DThrusterPlugin (ns = %s) nominalThrust = %f", robot_namespace_.c_str(), nominal_thrust_);
	ROS_INFO("3DThrusterPlugin (ns = %s) nominalReactionTorque = %f", robot_namespace_.c_str(), nominal_reaction_torque_);
    ROS_INFO("3DThrusterPlugin (ns = %s) distThrusterBaseCOM_ = %f", robot_namespace_.c_str(), dist_base_thruster_COM_);
	

    // joint_state_idle_sec
    joint_state_idle_sec_ = -1;
    if (sdf->HasElement("joint_state_idle_sec"))
      (sdf->GetElement("joint_state_idle_sec")->GetValue()->Get(joint_state_idle_sec_));
    last_thrust_publish_time_ = parent_->GetWorld()->GetSimTime();
	

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("3DThrusterPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

	available_thrust_=math::Vector3(nominal_thrust_,nominal_thrust_,nominal_thrust_);

    // subscribe to the joystick
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<sensor_msgs::Joy>(command_topic_, 1,
          boost::bind(&GazeboRos3DThruster::JoyCallback, this, _1),
          ros::VoidPtr(), &queue_);
    joy_sub_th_ = rosnode_->subscribe(so);
    thrusters_pub_=rosnode_->advertise<geometry_msgs::Twist>(forcetorque_topic_, 5);
    
    

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRos3DThruster::QueueThread, this));
	this->last_time_ = parent_->GetWorld()->GetSimTime();
    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRos3DThruster::UpdateChild, this));
    
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Update the controller
  void GazeboRos3DThruster::UpdateChild() 
  {
	static int n_persist = 3;
    boost::mutex::scoped_lock scoped_lock(lock); //!  Mutual Exclusion Synchornization Mechanism of the Different Threads
    common::Time current_time = parent_->GetWorld()->GetSimTime();
    math::Pose pose = parent_->GetWorldPose();
    //math::Vector3 principal_moments;
    //math::Vector3 products_inertia;
    //math::Vector3 max_angular_acc;
    //math::Vector3 max_angular_vel;
   
   /*  STOP GESTURE IN GUI
   static int stopx = 0;
   static int stopy = 0;
   static int stopz = 0;
   */
   
   if (publisher_rate_ > 0.0) {
       common::Time current_time = parent_->GetWorld()->GetSimTime();
       double seconds_since_last_update = 
         (current_time - last_thrust_publish_time_).Double();
	       if (seconds_since_last_update > (1.0 / publisher_rate_)) {
		 ForceTorquePublisher();
		 last_thrust_publish_time_ = current_time;
	       }
       }
   
   
   if (p_breset_>=n_persist) { 
	   flag_RESET_=!flag_RESET_;
        flag_ORIGINAL_ROT_=flag_ANGLE_CONTROL_=flag_STOPZ_=flag_STOPY_=flag_STOPX_=false; 
		ROS_WARN("RESET OF ALL STATES");
		p_breset_=0;}
   if (p_benable_>=n_persist) { 
		flag_ENABLED_=!flag_ENABLED_;
		if (flag_ENABLED_) ROS_WARN("JOYSTICK NAVIGATION ENABLED");
		else ROS_WARN("JOYSTICK NAVIGATION DISABLED"); 
		p_benable_=0;} //! The_big_X
   	
   	if (flag_ENABLED_){ 
		
		if (p_bax_stop_o_>=n_persist) { 
			flag_STOPORI_=!flag_STOPORI_; 
			if (flag_STOPORI_) ROS_WARN("AUTOSTOP IN ATTITUDE ENABLED");
			else ROS_WARN("AUTOSTOP IN ATTITUDE DISABLE");
			p_bax_stop_o_=0;} //! button X
		
		if (p_bxstop_>=n_persist) { 
			flag_STOPX_=!flag_STOPX_; 
			if (flag_STOPX_) ROS_WARN("AUTOSTOP IN X ENABLED");
			else ROS_WARN("AUTOSTOP IN X DISABLE");
			p_bxstop_=0;} //! button X
		if (p_bystop_>=n_persist) { 
			flag_STOPY_=!flag_STOPY_; 
			if (flag_STOPY_) ROS_WARN("AUTOSTOP IN Y ENABLED");
			else ROS_WARN("AUTOSTOP IN Y DISABLED");
			p_bystop_=0;} //! button Y
		if (p_bzstop_>=n_persist) {
			flag_STOPZ_=!flag_STOPZ_;
			if (flag_STOPZ_) ROS_WARN("AUTOSTOP IN Z ENABLED");
			else ROS_WARN("AUTOSTOP IN Z DISABLED");
			p_bzstop_=0;} //! button Z
		if (p_bangleco_>=n_persist)  { 
			flag_ANGLE_CONTROL_=!flag_ANGLE_CONTROL_; 
			if (flag_ANGLE_CONTROL_) ROS_WARN("NOW THE ATTITUDE CONTROL IS POSITION");
			else ROS_WARN("THE ATTITUDE CONTROL IS BACK TO VELOCITY");
			p_bangleco_=0; } //!RB
		if ((p_borigrot_>=n_persist))  { 
			if (flag_ANGLE_CONTROL_){
				flag_ORIGINAL_ROT_=!flag_ORIGINAL_ROT_; 
				if (flag_ORIGINAL_ROT_) ROS_WARN("COMMANDING WORLD ORIENTATION");
				else ROS_WARN("COMMANDING RELATIVE ORIENTATION");
			}
			else ROS_WARN("PLEASE ENABLE ATTITUDE CONTROL BY POSITION FIRST");
			p_borigrot_=0;
			 } //!back button in the middles
		
		float dt  = (current_time-this->last_time_).Double();

			//Calculation of the force and torque in the thrusters
		static double force_thrusters = nominal_thrust_*2; //Dual Burn 
		
		
		/**
		static double mass= base_link_->GetInertial()->GetMass();
		principal_moments= base_link_->GetInertial()->GetPrincipalMoments(); //Ixx Iyy Izz
		products_inertia= base_link_->GetInertial()->GetProductsofInertia(); // Ixy Ixz Iyz
		static double max_linear_acc=force_thrusters/mass;
		max_angular_acc= math::Vector3(torque_thrustersandwheels/(principal_moments.x + products_inertia.x + products_inertia.y),
										torque_thrustersandwheels/(principal_moments.y + products_inertia.x + products_inertia.z),
										torque_thrustersandwheels/(principal_moments.z + products_inertia.y + products_inertia.z));   
		static double max_linear_vel=max_linear_acc*dt;
		// ROS_WARN("max_linear_vel = %f", max_linear_vel);
		max_angular_vel = math::Vector3(max_angular_acc.x*dt,
															max_angular_acc.y*dt,
															max_angular_acc.z*dt);
		**/
		// set controllers limits
		/**
			controllers_.co_angular_vel_x_.limit=max_angular_vel.x;
			controllers_.co_angular_vel_y_.limit=max_angular_vel.y;
			controllers_.co_angular_vel_z_.limit=max_angular_vel.z;
			controllers_.co_angular_pos_x_.limit=max_angular_vel.x*dt;
			controllers_.co_angular_pos_y_.limit=max_angular_vel.y*dt;
			controllers_.co_angular_pos_z_.limit=max_angular_vel.z*dt;
			controllers_.co_linear_vel_x_.limit=max_linear_vel*2; //x2x2 thrusters
			controllers_.co_linear_vel_y_.limit=max_linear_vel;
			controllers_.co_linear_vel_z_.limit=max_linear_vel;
		**/
		
		
		//Get Velocities
		
		
		math::Vector3 linear_vel = base_link_->GetRelativeLinearVel();
		math::Vector3 linear_acc = base_link_->GetRelativeLinearAccel();
		//math::Vector3 world_angular_acc = base_link_->GetWorldAngularAccel();
		math::Vector3 relative_angular_acc = base_link_->GetRelativeAngularAccel();
		math::Vector3 world_angular_pos = base_link_->GetWorldPose().rot.GetAsEuler();
		math::Vector3 world_angular_vel = base_link_->GetWorldAngularVel();
		math::Vector3 relative_angular_pos = base_link_->GetRelativePose().rot.GetAsEuler();
		math::Vector3 relative_angular_vel = base_link_->GetRelativeAngularVel();
		
		
		// check for joint_state_idle_sec_
		if ( joint_state_idle_sec_ > 0 &&
			 (current_time-this->last_thrust_publish_time_).Double() > joint_state_idle_sec_ ) {
		  if (int((current_time-this->last_thrust_publish_time_).Double()*1000)%5000 < 1 ) {
			ROS_WARN("3DThrusterPlugin (ns = %s) did not received %s for %f sec",
					 robot_namespace_.c_str(), command_topic_.c_str(), joint_state_idle_sec_);
		  }
		 
		  
		  command_vel_x_ = command_vel_y_= command_vel_z_=0;
		  command_vel_yaw_=command_vel_pitch_=command_vel_roll_=0;
		  command_pos_yaw_=command_pos_pitch_=command_pos_roll_=0;
		}
		
		
		linear_cmd_ = math::Vector3 (command_vel_x_, command_vel_y_, command_vel_z_);
		angular_cmd_ = math::Vector3(command_pos_roll_, command_pos_pitch_,command_pos_yaw_);
		
		if ((command_vel_x_ > 0 || command_vel_x_ < 0)||flag_STOPX_) relative_force_.x = controllers_.co_linear_vel_x_.update(linear_cmd_.x,linear_vel.x,linear_acc.x,dt);
			else relative_force_.x=0; 
		if ((command_vel_y_ > 0 || command_vel_y_ < 0)||flag_STOPY_)	relative_force_.y = controllers_.co_linear_vel_y_.update(linear_cmd_.y,linear_vel.y,linear_acc.y,dt);
			else relative_force_.y=0;
		if ((command_vel_z_ > 0 || command_vel_z_ < 0)||flag_STOPZ_)	relative_force_.z = controllers_.co_linear_vel_z_.update(linear_cmd_.z,linear_vel.z,linear_acc.z,dt);
			else relative_force_.z=0;
		
		/** ALWAYS CONTROL THE ATTITUDE**/
		
		if (flag_ANGLE_CONTROL_)	{
			
			if (flag_ORIGINAL_ROT_){
				command_vel_roll_= controllers_.co_angular_pos_x_.update(angular_cmd_.x,world_angular_pos.x,world_angular_vel.x,dt);
				command_vel_pitch_= controllers_.co_angular_pos_y_.update(angular_cmd_.y,world_angular_pos.y,world_angular_vel.y,dt);
				command_vel_yaw_= controllers_.co_angular_pos_z_.update(angular_cmd_.z,world_angular_pos.z,world_angular_vel.z,dt);
			}
			else {
				command_vel_roll_= controllers_.co_angular_pos_x_.update(angular_cmd_.x,relative_angular_pos.x,relative_angular_vel.x,dt);
				command_vel_pitch_= controllers_.co_angular_pos_y_.update(angular_cmd_.y,relative_angular_pos.y,relative_angular_vel.y,dt);
				command_vel_yaw_= controllers_.co_angular_pos_z_.update(angular_cmd_.z,relative_angular_pos.z,relative_angular_vel.z,dt);
			}
		}
		else{
			command_vel_roll_=angular_cmd_.x; //! if the position control is not enabled, the command from the joystick is interpreted as a velocity control
			command_vel_pitch_=angular_cmd_.y;
			command_vel_yaw_=angular_cmd_.z;
			}
		
		
		//! Attitude control always enabled
		if (command_vel_roll_!=0 || flag_STOPORI_){
			relative_torque_.x = controllers_.co_angular_vel_x_.update(command_vel_roll_,relative_angular_vel.x,relative_angular_acc.x,dt);
		}
		else{
			relative_torque_.x=0;
		}
		if 	(command_vel_pitch_!=0 || flag_STOPORI_){
			relative_torque_.y = controllers_.co_angular_vel_y_.update(command_vel_pitch_,relative_angular_vel.y,relative_angular_acc.y,dt);
		}
		else {
			relative_torque_.y= 0;
		}
		if (command_vel_yaw_ != 0 || flag_STOPORI_) {
			relative_torque_.z = controllers_.co_angular_vel_z_.update(command_vel_yaw_,relative_angular_vel.z,relative_angular_acc.z,dt);
		}
		else{ 
			relative_torque_.z= 0;  
		}
		
		// else relative_torque_.z=0;
		
		/** **/
		
		
		/** TRUNCATE THE OUTPUT OF THE CONTROLLER TO THE FORCE / TORQUE LIMITS **/
		if (fabs(relative_force_.x) > force_thrusters) relative_force_.x = (relative_force_.x<0 ? -1.0 : 1.0) * force_thrusters;
		if (fabs(relative_force_.y) > force_thrusters) relative_force_.y = (relative_force_.y<0 ? -1.0 : 1.0) * force_thrusters;
		if (fabs(relative_force_.z) > force_thrusters) relative_force_.z = (relative_force_.z<0 ? -1.0 : 1.0) * force_thrusters;
		
		
		available_thrust_=math::Vector3(force_thrusters,force_thrusters,force_thrusters).operator-(relative_force_.GetAbs()); //! Calculate available_thrust_ for each direction;
		
		available_torque_=(available_thrust_.operator*(dist_base_thruster_COM_)).operator+(math::Vector3(nominal_reaction_torque_,nominal_reaction_torque_,nominal_reaction_torque_)); //! calculate nominal torque based on available thrust
		
		if (fabs(relative_torque_.x) > (available_torque_.x)) relative_torque_.x = (relative_torque_.x<0 ? -1.0 : 1.0) * available_torque_.x;
		if (fabs(relative_torque_.y) > (available_torque_.y)) relative_torque_.y = (relative_torque_.y<0 ? -1.0 : 1.0) * available_torque_.y;
		if (fabs(relative_torque_.z) > (available_torque_.z)) relative_torque_.z = (relative_torque_.z<0 ? -1.0 : 1.0) * available_torque_.z;
		
		available_torque_=(available_torque_.GetAbs()).operator-(relative_torque_.GetAbs());
		
		//!Aplication of the relative forces and torques


		base_link_->AddRelativeForce(relative_force_);
		base_link_->AddRelativeTorque(relative_torque_);
	}
    /**DEBUGGING MESSAGES **/
    //ROS_WARN("relative_accx= %f",base_link_->GetRelativeLinearAccel().x );
	//ROS_WARN("relative_torquey= %f",relative_torque_.y );
	//ROS_WARN("relative_torquez= %f",relative_torque_.z );
	
	/**   													**/
    this->last_time_ = current_time;
  }



/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  // Finalize the controller
  void GazeboRos3DThruster::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRos3DThruster::JoyCallback(
      const sensor_msgs::Joy::ConstPtr& joy) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    //p_breset_+=joy->buttons[b_reset_];
    //p_benable_+=joy->buttons[b_enable_];
    if (joy->buttons[b_reset_]==1)
		++p_breset_;
	if (joy->buttons[b_enable_]==1)
		++p_benable_;
    if (flag_ENABLED_){
		command_vel_x_ = l_scale_*joy->axes[linearx_];
		command_vel_y_ = l_scale_*joy->axes[lineary_];
		command_pos_yaw_ = a_scale_*joy->axes[angularz_];
		command_vel_z_ = l_scale_*joy->axes[linearz_];
		command_pos_roll_ = a_scale_*joy->axes[angularx_];
		command_pos_pitch_ = a_scale_*joy->axes[angulary_];   
		
		if (joy->axes[bax_stop_o_]==-1)
			++p_bax_stop_o_;
		if (joy->buttons[b_x_stop_]==1)	
			++p_bxstop_;
		if (joy->buttons[b_y_stop_]==1)
			++p_bystop_;
		if (joy->buttons[b_z_stop_]==1)
			++p_bzstop_;
		if (joy->buttons[b_angle_co_]==1)
			++p_bangleco_;
		if (joy->buttons[b_orig_rot_]==1)
			++p_borigrot_;
	}
	last_thrust_publish_time_ = parent_->GetWorld()->GetSimTime();
  }


   void GazeboRos3DThruster::ForceTorquePublisher(){
	   thruster_msg_.linear.x=available_thrust_.x;
	   thruster_msg_.linear.y=available_thrust_.y;
	   thruster_msg_.linear.z=available_thrust_.z;
	   thruster_msg_.angular.x=available_torque_.x;
	   thruster_msg_.angular.y=available_torque_.y;
	   thruster_msg_.angular.z=available_torque_.z;
	   thrusters_pub_.publish(thruster_msg_);
   }


/****
 * 
 * 
 * 
 *                                                                                                                 ****/

  void GazeboRos3DThruster::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

/****
 * 
 * 
 * 
 *                                                                                                                 ****/
	//Reset the controller
	void GazeboRos3DThruster::Reset()
		{
		controllers_.co_linear_vel_x_.reset();
		controllers_.co_linear_vel_y_.reset();
		controllers_.co_linear_vel_z_.reset();
		controllers_.co_angular_vel_x_.reset();
		controllers_.co_angular_vel_y_.reset();
		controllers_.co_angular_vel_z_.reset();
		command_vel_x_ = 0.0;
		command_vel_y_ = 0.0;
		command_vel_z_ = 0.0;
		command_vel_yaw_= 0.0;
		command_vel_roll_= 0.0;
		command_vel_pitch_= 0.0;
		command_pos_yaw_= 0.0;
		command_pos_roll_= 0.0;
		command_pos_pitch_= 0.0;
		relative_force_ = math::Vector3(0.0,0.0,0.0);
		relative_torque_ = math::Vector3(0.0,0.0,0.0); 
		linear_cmd_ = math::Vector3(0.0,0.0,0.0); 
		//angular_cmd_ = math::Vector3(0.0,0.0,0.0);  
		//! Reinitialization of flags;
		flag_STOPX_=true;
		flag_STOPY_=true;
		flag_STOPZ_=true;
		flag_ENABLED_=false;
		flag_ANGLE_CONTROL_=true;
		flag_ORIGINAL_ROT_=true;
		flag_RESET_=false;
		alive_ = true;
		//! Reinitialization of buttons
		p_bxstop_=0; p_bystop_=0;p_bzstop_=0;
		p_benable_=0;p_bangleco_=0;p_borigrot_=0;
		p_breset_=0;p_bax_stop_o_=0;
		
		}
		
  GZ_REGISTER_MODEL_PLUGIN(GazeboRos3DThruster)
}
