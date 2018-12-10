#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo/math/gzmath.hh>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
	class footTfUpdate : public ModelPlugin
	{
		
		private: 
				
			//! Variable declaration for actor
			std::string _actorName;
			physics::ModelPtr _actorPtr;
			physics::LinkPtr _actorBaseLink;
			physics::JointPtr _joint;
			

			//! Node Handler, Publishers and Subscribers
			event::ConnectionPtr updateConnection;
			ros::NodeHandle _n;
			tf::TransformListener _actorTfListener;
			tf::StampedTransform _actorTransform;
			//! Variables for current status
			math::Pose _actorPose;
			boost::mutex _lock;

			double _jointStiffness;
			double _jointDamping;

			geometry_msgs::Vector3 _msgForce;
			ros::Publisher _pubForce;

			
		//! Constructor and destroyer
		public: 
		
			footTfUpdate(){}
			~footTfUpdate(){
			ROS_INFO("footTfUpdate finished");	
			this->_n.shutdown();
		}

		//! Gazebo runs this method when the model is spawned
		public: void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_)
		{
			//! Telling to ROS for chaser_vis
			ROS_INFO("Loaded footTfUpdate pluging");	
			// _actorName="chaser";
		    if (!sdf_->HasElement("actorName")) 
			{
			  	ROS_INFO("footTfUpdate missing <actorName> default to %s", _actorName.c_str());
			}
			else 
			{
			  	_actorName = sdf_->GetElement("actorName")->Get<std::string>();
			}
			
			_pubForce = _n.advertise<geometry_msgs::Vector3>(_actorName+"/force", 1);
			std::cerr << _actorName << std::endl;
			
			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "footTfUpdate");
			
			_actorPtr=parent_;
			_actorBaseLink=_actorPtr->GetLink(_actorName+"/base_link");
			//! Connect the update handler with an event
			updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&footTfUpdate::OnUpdate,
										this, _1));
			//! variable initialization
			 //! - 
			_joint = _actorPtr->GetJoint(_actorName+"/sensor_joint");
		    if (!sdf_->HasElement("jointStiffness")) 
			{
				_jointStiffness = 0.0f;
			}
			else 
			{
		  		_jointStiffness = sdf_->GetElement("jointStiffness")->Get<double>();
			}
		    if (!sdf_->HasElement("jointDamping")) 
			{
				_jointDamping = 0.0f;
			}
			else 
			{
		  		_jointDamping = sdf_->GetElement("jointDamping")->Get<double>();
			}
		}


		//! Gazebo periodically runs this callback to update the model status 
		public: void OnUpdate(const common::UpdateInfo &)
		{
			//~ boost::mutex::scoped_lock scoped_lock(_lock); //!  Mutual Exclusion Synchornization Mechanism of the Different Thread
			try{
			  _actorTfListener.lookupTransform("world",_actorName+"/base_link",  
									   ros::Time(0), _actorTransform);
			}
			catch (tf::TransformException ex){
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(1.0).sleep();
			}

			// std::cerr << _actorName << std::endl;
			_actorTransform.getRotation().getW();
			  _actorPose.pos.x=_actorTransform.getOrigin().x();
			  _actorPose.pos.y=_actorTransform.getOrigin().y();
			  _actorPose.pos.z=_actorTransform.getOrigin().z();
			  _actorPose.rot.x=_actorTransform.getRotation().x();
			  _actorPose.rot.y=_actorTransform.getRotation().y();
			  _actorPose.rot.z=_actorTransform.getRotation().z();
			  _actorPose.rot.w=_actorTransform.getRotation().w();
			  		
			math::Vector3 temp(0.1f,0.0f,0.0f);
			math::Vector3 footAttractor(_actorTransform.getOrigin().x(),_actorTransform.getOrigin().y(),
				                                  _actorTransform.getOrigin().z());
			math::Vector3 footPosition = _actorBaseLink->GetWorldCoGPose().pos;
			math::Quaternion footQuaternion = _actorBaseLink->GetWorldCoGPose().rot;
			// std::cerr << footQuaternion.w << " " << footQuaternion.x << " " 
			          // << footQuaternion.y << " " << footQuaternion.z << std::endl;

			// std::cerr << _actorName << " " << footAttractor << " " << footPosition << std::endl;
			math::Vector3 force = 40*(2.0f*(footAttractor-footPosition)-_actorBaseLink->GetWorldLinearVel());
// 			// temp() << 10.0f,0.0f, 0.0f;
// // 
			math::Pose pose;
			pose.pos = _actorBaseLink->GetWorldCoGPose().pos;
			math::Quaternion qd(1.0f,0.0f,0.0f,0.0f);
			math::Quaternion qe = qd*footQuaternion.GetInverse();
			double angle;
			math::Vector3 axis;
			qe.GetAsAxis(axis,angle);
			// _actorBaseLink->SetWorldPose(pose);
			_actorBaseLink->SetForce(force);
			// std::cerr << _actorBaseLink->GetRelativeForce() << std::endl;
			// std::cerr << force << std::endl;
			_actorBaseLink->SetTorque(200*angle*axis-20*_actorBaseLink->GetRelativeAngularVel());

			double jointPosition= _joint->GetAngle(0).Radian();
			double jointForce = _jointStiffness*(0.0f-jointPosition)-_joint->GetVelocity(0)*_jointDamping;
			_joint->SetForce(0,_jointStiffness*(0.0f-jointPosition)-_joint->GetVelocity(0)*_jointDamping);
			// std::cerr << _actorName << " " << jointPosition << " " << jointForce << std::endl;

			_msgForce.x = jointForce;
			_msgForce.y = 0.0f;
			_msgForce.z = 0.0f;
			_pubForce.publish(_msgForce);


			}
		};
	//! Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(footTfUpdate)
}








