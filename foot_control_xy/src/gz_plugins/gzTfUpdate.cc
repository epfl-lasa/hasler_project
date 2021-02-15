#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#if GAZEBO_MAJOR_VERSION >= 9
	#include <ignition/math.hh>
#else 
	#include <gazebo/math/gzmath.hh>
#endif
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>

namespace gazebo
{
	class gzTfUpdate : public ModelPlugin
	{
		
		private: 
				
			//! Variable declaration for actor
			std::string _actorName;
			physics::ModelPtr _actorPtr;
			physics::LinkPtr _actorBaseLink;
			

			//! Node Handler, Publishers and Subscribers
			event::ConnectionPtr updateConnection;
			ros::NodeHandle _n;
			tf::TransformListener _actorTfListener;
			tf::StampedTransform _actorTransform;
			//! Variables for current status

#if GAZEBO_MAJOR_VERSION >= 9
			ignition::math::Pose3d _actorPose;
#else 
			math::Pose _actorPose;
#endif
			boost::mutex _lock;
			
		//! Constructor and destroyer
		public: 
		
			gzTfUpdate(){}
			~gzTfUpdate(){
			ROS_INFO("gzTfUpdate finished");	
			this->_n.shutdown();
		}

		//! Gazebo runs this method when the model is spawned
		public: void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_)
		{
			//! Telling to ROS for chaser_vis
			ROS_INFO("Loaded gzTfUpdate pluging");	
			_actorName="chaser";
			    if (!sdf_->HasElement("actorName")) 
					{
					  ROS_INFO("gzTfUpdate missing <actorName> default to %s", _actorName.c_str());
					}
					else 
					{
					  _actorName = 
						sdf_->GetElement("actorName")->Get<std::string>();
					}
			
			
			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "gzTfUpdate");
			
			_actorPtr=parent_;
			_actorBaseLink=_actorPtr->GetLink(_actorName+"/base_link");
			//! Connect the update handler with an event
			updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&gzTfUpdate::OnUpdate,
										this, _1));
			//! variable initialization
			 //! - 

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
				_actorTransform.getRotation().getW();

#if GAZEBO_MAJOR_VERSION >= 9
		  	_actorPose.Pos().X() =_actorTransform.getOrigin().x();
			_actorPose.Pos().Y() =_actorTransform.getOrigin().y();
			_actorPose.Pos().Z() =_actorTransform.getOrigin().z();
			_actorPose.Rot().X() =_actorTransform.getRotation().x();
			_actorPose.Rot().Y() =_actorTransform.getRotation().y();
			_actorPose.Rot().Z() =_actorTransform.getRotation().z();
			_actorPose.Rot().W() =_actorTransform.getRotation().w();
#else 
		  	_actorPose.pos.x=_actorTransform.getOrigin().x();
			_actorPose.pos.y=_actorTransform.getOrigin().y();
			_actorPose.pos.z=_actorTransform.getOrigin().z();
			_actorPose.rot.x=_actorTransform.getRotation().x();
			_actorPose.rot.y=_actorTransform.getRotation().y();
			_actorPose.rot.z=_actorTransform.getRotation().z();
			_actorPose.rot.w=_actorTransform.getRotation().w();
#endif

			_actorBaseLink->SetWorldPose(_actorPose);
		}
	};
	//! Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(gzTfUpdate)
}








