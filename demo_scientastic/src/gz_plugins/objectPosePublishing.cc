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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
	class objectPosePublishing : public ModelPlugin
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

			ros::Publisher _pubObjectPose;
			geometry_msgs::Pose _msgObjectPose;
			
			tf::TransformBroadcaster _br;
			tf::Transform _transform;

		//! Constructor and destroyer
		public: 
		
			objectPosePublishing(){}
			~objectPosePublishing(){
			ROS_INFO("objectPosePublishing finished");	
			this->_n.shutdown();
		}

		//! Gazebo runs this method when the model is spawned
		public: void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_)
		{
			//! Telling to ROS for chaser_vis
			ROS_INFO("Loaded objectPosePublishing pluging");	
			// _actorName="chaser";
		    if (!sdf_->HasElement("actorName")) 
			{
			  	ROS_INFO("objectPosePublishing missing <actorName> default to %s", _actorName.c_str());
			}
			else 
			{
			  	_actorName = sdf_->GetElement("actorName")->Get<std::string>();
			}			
			_pubObjectPose = _n.advertise<geometry_msgs::Pose>(_actorName+"/pose", 1);
			
			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "objectPosePublishing");
			
			_actorPtr=parent_;
			_actorBaseLink=_actorPtr->GetLink(_actorName+"/base_link");
			//! Connect the update handler with an event
			updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&objectPosePublishing::OnUpdate,
										this, _1));
		}


		//! Gazebo periodically runs this callback to update the model status 
		public: void OnUpdate(const common::UpdateInfo &)
		{
			//~ boost::mutex::scoped_lock scoped_lock(_lock); //!  Mutual Exclusion Synchornization Mechanism of the Different Thread
			// try{
			//   _actorTfListener.lookupTransform("world",_actorName+"/base_link",  
			// 						   ros::Time(0), _actorTransform);
			// }
			// catch (tf::TransformException ex){
			//   ROS_ERROR("%s",ex.what());
			//   ros::Duration(1.0).sleep();
			// }

			// std::cerr << _actorName << std::endl;
			// _actorTransform.getRotation().getW();
			// _actorPose.pos.x=_actorTransform.getOrigin().x();
			// _actorPose.pos.y=_actorTransform.getOrigin().y();
			// _actorPose.pos.z=_actorTransform.getOrigin().z();
			// _actorPose.rot.x=_actorTransform.getRotation().x();
			// _actorPose.rot.y=_actorTransform.getRotation().y();
			// _actorPose.rot.z=_actorTransform.getRotation().z();
			// _actorPose.rot.w=_actorTransform.getRotation().w();
			  		
			math::Vector3 objectPosition = _actorBaseLink->GetWorldCoGPose().pos;
			math::Quaternion objectOrientation = _actorBaseLink->GetWorldCoGPose().rot;
	
			_msgObjectPose.position.x = objectPosition.x;
			_msgObjectPose.position.y = objectPosition.y;
			_msgObjectPose.position.z = objectPosition.z;
			_msgObjectPose.orientation.w = objectOrientation.w;
			_msgObjectPose.orientation.x = objectOrientation.x;
			_msgObjectPose.orientation.y = objectOrientation.y;
			_msgObjectPose.orientation.z = objectOrientation.z;
			_pubObjectPose.publish(_msgObjectPose);

    		_transform.setOrigin(tf::Vector3(objectPosition.x, objectPosition.y, objectPosition.z));
    		_transform.setRotation(tf::Quaternion(objectOrientation.x, objectOrientation.y, objectOrientation.z, objectOrientation.w));
    		_br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world", _actorName + "/base_link"));
		}
	};
	//! Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(objectPosePublishing)
}








