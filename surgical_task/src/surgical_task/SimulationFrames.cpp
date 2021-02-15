#include "SurgicalTask.h"
#include "gazebo_msgs/SetModelState.h"


void SurgicalTask::checkAllFrames()
{
  bool frameStatus[NB_ROBOTS];
  frameStatus[LEFT] = false;
  frameStatus[RIGHT] = false;

  if(_useSim)
  {
    for(int k = 0; k < NB_ROBOTS; k++)
    {

      bool pillarsStatus = true;
      if(k == 0)
      {
        for(int m = 0; m < _pillarsId.size(); m++)
        {
          pillarsStatus = pillarsStatus && _firstPillarsFrame[m];
        }
      }

      frameStatus[k] = !_useRobot[k] || (_firstRobotBaseFrame[k] && _firstSphericalTrocarFrame[k] && pillarsStatus);

      if(!frameStatus[k])
      {
        std::cerr << k << ": Status: " << "not use: " << !_useRobot[k] 
                  << " robot base: " << _firstRobotBaseFrame[k] 
                  << " trocar : " << _firstSphericalTrocarFrame[k] << " pillars: " << pillarsStatus << std::endl;
      }
    }

  }
  _allFramesOK = !_useSim || (frameStatus[LEFT] && frameStatus[RIGHT]); 
}


void SurgicalTask::receiveFrames()
{

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_useRobot[k])
    {      
      try
      { 
        if(!_firstRobotBaseFrame[k])
        {
          if(k==LEFT)
          {
            if(!_useFranka)
            {
              _lr.waitForTransform("/world", "/left_lwr_base_link", ros::Time(0), ros::Duration(3.0));
              _lr.lookupTransform("/world", "/left_lwr_base_link", ros::Time(0), _transform);                      
            }
            else
            {
              _lr.waitForTransform("/world", "/left_panda_link0", ros::Time(0), ros::Duration(3.0));
              _lr.lookupTransform("/world", "/left_panda_link0", ros::Time(0), _transform);                                    
            }
          }
          else
          {
            if(!_useFranka)
            {
              _lr.waitForTransform("/world", "/right_lwr_base_link", ros::Time(0), ros::Duration(3.0));
              _lr.lookupTransform("/world", "/right_lwr_base_link", ros::Time(0), _transform);                      
            }
            else
            {
              _lr.waitForTransform("/world", "/right_panda_link0", ros::Time(0), ros::Duration(3.0));
              _lr.lookupTransform("/world", "/right_panda_link0", ros::Time(0), _transform);                                    
            }
          }
          _xRobotBaseOrigin[k] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
          _qRobotBaseOrigin[k] << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
          _wRRobotBasis[k] = Utils<float>::quaternionToRotationMatrix(_qRobotBaseOrigin[k]);
          _firstRobotBaseFrame[k] = true;
          std::cerr << "[SurgicalTask]: Robot " << k << " origin received: " << _xRobotBaseOrigin[k].transpose() << std::endl;
        } 
      } 
      catch (tf::TransformException ex)
      {
      }
    }

    if(!_firstSphericalTrocarFrame[k])
    {
      try
      { 

        _lr.waitForTransform("/world", "f" + std::to_string(_sphericalTrocarId[k]), ros::Time(0), ros::Duration(3.0));
        _lr.lookupTransform("/world", "f" + std::to_string(_sphericalTrocarId[k]), ros::Time(0), _transform);        
        _trocarPosition[k] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
        Eigen::Vector4f temp;
        temp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
        _trocarOrientation[k] = -Utils<float>::quaternionToRotationMatrix(temp).col(2);
        _firstSphericalTrocarFrame[k] = true;
        // _trocarsRegistered[k] = true;
        std::cerr << "[SurgicalTask]: Spherical trocar for robot " << k << " origin received: " << _trocarPosition[k].transpose() << std::endl;
        std::cerr << "[SurgicalTask]: Spherical trocar for robot " << k << " orientation received: " << _trocarOrientation[k].transpose() << std::endl;
      } 
      catch (tf::TransformException ex)
      {
      }
    }
  }

  for(int k = 0; k < _pillarsId.size(); k++)
  {
    if(_firstSphericalTrocarFrame[LEFT] && !_firstPillarsFrame[k])
    {    
      try
      { 
        _lr.waitForTransform("/world", "p" + std::to_string(_pillarsId[k]), ros::Time(0), ros::Duration(3.0));
        _lr.lookupTransform("/world", "p" + std::to_string(_pillarsId[k]), ros::Time(0), _transform);        
        _pillarsPosition.row(k) = Eigen::Vector3f(_transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z());
        Eigen::Vector4f q;
        q << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
        // _trocarOrientation[k] = -Utils<float>::quaternionToRotationMatrix(temp).col(2);
        _firstPillarsFrame[k] = true;
        std::cerr << "[SurgicalTask]: Pillars " << k << " origin received: " << _pillarsPosition.row(k).transpose() << std::endl;


        ros::ServiceClient client = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        //position
        geometry_msgs::Point linkPosition;
        linkPosition.x = _pillarsPosition(k,0);
        linkPosition.y = _pillarsPosition(k,1);
        linkPosition.z = _pillarsPosition(k,2);
        //orientation
        geometry_msgs::Quaternion linkOrientation;

        Eigen::Vector4f qe, qd;
        Eigen::Vector3f z, zd;
        z = Utils<float>::quaternionToRotationMatrix(q).col(2);
        zd = (_trocarPosition[LEFT]-_pillarsPosition.row(k).transpose()).normalized();
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(z,zd));
        qd = Utils<float>::quaternionProduct(qe,q);

        // Compute final quaternion on plane
        linkOrientation.x = qd(1);
        linkOrientation.y = qd(2);
        linkOrientation.z = qd(3);
        linkOrientation.w = qd(0);

        //pose (Pose + Orientation)
        geometry_msgs::Pose modelPose;
        modelPose.position = linkPosition;
        modelPose.orientation = linkOrientation;

        //ModelState
        gazebo_msgs::ModelState modelState;
        modelState.model_name = (std::string) "target"+std::to_string(k+1);
        modelState.pose = modelPose;

        gazebo_msgs::SetModelState srv;
        srv.request.model_state = modelState;

        if(client.call(srv))
        {
            ROS_INFO("Set object pose");
        }
        else
        {
            ROS_ERROR("Reset frame pose! Error msg:%s",srv.response.status_message.c_str());
        }

      } 
      catch (tf::TransformException ex)
      {
      }
    }
  }
}
