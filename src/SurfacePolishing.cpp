#include "SurfacePolishing.h"
#include "Utils.h"

SurfacePolishing* SurfacePolishing::me = NULL;

SurfacePolishing::SurfacePolishing(ros::NodeHandle &n, double frequency, std::string fileName, SurfaceType surfaceType, 
                                   OriginalDynamics originalDynamics, ModulationType modulationType,
                                  Formulation formulation, Constraint constraint, float targetVelocity, float targetForce):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName),
  _surfaceType(surfaceType),
  _originalDynamics(originalDynamics),
  _modulationType(modulationType),
  _formulation(formulation),
  _constraint(constraint),
  _targetVelocity(targetVelocity),
  _targetForce(targetForce)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.035f;
  _toolOffset = 0.15f;
  _loadMass = 0.0f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);

  _xd.setConstant(0.0f);
  _vdOrig.setConstant(0.0f);
  _vdR.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _Fd = 0.0f;
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  // _taskAttractor << -0.6f, -0.2f, 0.186f;
  // _contactAttractor << -0.6f, 0.2f, 0.186f;
  // _taskAttractor << -0.55f, 0.0f, -0.007f;
  // _contactAttractor << -0.55f, 0.0f, -0.007f;
  _p << 0.0f,0.0f,-0.007f;

  _taskAttractor << -0.65f, 0.05f, -0.007f;
  _contactAttractor << -0.65f, 0.05f, -0.007f;
  // _p << 0.0f,0.0f,-0.007f;
  
  _planeNormal << 0.0f, 0.0f, 1.0f;

  _Fc.setConstant(0.0f);
  _Tc.setConstant(0.0f);

  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;

  _wrenchBiasOK = false;
  _stop = false;

  _sequenceID = 0;
  _normalDistance = 0.0f;
  _normalForce = 0.0f;

  _firstOptitrackRobotPose = false;
  _firstOptitrackP1Pose = false;
  _firstOptitrackP2Pose = false;
  _firstOptitrackP3Pose = false;
  _optitrackOK = false;
  _ensurePassivity = true;

  _averageCount = 0;

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _lambda1 = 0.0f;
  _integrator.setConstant(0.0f);

  _firstDampingMatrix = false;
  _D.setConstant(0.0f);
  _smax = 0.2f;
  _s = 0.0f;
  _dW = 0.0f;

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.ns = "marker_test_triangle_list";
  _msgMarker.id = 0;
  _msgMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  _msgMarker.action = visualization_msgs::Marker::ADD;
  _msgMarker.pose.position.x = _p(0);
  _msgMarker.pose.position.y = _p(1);
  _msgMarker.pose.position.z = _p(2);
  _msgMarker.pose.orientation.x = 0.0;
  _msgMarker.pose.orientation.y = 1.0;
  _msgMarker.pose.orientation.z = 0.0;
  _msgMarker.pose.orientation.w = 0.0;
  _msgMarker.scale.x = 1.0;
  _msgMarker.scale.y = 1.0;
  _msgMarker.scale.z = 1.0;
  _msgMarker.color.a = 1.0;

  geometry_msgs::Point p1,p2,p3,p4,p5,p6;
  float objectWidth = 0.59f;
  float objectLength = 0.82f;
  p1.x = objectWidth/2.0f;
  p1.y = -objectLength/2.0f;
  p1.z = 0.0f;
  p2.x = -objectWidth/2.0f;
  p2.y = -objectLength/2.0f;
  p2.z = 0.0f;
  p3.x = -objectWidth/2.0f;
  p3.y = objectLength/2.0f;
  p3.z = 0.0f;
  p4.x = -objectWidth/2.0f;
  p4.y = objectLength/2.0f;
  p4.z = 0.0f;
  p5.x = objectWidth/2.0f;
  p5.y = objectLength/2.0f;
  p5.z = 0.0f;
  p6.x = objectWidth/2.0f;
  p6.y = -objectLength/2.0f;
  p6.z = 0.0f;

  Eigen::Vector3f t1,t2;
  t1 << 1.0f,0.0f,0.0f;
  t2 << 0.0f,1.0f,0.0f;
  _p1 = _p-0.3f*t1+(objectLength/2.0f)*t2;
  _p2 = _p1-objectLength*t2;
  _p3 = _p1-objectWidth*t1;

  std_msgs::ColorRGBA c;
  c.r = 0.7;
  c.g = 0.7;
  c.b = 0.7;
  c.a = 1.0;

  _msgArrowMarker.header.frame_id = "world";
  _msgArrowMarker.header.stamp = ros::Time();
  _msgArrowMarker.ns = "marker_test_arrow";
  _msgArrowMarker.id = 1;
  _msgArrowMarker.type = visualization_msgs::Marker::ARROW;
  _msgArrowMarker.action = visualization_msgs::Marker::ADD;
  p1.x = 0.0f;
  p1.y = 0.0f;
  p1.z = 0.0f;
  p2.x = 0.0f+0.3f*_e1(0);
  p2.x = 0.0f+0.3f*_e1(1);
  p2.x = 0.0f+0.3f*_e1(2);
  _msgArrowMarker.scale.x = 0.05;
  _msgArrowMarker.scale.y = 0.1;
  _msgArrowMarker.scale.z = 0.1;
  _msgArrowMarker.color.a = 1.0;
  _msgArrowMarker.color.r = 1.0;
  _msgArrowMarker.color.g = 0.0;
  _msgArrowMarker.color.b = 0.0;
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);


  for(int k = 0; k < 6; k++)
  {
    _msgMarker.colors.push_back(c);
  }

  _msgMarker.points.push_back(p1);
  _msgMarker.points.push_back(p2);
  _msgMarker.points.push_back(p3);
  _msgMarker.points.push_back(p4);
  _msgMarker.points.push_back(p5);
  _msgMarker.points.push_back(p6);

}


bool SurfacePolishing::init() 
{
  // Subscriber definitions
  _subRobotPose = _n.subscribe("/lwr/ee_pose", 1, &SurfacePolishing::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _n.subscribe("/lwr/joint_controllers/twist", 1, &SurfacePolishing::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &SurfacePolishing::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackRobotBasisPose = _n.subscribe("/optitrack/robot/pose", 1, &SurfacePolishing::updateOptitrackRobotPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane1Pose = _n.subscribe("/optitrack/plane1/pose", 1, &SurfacePolishing::updateOptitrackP1Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane2Pose = _n.subscribe("/optitrack/plane2/pose", 1, &SurfacePolishing::updateOptitrackP2Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane3Pose = _n.subscribe("/optitrack/plane3/pose", 1, &SurfacePolishing::updateOptitrackP3Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix = _n.subscribe("/lwr/joint_controllers/passive_ds_damping_matrix", 1, &SurfacePolishing::updateDampingMatrix,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("SurfacePolishing/filteredWrench", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("SurfacePolishing/plane", 1);
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("SurfacePolishing/taskAttractor", 1);  
  _pubNormalForce = _n.advertise<std_msgs::Float32>("SurfacePolishing/normalForce", 1);
  _pubOrientationIntegrator = _n.advertise<std_msgs::Float32>("/lwr/joint_controllers/passive_ds_command_orient_integrator", 1);


  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&SurfacePolishing::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,SurfacePolishing::stopNode);


  ROS_INFO("Filename: %s", _fileName.c_str());

  _outputFile.open(ros::package::getPath(std::string("force_based_ds_modulation"))+"/data_modulation/"+_fileName+".txt");


  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_lambda1))
  {
    ROS_ERROR("Cannot read first eigen value of passive ds controller");
    return false;
  }


  if(!_outputFile.is_open())
  {
    ROS_ERROR("Cannot open data file");
    return false;
  }

  if(_surfaceType == PLANE)
  {
    ROS_INFO("Surface type: PLANE");
    _firstOptitrackRobotPose = true;
    _firstOptitrackP1Pose = true;
    _firstOptitrackP2Pose = true;
    _firstOptitrackP3Pose = true;  
    _optitrackOK = true;
  }
  else if(_surfaceType == PLANE_OPTITRACK)
  {
    ROS_INFO("Surface type: PLANE_OPTITRACK");
  }
  else if(_surfaceType == LEARNED_SURFACE)
  {
    ROS_INFO("Surface type: LEANRED_SURFACE");

     std::string modelPath = ros::package::getPath(std::string("force_based_ds_modulation"))+"/data_surface/learned_surface_svmgrad_model.txt";
     _inputFile.open(modelPath);
     if(!_inputFile.is_open())
     {
        ROS_ERROR("Cannot open model file");
        return false;
     }
     else
     {
        _inputFile.close();
        _svm.loadModel(modelPath);
     }
  }
  else
  {
    ROS_ERROR("Surface type not recognized");
    return false;
  }

  if(_originalDynamics == CONSTANT)
  {
    ROS_INFO("Original dynamics: CONSTANT");
  }
  else if(_originalDynamics == ARBITRARY)
  {
    ROS_INFO("Original dynamics: ARBITRARY");
  }
  else
  {
    ROS_ERROR("Original dynamics not recognized");
    return false;
  }

  if(_modulationType == ROTATION)
  {
    ROS_INFO("Modulation type: ROTATION");
  }
  else if(_modulationType == ROTATION_AND_FORCE)
  {
    ROS_INFO("Modulation type: ROTATION_AND_FORCE");
  }
  else
  {
    ROS_ERROR("Modulation type not recognized");
    return false;
  }

  if(_formulation == F1)
  {
    ROS_INFO("Formulation: F1");
  }
  else if(_formulation == F2)
  {
    ROS_INFO("Formulation: F2");
  }
  else if(_formulation == F3)
  {
    ROS_INFO("Formulation: F3");
  }
  else
  {
    ROS_ERROR("Formulation not recognized");
    return false;
  }

  if(_constraint == VELOCITY_NORM)
  {
    ROS_INFO("Constraint: VELOCITY_NORM");
  }
  else if(_constraint == APPARENT_VELOCITY_NORM)
  {
    ROS_INFO("Constraint: APPARENT_VELOCITY_NORM");
  }
  else
  {
    ROS_ERROR("Constraint not recognized");
    return false;
  }

  if(_targetVelocity>0.0f)
  {
    ROS_INFO("Target velocity: %f", _targetVelocity);
  }
  else
  {
    ROS_ERROR("Target velocity should be positive");
  }

  if(_targetForce>0.0f)
  {
    ROS_INFO("Target force: %f", _targetForce);
  }
  else
  {
    ROS_ERROR("Target force should be positive");
  }


  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void SurfacePolishing::run()
{

  _timeInit = ros::Time::now().toSec();

  std_msgs::Float32 msg;
  msg.data = 0.0f;
  _pubOrientationIntegrator.publish(msg);
  ros::spinOnce();
  _loopRate.sleep();

  while (!_stop && ros::Time::now().toSec()-_timeInit < _duration) 
  {


    if(_firstRobotPose && _firstRobotTwist && _wrenchBiasOK &&
       _firstOptitrackRobotPose && _firstOptitrackP1Pose &&
       _firstOptitrackP2Pose && _firstOptitrackP3Pose)
    {
      _mutex.lock();

      // Check for update of passive ds controller eigen value
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_lambda1);

      if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {

        // Compute control command
        computeCommand();

        // Publish data to topics
        publishData();

        // Log data
        logData();
      }

      _mutex.unlock();

    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;
  _Fc.setConstant(0.0f);
  _Tc.setConstant(0.0f);

  publishData();
  msg.data = 0.0f;
  _pubOrientationIntegrator.publish(msg);
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void SurfacePolishing::stopNode(int sig)
{
  me->_stop = true;
}

void SurfacePolishing::computeCommand()
{

  computeProjectionOnSurface();

  computeOriginalDynamics();


  switch(_modulationType)
  {
    case ROTATION:
    {

      rotatingDynamics();
      _vd = _vdR;

      break;
    }
    case ROTATION_AND_FORCE:
    {
      rotatingDynamics();

      updateTankScalars();

      forceModulation();
      
      break;
    }
    default:
    {
      ROS_ERROR("Modulation type does not exist");
      me->_stop = true;

      break;
    }
  }

  computeDesiredOrientation();
}

void SurfacePolishing::computeProjectionOnSurface()
{

  switch(_surfaceType)
  {
    case PLANE:
    {
      _xProj = _x;
      _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_p(0))-_planeNormal(1)*(_xProj(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);

      // Compute _e1 = normal vector pointing towards the surface
      _e1 = -_planeNormal;
      
      // Compute signed normal distance to the plane
      _normalDistance = (_xProj-_x).dot(_e1);
      break;
    }
    case PLANE_OPTITRACK:
    {
      _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS);
      _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS);
      _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS);
      Eigen::Vector3f p13,p12;
      p13 = _p3-_p1;
      p12 = _p2-_p1;
      p13 /= p13.norm();
      p12 /= p12.norm();
      _planeNormal = p13.cross(p12);
      _planeNormal /= _planeNormal.norm();
  
      _xProj = _x;
      _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_p3(0))-_planeNormal(1)*(_xProj(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
      
      // Compute _e1 = normal vector pointing towards the surface
      _e1 = -_planeNormal;
      
      // Compute signed normal distance to the plane
      _normalDistance = (_xProj-_x).dot(_e1);
      break;
    }
    case LEARNED_SURFACE:
    {
      _svm.preComputeKernel(true);
      Eigen::Vector3f x;
      x = _x-(_markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS));
      _normalDistance = _svm.calculateGamma(x.cast<double>());
      _planeNormal = _svm.calculateGammaDerivative(x.cast<double>()).cast<float>();
      _planeNormal.normalize();
      _e1 = -_planeNormal;
       std::cerr << _normalDistance << " " << _e1.transpose() << std::endl;    

      break;
    }
    default:
    {
      break;
    }
  }


  if(_normalDistance < 0.0f)
  {
    _normalDistance = 0.0f;
  }

  // Compute normal force
  Eigen::Vector3f F = _filteredWrench.segment(0,3);
  _normalForce = _e1.dot(-_wRb*F);

}


void SurfacePolishing::computeOriginalDynamics()
{
  // Compute fixed attractor on plane
  if(_surfaceType == PLANE_OPTITRACK)
  {
    _xAttractor = _p1+0.5f*(_p2-_p1)+0.3f*(_p3-_p1);
    _xAttractor(2) = (-_planeNormal(0)*(_xAttractor(0)-_p1(0))-_planeNormal(1)*(_xAttractor(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  }
  else if(_surfaceType == PLANE || _surfaceType == LEARNED_SURFACE)
  {
    _xAttractor = _taskAttractor;
    _xAttractor += _offset;
    _xAttractor(2) = (-_planeNormal(0)*(_xAttractor(0)-_p(0))-_planeNormal(1)*(_xAttractor(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  }
  
  if(_originalDynamics == CONSTANT)
  {
    Eigen::Vector3f dir;
    dir = -_planeNormal;
    dir.normalize();
    _vdOrig = _targetVelocity*dir;
  }
  else if(_originalDynamics == ARBITRARY)
  {
    float alpha = std::tanh(100.0f*_normalDistance);
    Eigen::Vector3f temp;
    temp = _contactAttractor;
    temp(2) = (-_planeNormal(0)*(temp(0)-_p(0))-_planeNormal(1)*(temp(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);

    // _vdOrig = alpha*(_xProj-_x)+_convergenceRate*(1.0f-alpha)*(Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*getCyclingMotionVelocity(_x,_xAttractor);
    // if(_normalDistance> 0)
    // {
      // _vdOrig = _convergenceRate*(_xProj-_x)+_convergenceRate*(1.0f-alpha)*(Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*(_xAttractor-_x);
      _vdOrig = alpha*_convergenceRate*(temp-_x) +_convergenceRate*(1.0f-alpha)*(Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*(_xAttractor-_x);
    // }
    // else
    // {
      // _vdOrig = (1.0f-alpha)*(Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*(_xAttractor-_x);
    // }

      if(fabs(_normalDistance)<FLT_EPSILON)
      {
        _vdOrig = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*(_xAttractor-_x);
      }
      else
      {
        _vdOrig = (temp-_x);
      }
  }

  if(_vdOrig.norm()>_velocityLimit)
  {
    _vdOrig *= _velocityLimit/_vdOrig.norm();
  }
}


Eigen::Vector3f SurfacePolishing::getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
{
  Eigen::Vector3f velocity;

  position = position-attractor;

  velocity(2) = -position(2);

  float R = sqrt(position(0) * position(0) + position(1) * position(1));
  float T = atan2(position(1), position(0));

  float r = 0.05f;
  float omega = M_PI;

  velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
  velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);

  return velocity;
}


void SurfacePolishing::rotatingDynamics()
{
  if(_originalDynamics == CONSTANT)
  {
    Eigen::Vector3f vdContact;
    vdContact = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*getCyclingMotionVelocity(_x,_xAttractor);
    // vdContact = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*(_xAttractor-_x);

    // if((_xAttractor-_x).norm() < 1e-2f)
    // {
    //   _vdR.setConstant(0.0f);
    // }
    // else
    {
      vdContact.normalize();

      // Compute rotation angle
      float angle = std::acos(_vdOrig.normalized().dot(vdContact));
      float theta = (1.0f-std::tanh(10*_normalDistance))*angle;

      // Compute rotation direction
      Eigen::Vector3f u = (_vdOrig.normalized()).cross(vdContact);
      Eigen::Matrix3f K,R;

      if(u.norm() < FLT_EPSILON)
      {
        R.setIdentity();
      }
      else
      {
        u/=u.norm();
        K = Utils::getSkewSymmetricMatrix(u);
        R = Eigen::Matrix3f::Identity()+std::sin(theta)*K+(1.0f-std::cos(theta))*K*K;
      }
      _vdR = R*_vdOrig;
      
    }
    
  }
  else
  {
    _vdR = _vdOrig;
  }
}


void SurfacePolishing::updateTankScalars()
{
  if(_s>_smax)
  {
    _alpha = 0.0f;
  }
  else
  {
    _alpha = 1.0f;
  }

  float dz = 0.01f;
  float ds = 0.1f*_smax;

  _alpha = Utils::smoothFall(_s,_smax-0.1f*_smax,_smax);
  _ut = _v.dot(_vdR);
  if(_s < 0.0f && _ut < 0.0f)
  {
    _beta = 0.0f;
  }
  else if(_s > _smax && _ut > 0.0f)
  {
    _beta = 0.0f;
  }
  else
  {
    _beta = 1.0f;
  }


  _vt = _v.dot(_e1);
  if(_s < 0.0f && _vt > 0.0f)
  {
    _gamma = 0.0f;
  }
  else if(_s > _smax && _vt < 0.0f)
  {
    _gamma = 0.0f;
  }
  else
  {
    _gamma = 1.0f;
  }

  if(_vt<0.0f)
  {
    _gammap = 1.0f;
  }
  else
  {
    _gammap = _gamma;
  }

  std::cerr << "alpha: " << _alpha << " beta: " << _beta << " gamma: " << _gamma << " gammap: " << _gammap << std::endl;

}


void SurfacePolishing::forceModulation()
{
  // Extract linear speed, force and torque data

  // Compute force profile
  if(_lambda1<1.0f)
  {
    _lambda1 = 1.0f;
  }

  if(_normalForce<3.0f)
  {
    _Fd = 5.0f;
  }
  else
  {
    // _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;
    // _Fd = _targetForce*smoothFall(_normalDistance,0.02f,0.1f)/_lambda1;
    _Fd = _targetForce;
    // smoothFall(_distance,0.02f,0.1f)    
    // _Fd = (_targetForce*(1.0-f-std::tanh(100.0f*_normalDistance))+_integratorGain*(_targetForce*(1.0f-std::tanh(100.0f*_normalDistance))+(_wRb*_filteredWrench.segment(0,3)).dot(_e1)))/_lambda1;    

  }
  // _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;

  if(_ensurePassivity)
  {
    _Fd*=_gammap;
  }

  float delta = std::pow(2.0f*_e1.dot(_vdR)*_Fd/_lambda1,2.0f)+4.0f*std::pow(_vdR.norm(),4.0f); 

  float la;

  if(fabs(_vdR.norm())<FLT_EPSILON)
  {
    la = 0.0f;
  }
  else
  {
    la = (-2.0f*_e1.dot(_vdR)*_Fd/_lambda1+sqrt(delta))/(2.0f*std::pow(_vdR.norm(),2.0f));
  }
  
  if(_ensurePassivity && _s < 0.0f && _ut < 0.0f)
  {
    la = 1.0f;
  }

  // Update tank dynamics
  float ds;

  if(_firstDampingMatrix)
  {
    ds = _dt*(_alpha*_v.transpose()*_D*_v-_beta*_lambda1*(la-1.0f)*_ut-_gamma*_Fd*_vt);

    if(_s+ds>=_smax)
    {
      _s = _smax;
    }
    else if(_s+ds<=0.0f)
    {
      _s = 0.0f;
    }
    else
    {
      _s+=ds;
    }
  }

  _dW = _lambda1*(la-1.0f)*(1-_beta)*_ut+_Fd*(_gammap-_gamma)*_vt-(1-_alpha)*_v.transpose()*_D*_v;

  if(_ensurePassivity)
  {
    _vd = la*_vdR+_gammap*_Fd*_e1/_lambda1;
  }
  else
  {
    _vd = la*_vdR+_Fd*_e1/_lambda1;
  }


  std::cerr <<"Measured force: " << _normalForce << " Fd:  " << _Fd*_lambda1 << " vdR: " << _vdR.norm() << std::endl;
  std::cerr << "delta: " << delta << " la: " << la << " vd: " << _vd.norm() << std::endl;

  // Bound desired velocity  
  if(_vd.norm()>_velocityLimit)
  {
    _vd *= _velocityLimit/_vd.norm();
  }

  _Fc.setConstant(0.0f);

  std::cerr << "vd after scaling: " << _vd.norm() << " distance: " << _normalDistance << " v: " << _v.segment(0,3).norm() <<std::endl;
}


void SurfacePolishing::computeDesiredOrientation()
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  
  Eigen::Matrix3f K;
  K << Utils::getSkewSymmetricMatrix(k);

  Eigen::Matrix3f Re;
  if(fabs(s)< FLT_EPSILON)
  {
    Re = Eigen::Matrix3f::Identity();
  }
  else
  {
    Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  }
  
  // Convert rotation error into axis angle representation
  Eigen::Vector3f omega;
  float angle;
  Eigen::Vector4f qtemp = Utils::rotationMatrixToQuaternion(Re);
  Utils::quaternionToAxisAngle(qtemp,omega,angle);

  // Compute final quaternion on plane
  Eigen::Vector4f qf = Utils::quaternionProduct(qtemp,_q);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
  _qd = Utils::slerpQuaternion(_q,qf,1.0f-std::tanh(5.0f*_normalDistance));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*Utils::quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp; 
  // _omegad.setConstant(0.0f);
  // std::cerr << "od:  "<<_omegad.transpose() << std::endl;
  // std::cerr << "o:  "<<_w.transpose()<< std::endl;

  // std::cerr << omega.normalized() << " " << angle  <<std::endl;
  // std::cerr << k.normalized() << std::endl;
  // _integrator += _integratorGain*angle*omega;
  // if(_integrator.norm()> 2*M_PI*_integratorGain)
  // {
  //   _integrator *= 2*M_PI*_integratorGain/_integrator.norm();
  // }
  // _Tc = _integrator;
  // std::cerr << _Tc.transpose() << std::endl;
  // _Tc = _wRb*_filteredWrench.segment(3,3);
    // _Fc = _integratorGain*(Eigen::MatrixXf::Identity(3,3)-_e1*_e1.transpose())*_wRb*_filteredWrench.segment(0,3);
  // _Tc = _integratorGain*_wRb*_filteredWrench.segment(3,3);
}

void SurfacePolishing::logData()
{
    _outputFile << ros::Time::now() << " "
                << _x.transpose() << " "
                << _v.transpose() << " "
                << _vdOrig.transpose() << " "
                << _vdR.transpose() << " "
                << _vd.transpose() << " "
                << _e1.transpose() << " "
                << _wRb.col(2).transpose() << " "
                << (_markersPosition.col(P1)-_markersPosition.col(ROBOT_BASIS)).transpose() << " "
                << _normalDistance << " "
                << _normalForce << " "
                << _Fd*_lambda1 << " "
                << _sequenceID << " "
                << (int) _ensurePassivity << " "
                << _s << " " 
                << _alpha << " "
                << _beta << " "
                << _betap << " "
                << _gamma << " "
                << _gammap << " "
                << _dW << " " << std::endl;
}


void SurfacePolishing::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);

  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);

  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  _msgTaskAttractor.header.frame_id = "world";
  _msgTaskAttractor.header.stamp = ros::Time::now();
  _msgTaskAttractor.point.x = _taskAttractor(0);
  _msgTaskAttractor.point.y = _taskAttractor(1);
  _msgTaskAttractor.point.z = _taskAttractor(2);
  _pubTaskAttractor.publish(_msgTaskAttractor);

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  Eigen::Vector3f center, u,v,n;
  if(_surfaceType == PLANE_OPTITRACK)
  {
    center = _p1+0.5f*(_p2-_p1)+0.2f*(_p3-_p1); 
    u = _p3-_p1;
    v = _p2-_p1;  
  }
  else
  {
    center = _p;
    u << 1.0f, 0.0f, 0.0f;
    v << 0.0f, 1.0f, 0.0f;
  }
  _msgMarker.pose.position.x = center(0);
  _msgMarker.pose.position.y = center(1);
  _msgMarker.pose.position.z = center(2);
  u /= u.norm();
  v /= v.norm();
  n = u.cross(v);
  Eigen::Matrix3f R;
  R.col(0) = u;
  R.col(1) = v;
  R.col(2) = n;
  Eigen::Vector4f q = Utils::rotationMatrixToQuaternion(R);

  _msgMarker.pose.orientation.x = q(1);
  _msgMarker.pose.orientation.y = q(2);
  _msgMarker.pose.orientation.z = q(3);
  _msgMarker.pose.orientation.w = q(0);

  _pubMarker.publish(_msgMarker);

  _msgArrowMarker.points.clear();
  geometry_msgs::Point p1, p2;
  p1.x = _x(0);
  p1.y = _x(1);
  p1.z = _x(2);
  p2.x = _x(0)+0.3f*_e1(0);
  p2.y = _x(1)+0.3f*_e1(1);
  p2.z = _x(2)+0.3f*_e1(2);
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
  _pubMarker.publish(_msgArrowMarker);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);


  _msgDesiredWrench.force.x = _Fc(0);
  _msgDesiredWrench.force.y = _Fc(1);
  _msgDesiredWrench.force.z = _Fc(2);
  _msgDesiredWrench.torque.x = _Tc(0);
  _msgDesiredWrench.torque.y = _Tc(1);
  _msgDesiredWrench.torque.z = _Tc(2);
  _pubDesiredWrench.publish(_msgDesiredWrench);

  std_msgs::Float32 msg;
  msg.data = _normalForce;
  _pubNormalForce.publish(msg);
}


void SurfacePolishing::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  Eigen::Vector3f temp = _x;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = Utils::quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if((_x-temp).norm()>FLT_EPSILON)
  {
    _sequenceID++;
  }

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void SurfacePolishing::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _v(0) = msg->linear.x;
  _v(1) = msg->linear.y;
  _v(2) = msg->linear.z;
  _w(0) = msg->angular.x;
  _w(1) = msg->angular.y;
  _w(2) = msg->angular.z;

  if(!_firstRobotTwist)
  {
    _firstRobotTwist = true;
  }
}
 
void SurfacePolishing::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRobotPose)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _loadOffset.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }

}

void SurfacePolishing::optitrackInitialization()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    if(_markersTracked(ROBOT_BASIS))
    {
      _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
      _averageCount++;
    }
    std::cerr << "Optitrack Initialization count: " << _averageCount << std::endl;
    if(_averageCount == 1)
    {
      ROS_INFO("Optitrack Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      ROS_INFO("Optitrack Initialization done !");
    }
  }
  else
  {
    _optitrackOK = true;
  }
}



void SurfacePolishing::updateOptitrackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  if(!_firstOptitrackRobotPose)
  {
    _firstOptitrackRobotPose = true;
  }

   _markersSequenceID(ROBOT_BASIS) = msg->header.seq;
  _markersTracked(ROBOT_BASIS) = checkTrackedMarker(_markersPosition.col(ROBOT_BASIS)(0),msg->pose.position.x);
  _markersPosition.col(ROBOT_BASIS) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  _markersPosition.col(ROBOT_BASIS)(2) -= 0.03f;
}


void SurfacePolishing::updateOptitrackP1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  if(!_firstOptitrackP1Pose)
  {
    _firstOptitrackP1Pose = true;
  }

  _markersSequenceID(P1) = msg->header.seq;
  _markersTracked(P1) = checkTrackedMarker(_markersPosition.col(P1)(0),msg->pose.position.x);
  _markersPosition.col(P1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}


void SurfacePolishing::updateOptitrackP2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{

  if(!_firstOptitrackP2Pose)
  {
    _firstOptitrackP2Pose = true;
  }

  _markersSequenceID(P2) = msg->header.seq;
  _markersTracked(P2) = checkTrackedMarker(_markersPosition.col(P2)(0),msg->pose.position.x);
  _markersPosition.col(P2) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}


void SurfacePolishing::updateOptitrackP3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  if(!_firstOptitrackP3Pose)
  {
    _firstOptitrackP3Pose = true;
  }

  _markersSequenceID(P3) = msg->header.seq;
  _markersTracked(P3) = checkTrackedMarker(_markersPosition.col(P3)(0),msg->pose.position.x);
  _markersPosition.col(P3) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}


void SurfacePolishing::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg) 
{
  if(!_firstDampingMatrix)
  {
    _firstDampingMatrix = true;
  }

  _D << msg->data[0],msg->data[1],msg->data[2],
        msg->data[3],msg->data[4],msg->data[5],
        msg->data[6],msg->data[7],msg->data[8];
}


uint16_t SurfacePolishing::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


void SurfacePolishing::dynamicReconfigureCallback(force_based_ds_modulation::surfacePolishing_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _integratorGain = config.integratorGain;
  _duration = config.duration;
  _offset(0) = config.xOffset;
  _offset(1) = config.yOffset;
  _offset(2) = config.zOffset;
  _integrator.setConstant(0.0f);
}


void SurfacePolishing::forceModulation2()
{
  // Extract linear speed, force and torque data

  // Compute modulation matrix used to apply a force Fd when the surface is reached while keeping the norm of the velocity constant 
  // M(x) = B(x)L(x)B(x)
  // B(x) = [e1 e2 e3] with e1 = -n is an orthognal basis defining the modulation frame
  //        [la lb lb] 
  // L(x) = [0  lc 0 ] is the matrix defining the modulation gains to apply on the frame
  //        [0  0  lc]

  // Compute modulation frame B(x)
  Eigen::Vector3f xDir;
  xDir << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B;
  _e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*xDir;
  _e2.normalize();
  _e3 = _e1.cross(_e2);
  _e3.normalize();
  B.col(0) = _e1;
  B.col(1) = _e2;
  B.col(2) = _e3;

  // Compute force profile
  if(_lambda1<1.0f)
  {
    _lambda1 = 1.0f;
  }

  if((-_wRb*_filteredWrench.segment(0,3)).dot(_e1)<3.0f)
  {
    _Fd = 5.0f/_lambda1;
  }
  else
  {
    // _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;    
    _Fd = _targetForce/_lambda1;    
    // _Fd = (_targetForce*(1.0f-std::tanh(100.0f*_normalDistance))+_integratorGain*(_targetForce*(1.0f-std::tanh(100.0f*_normalDistance))+(_wRb*_filteredWrench.segment(0,3)).dot(_e1)))/_lambda1;    

  }
  // _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;






  // Compute diagonal gain matrix L(x)
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  float la, lb, lc;

  float temp, delta;

  switch(_formulation)
  {
    case F1:
    {
      temp = (_e2+_e3).dot(_vdR);
      if(fabs(temp)<FLT_EPSILON)
      {
        lb = 0.0f;
      }
      else
      {
        lb = _Fd/temp;
      }

      if(_constraint == VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)-4.0f*std::pow(_vdOrig.norm(),2.0f)*(std::pow(lb*temp,2.0f)-std::pow(_vdOrig.norm(),2.0f));
      }
      else if(_constraint == APPARENT_VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)+4.0f*std::pow(_vdOrig.norm(),4.0f); 
      }

      if(delta < 0.0f)
      {
        delta = 0.0f;
      }

      la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdOrig.norm(),2.0f));

      L(0,0) = la;
      L(0,1) = lb;
      L(0,2) = lb;
      L(1,1) = la;
      L(2,2) = la;

      break;
    }
    
    case F2:
    {
      temp = (_e1+_e2+_e3).dot(_vdR);
      if(fabs(temp)<FLT_EPSILON)
      {
        lb = 0.0f;
      }
      else
      {
        lb = _Fd/temp;
        // lb = _gammap*_Fd/temp;
      }

      if(_constraint == VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)-4.0f*std::pow(_vdR.norm(),2.0f)*(std::pow(lb*temp,2.0f)-std::pow(_vdR.norm(),2.0f));
      }
      else if(_constraint == APPARENT_VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)+4.0f*std::pow(_vdR.norm(),4.0f); 
      }

      if(delta < 0.0f)
      {
        delta = 0.0f;
        la = 0.0f;
      }
      else
      {
        la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdR.norm(),2.0f));
      }

      
      // if(_s < 0.0f && _ut < 0.0f)
      // {
      //   la = 1.0f;
      // }
      // else
      // {
      //   la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdOrig.norm(),2.0f));
      // }


      // Update tank dynamics
      float ds;

      if(_firstDampingMatrix)
      {
        ds = _dt*(_alpha*_v.transpose()*_D*_v-_beta*_lambda1*(la-1.0f)*_ut-_gamma*_Fd*_vt);
        // ds = _dt*(-_beta*_lambda1*(la-1.0f)*_ut-_gamma*_Fd*_vt);
        // ds = _dt*(_alpha*_v.transpose()*_D*_v-_beta*_lambda1*la*_ut-_gamma*_Fd*_vt);
        if(_s+ds>=_smax)
        {
          _s = _smax;
        }
        else if(_s+ds<=0.0f)
        {
          _s = 0.0f;
        }
        else
        {
          _s+=ds;
        }
      }

      float dW;
      dW = _lambda1*(la-1.0f)*(1-_beta)*_ut+_Fd*(_gammap-_gamma)*_vt-(1-_alpha)*_v.transpose()*_D*_v;

      // std::cerr << "Tank: " << _s << " " <<_alpha*_v.transpose()*_D*_v<< " " << -_beta*_lambda1*(la-1.0f)*_ut << " " << -_gamma*_Fd*_vt << std::endl;
      // std::cerr << "at: " << _alpha*_v.transpose()*_D*_v << std::endl;
      // std::cerr << "ut: " << _ut <<  " " << -_beta*_lambda1*(la-1.0f)*_ut << std::endl;
      // std::cerr << "vt: " << _vt << " " << -_gamma*_Fd*_vt << std::endl;
      // std::cerr << "Tank: " << _s  <<" dW: " << dW <<std::endl;

      L(0,0) = la+lb;
      L(0,1) = lb;
      L(0,2) = lb;
      L(1,1) = la;
      L(2,2) = la;

      break;
    }

    case F3:
    {
      temp = (_e1+_e2+_e3).dot(_vdR);
      if(fabs(temp)<FLT_EPSILON)
      {
        lb = 0.0f;
      }
      else
      {
        lb = (_Fd+_e1.dot(_vdR))/temp;
      }

      if(_constraint == VELOCITY_NORM)
      {
        delta = (_vdR.squaredNorm()-std::pow(lb*temp,2.0f))/(std::pow(_e2.dot(_vdR),2.0f)+std::pow(_e3.dot(_vdR),2.0f));
      }
      else if(_constraint == APPARENT_VELOCITY_NORM)
      {
        delta = (_vdR.squaredNorm()-2*lb*temp*_e1.dot(_vdR)+std::pow(_e1.dot(_vdR),2.0f))/(std::pow(_e2.dot(_vdR),2.0f)+std::pow(_e3.dot(_vdR),2.0f));
      }

      if(delta<0)
      {
        delta = 0.0f;
      }
      la = sqrt(delta)-1.0f; 

      L(0,0) = lb;
      L(0,1) = lb;
      L(0,2) = lb;
      L(1,1) = 1.0f+la;
      L(2,2) = 1.0f+la; 

      break;
    }

    default:
    {
      break; 
    }
  }

  // Compute modulation matrix
  Eigen::Matrix3f M;
  M = B*L*B.transpose();

  // Apply force modulation to the rotating dynamics

  if(fabs(_vdR.norm())<FLT_EPSILON)
  {
    _vd = _Fd*_e1;
  }
  else
  {
    _vd = M*_vdR;
  }


  std::cerr <<"Measured force: " << (-_wRb*_filteredWrench.segment(0,3)).dot(_e1) << " Fd:  " << _Fd*_lambda1 << " vdR: " << _vdR.norm() << std::endl;
  std::cerr << "delta: " << delta << " la: " << la << " lb: " << lb << " vd: " << _vd.norm() << std::endl;

  // Bound desired velocity  
  if(_vd.norm()>_velocityLimit)
  {
    _vd *= _velocityLimit/_vd.norm();
  }

  _Fc.setConstant(0.0f);

  std::cerr << "vd after scaling: " << _vd.norm() << " distance: " << _normalDistance << " v: " << _v.segment(0,3).norm() <<std::endl;
}