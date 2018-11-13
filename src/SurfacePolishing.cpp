#include "SurfacePolishing.h"
#include "Utils.h"

SurfacePolishing* SurfacePolishing::me = NULL;

SurfacePolishing::SurfacePolishing(ros::NodeHandle &n, double frequency, std::string fileName, 
                                   SurfaceType surfaceType, float targetVelocity, float targetForce):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName),
  _surfaceType(surfaceType),
  _targetVelocity(targetVelocity),
  _targetForce(targetForce)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE = 0.15f;
  _toolMass = 0.0f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);
  _normalDistance = 0.0f;
  _normalForce = 0.0f;


  _xd.setConstant(0.0f);
  _fxc.setConstant(0.0f);
  _fxr.setConstant(0.0f);
  _fx.setConstant(0.0f);
  _fxp.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _Fd = 0.0f;
  _Fdp = 0.0f;
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);
  _lambdaf = 0.0f;

  _p << 0.0f,0.0f,-0.007f;
  // _taskAttractor << -0.65f, 0.05f, -0.007f;
  _taskAttractor << -0.6f, 0.1f, 0.4f;
  _planeNormal << 0.0f, 0.0f, 1.0f;

  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;
  for(int k = 0; k < TOTAL_NB_MARKERS; k++)
  {
    _firstOptitrackPose[k] = false;
  }
  _firstDampingMatrix = false;
  _optitrackOK = false;
  _wrenchBiasOK = false;
  _stop = false;

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _smax = 4.0f;
  _s = _smax;
  _dW = 0.0f;

  _sequenceID = 0;
  _averageCount = 0;
  _wrenchCount = 0;
  _d1 = 0.0f;
  _D.setConstant(0.0f);

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
  p2.x = 0.0f+0.3f*_n(0);
  p2.x = 0.0f+0.3f*_n(1);
  p2.x = 0.0f+0.3f*_n(2);
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
  _subRobotPose = _nh.subscribe("/lwr/ee_pose", 1, &SurfacePolishing::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _nh.subscribe("/lwr/joint_controllers/twist", 1, &SurfacePolishing::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _nh.subscribe("/ft_sensor/netft_data", 1, &SurfacePolishing::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot/pose", 1, boost::bind(&SurfacePolishing::updateOptitrackPose,this,_1,ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p1/pose", 1, boost::bind(&SurfacePolishing::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p2/pose", 1, boost::bind(&SurfacePolishing::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/p3/pose", 1, boost::bind(&SurfacePolishing::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix = _nh.subscribe("/lwr/joint_controllers/passive_ds_damping_matrix", 1, &SurfacePolishing::updateDampingMatrix,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench = _nh.advertise<geometry_msgs::WrenchStamped>("SurfacePolishing/filteredWrench", 1);
  _pubMarker = _nh.advertise<visualization_msgs::Marker>("SurfacePolishing/plane", 1);
  _pubTaskAttractor = _nh.advertise<geometry_msgs::PointStamped>("SurfacePolishing/taskAttractor", 1);  
  _pubNormalForce = _nh.advertise<std_msgs::Float32>("SurfacePolishing/normalForce", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&SurfacePolishing::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,SurfacePolishing::stopNode);


  ROS_INFO("[SurfacePolishing]: Filename: %s", _fileName.c_str());

  _outputFile.open(ros::package::getPath(std::string("force_based_ds_modulation"))+"/data_polishing/"+_fileName+".txt");


  if(!_nh.getParamCached("/lwr/ds_param/damping_eigval0",_d1))
  {
    ROS_ERROR("[SurfacePolishing]: Cannot read first eigen value of passive ds controller");
    return false;
  }

  if(!_outputFile.is_open())
  {
    ROS_ERROR("[SurfacePolishing]: Cannot open output data file, the data_polishing directory might be missing");
    return false;
  }

  if(_surfaceType == PLANAR)
  {
    ROS_INFO("[SurfacePolishing]: Surface type: PLANAR");
  }
  else if(_surfaceType == NON_FLAT)
  {
    ROS_INFO("[SurfacePolishing]: Surface type: NON_FLAT");

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
    ROS_ERROR("[SurfacePolishing]: Surface type not recognized");
    return false;
  }

  if(_targetVelocity>0.0f)
  {
    ROS_INFO("[SurfacePolishing]: Target velocity: %f", _targetVelocity);
  }
  else
  {
    ROS_ERROR("[SurfacePolishing]: Target velocity should be positive");
    return false;
  }

  if(_targetForce>0.0f)
  {
    ROS_INFO("[SurfacePolishing]: Target force: %f", _targetForce);
  }
  else
  {
    ROS_ERROR("[SurfacePolishing]: Target force should be positive");
    return false;
  }

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[SurfacePolishing]: The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[SurfacePolishing]: The ros node has a problem.");
    return false;
  }
}


void SurfacePolishing::run()
{
  _timeInit = ros::Time::now().toSec();

  while (!_stop && ros::Time::now().toSec()-_timeInit < _duration) 
  {
    if(_firstRobotPose && _firstRobotTwist && _wrenchBiasOK &&
       _firstOptitrackPose[ROBOT_BASIS] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3] && _firstDampingMatrix)
    {
      _mutex.lock();

      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_d1);

      // Initialize optitrack
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

  // Send zero velocity command to stop the robot
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;

  publishData();
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
  // Update surface info
  updateSurfaceInformation();

  // Compute nominal DS
  computeNominalDS();

  // Compute modulated DS
  computeModulatedDS();
    
  // Compute desired orientation
  computeDesiredOrientation();
}

void SurfacePolishing::updateSurfaceInformation()
{
  switch(_surfaceType)
  {
    case PLANAR:
    {
      // Compute markers position in the robot frame
      // The three markers are positioned on the surface to form an angle of 90 deg:
      // P1 ----- P2
      // |       
      // |
      // P3
      _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS);
      _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS);
      _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS);
      Eigen::Vector3f p13,p12;

      // Compute main directions between the markers  
      p13 = _p3-_p1;
      p12 = _p2-_p1;
      p13 /= p13.norm();
      p12 /= p12.norm();

      // Compute normal vector 
      _planeNormal = p13.cross(p12);
      _planeNormal /= _planeNormal.norm();
  
      // Compute vertical projection onto the surface
      _xProj = _x;
      _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_p3(0))-_planeNormal(1)*(_xProj(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
      
      // Compute _n = normal vector pointing towards the surface
      _n = -_planeNormal;
      
      // Compute signed normal distance to the plane
      _normalDistance = (_xProj-_x).dot(_n);

      break;
    }
    case NON_FLAT:
    {
      _svm.preComputeKernel(true);

      // The surface is learned with respect to a frame defined by the marker P1
      // We get the robot position in the surface frame
      Eigen::Vector3f x;
      _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS);
      _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS);
      _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS);

      // Compute surface frame, wRs is the rotation matrix for the surface frame to the world frame  
      _wRs.col(0) = (_p1-_p3).normalized();
      _wRs.col(1) = (_p1-_p2).normalized();
      _wRs.col(2) = ((_wRs.col(0)).cross(_wRs.col(1))).normalized();

      // Compute robot postion in surface frame
      x = _wRs.transpose()*(_x-_p1);

      // We compute the normal distance by evlauating the SVM model
      _normalDistance = _svm.calculateGamma(x.cast<double>());

      // We get the normal vector by evaluating the gradient of the model
      _planeNormal = _svm.calculateGammaDerivative(x.cast<double>()).cast<float>();
      _planeNormal = _wRs*_planeNormal;
      _planeNormal.normalize();
      _n = -_planeNormal;
      std::cerr << "[SurfacePolishing]: Normal distance: " << _normalDistance << " Normal vector: " << _n.transpose() << std::endl;    

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
  _normalForce = _n.dot(-_wRb*F);

}


void SurfacePolishing::computeNominalDS()
{
  // Compute fixed attractor on plane
  if(_surfaceType == PLANAR)
  {
    _xAttractor = _p1+0.5f*(_p2-_p1)+0.3f*(_p3-_p1);
    _xAttractor(2) = (-_planeNormal(0)*(_xAttractor(0)-_p1(0))-_planeNormal(1)*(_xAttractor(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  }
  else 
  {
    _xAttractor = _p1+0.45f*(_p2-_p1)+0.5f*(_p3-_p1);
    // _xAttractor += _offset;

    // Compute normal distance and vector at the attractor location in the surface frame
    Eigen::Vector3f x, attractorNormal; 
    x = _wRs.transpose()*(_xAttractor-_p1);

    float attractorNormalDistance = _svm.calculateGamma(x.cast<double>());
    attractorNormal = _svm.calculateGammaDerivative(x.cast<double>()).cast<float>();
    attractorNormal = _wRs*attractorNormal;
    attractorNormal.normalize();

    // Compute attractor normal projection on the surface int the world frame
    _xAttractor -= attractorNormalDistance*attractorNormal;
  }

  // The reaching velocity direction is aligned with the normal vector to the surface
  Eigen::Vector3f v0 = _targetVelocity*_n;
 
  // Compute normalized circular dynamics projected onto the surface
  Eigen::Vector3f vdContact;
  vdContact = (Eigen::Matrix3f::Identity()-_n*_n.transpose())*getCircularMotionVelocity(_x,_xAttractor);
  vdContact.normalize();

  // Compute rotation angle + axis between reaching velocity vector and circular dynamics
  float angle = std::acos(v0.normalized().dot(vdContact));
  float theta = (1.0f-std::tanh(10*_normalDistance))*angle;
  Eigen::Vector3f u = (v0.normalized()).cross(vdContact);

  // Get corresponding rotation matrix
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

  // Compute nominal DS
  _fxc.setConstant(0.0f);
  _fxr = R*v0;
  _fx = _fxc+_fxr;
      


}


Eigen::Vector3f SurfacePolishing::getCircularMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
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


void SurfacePolishing::updateTankScalars()
{
  _alpha = Utils::smoothFall(_s,_smax-0.1f*_smax,_smax);

  _pc = _d1*_v.dot(_fxc);

  if(_s < FLT_EPSILON && _pc < FLT_EPSILON)
  {
    _betac = 0.0f;
  }
  else if(_s > _smax && _pc > FLT_EPSILON)
  {
    _betac = 0.0f;
  }
  else
  {
    _betac = 1.0f;
  }

  if(_pc>FLT_EPSILON)
  {
    _betacp = 1.0f;
  }
  else
  {
    _betacp = _betac;
  }

  _pr = _d1*_v.dot(_fxr);

  if(_s < FLT_EPSILON && _pr > FLT_EPSILON)
  {
    _betar = 0.0f;
  }
  else if(_s > _smax && _pr < FLT_EPSILON)
  {
    _betar = 0.0f;
  }
  else
  {
    _betar = 1.0f;
  }

  if(_pr<FLT_EPSILON)
  {
    _betarp = 1.0f;
  }
  else
  {
    _betarp = _betar;
  }

  _pf = _Fd*_v.dot(_n);
  
  if(_s < FLT_EPSILON && _pf > FLT_EPSILON)
  {
    _gamma = 0.0f;
  }
  else if(_s > _smax && _pf < FLT_EPSILON)
  {
    _gamma = 0.0f;
  }
  else
  {
    _gamma = 1.0f;
  }

  if(_pf<FLT_EPSILON)
  {
    _gammap = 1.0f;
  }
  else
  {
    _gammap = _gamma;
  }
}


void SurfacePolishing::computeModulatedDS()
{
  // Check the first impedance gain of the DS-impedance controller
  if(_d1<1.0f)
  {
    _d1 = 1.0f;
  }

  // Compute desired force profile
  if(_normalForce<5.0f)
  {
    _Fd = 5.0f;
  }
  else
  {
    _Fd = _targetForce;
  }

  // Update tank scalar variables
  updateTankScalars();

  // Compute corrected force profile and nominal DS
  _fxp = _betacp*_fxc+_betarp*_fxr;
  _Fdp = _gammap*_Fd;

  // Compute modulation gain
  float delta = std::pow(2.0f*_n.dot(_fxp)*_Fdp/_d1,2.0f)+4.0f*std::pow(_fxp.norm(),4.0f); 
  if(fabs(_fxp.norm())<FLT_EPSILON)
  {
    _lambdaf = 0.0f;
  }
  else
  {
    _lambdaf = (-2.0f*_n.dot(_fxp)*_Fdp/_d1+sqrt(delta))/(2.0f*std::pow(_fxp.norm(),2.0f));
  }

  // Update tank dynamics
  _pd = _v.transpose()*_D*_v;
  float ds = _dt*(_alpha*_pd-_betac*(_lambdaf-1.0f)*_pc-_betar*_lambdaf*_pr-_gamma*_pf);

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

  // Update robot's power flow
  _dW = (_lambdaf-1.0f)*(_betacp-_betac)*_pc+_lambdaf*(_betarp-_betar)*_pr+(_gammap-_gamma)*_pf-(1-_alpha)*_pd;

  // Compute modulated DS
  _vd = _lambdaf*_fxp+_Fdp*_n/_d1;

  std::cerr << "[SurfacePolishing]: F: " << _normalForce << " Fdp:  " << _Fdp << " ||fx||: " << _fxp.norm() << std::endl;
  std::cerr << "[SurfacePolishing]: lambdaf: " << _lambdaf << " vd: " << _vd.norm() << std::endl;
  std::cerr << "[SurfacePolishing]: Tank: " << _s  <<" dW: " << _dW <<std::endl;


  // Bound modulated DS for safety 
  if(_vd.norm()>_velocityLimit)
  {
    _vd *= _velocityLimit/_vd.norm();
  }

  std::cerr << "[SurfacePolishing]: vd after scaling: " << _vd.norm() << " distance: " << _normalDistance << " v: " << _v.segment(0,3).norm() <<std::endl;
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

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
  _qd = Utils::slerpQuaternion(_q,qf,1.0f-std::tanh(5.0f*_normalDistance));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*Utils::quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp; 
}


void SurfacePolishing::logData()
{
  _outputFile << ros::Time::now() << " "
              << _x.transpose() << " "
              << _v.transpose() << " "
              << _fxc.transpose() << " "
              << _fxr.transpose() << " "
              << _fxp.transpose() << " "
              << _vd.transpose() << " "
              << _n.transpose() << " "
              << _wRb.col(2).transpose() << " "
              << (_markersPosition.col(P1)-_markersPosition.col(ROBOT_BASIS)).transpose() << " "
              << _normalDistance << " "
              << _normalForce << " "
              << _Fd << " "
              << _lambdaf << " "
              << _sequenceID << " "
              << _s << " " 
              << _alpha << " "
              << _betac << " "
              << _betacp << " "
              << _betar << " "
              << _betarp << " "
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
  if(_surfaceType == PLANAR)
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
  p2.x = _x(0)+0.3f*_n(0);
  p2.y = _x(1)+0.3f*_n(1);
  p2.z = _x(2)+0.3f*_n(2);
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
  _x = _x+_toolOffsetFromEE*_wRb.col(2);

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
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "[SurfacePolishing]: Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
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
    std::cerr << "[SurfacePolishing]: Optitrack Initialization count: " << _averageCount << std::endl;
    if(_averageCount == 1)
    {
      ROS_INFO("[SurfacePolishing]: Optitrack Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      ROS_INFO("[SurfacePolishing]: Optitrack Initialization done !");
    }
  }
  else
  {
    _optitrackOK = true;
  }
}

void SurfacePolishing::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(k == (int) ROBOT_BASIS)
  {
    _markersPosition.col(k)(2) -= 0.03f;
  }
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

  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _duration = config.duration;
  _offset(0) = config.xOffset;
  _offset(1) = config.yOffset;
  _offset(2) = config.zOffset;
}