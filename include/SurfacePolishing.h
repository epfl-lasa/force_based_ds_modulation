#ifndef __SURFACE_POLISHING_H__
#define __SURFACE_POLISHING_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <dynamic_reconfigure/server.h>
#include "force_based_ds_modulation/surfacePolishing_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "svm_grad.h"
#include "std_msgs/Float32MultiArray.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define TOTAL_NB_MARKERS 4

class SurfacePolishing 
{

	public:

		enum SurfaceType {PLANE = 0, PLANE_OPTITRACK = 1, LEARNED_SURFACE = 2};
		enum OriginalDynamics {CONSTANT = 0, ARBITRARY = 1};
		enum ModulationType	{ROTATION = 0, ROTATION_AND_FORCE = 1};
		enum Formulation {F1 = 0, F2 = 1, F3 = 2};
		enum Constraint {VELOCITY_NORM = 0, APPARENT_VELOCITY_NORM = 1};
    enum MarkersID {ROBOT_BASIS = 0, P1 = 1, P2 = 2, P3 = 3};

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRobotPose;						// Subscribe to robot current pose
		ros::Subscriber _subRobotTwist;						// Subscribe to robot current pose
		ros::Subscriber _subForceTorqueSensor;				// Subscribe to robot current pose
		ros::Subscriber _subOptitrackRobotBasisPose;
		ros::Subscriber _subOptitrackPlane1Pose;
		ros::Subscriber _subOptitrackPlane2Pose;
		ros::Subscriber _subOptitrackPlane3Pose;
		ros::Subscriber _subDampingMatrix;

		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
		ros::Publisher _pubDesiredWrench;				// Publish desired twist
		ros::Publisher _pubFilteredWrench;
		ros::Publisher _pubMarker;
		ros::Publisher _pubTaskAttractor;
		ros::Publisher _pubNormalForce;
		ros::Publisher _pubOrientationIntegrator;
		
		// Subsciber and publisher messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::Wrench _msgDesiredWrench;
		visualization_msgs::Marker _msgMarker;
		visualization_msgs::Marker _msgArrowMarker;

		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		
		// Tool variables
		float _loadMass;
		float _toolOffset;
		Eigen::Vector3f _loadOffset;
		Eigen::Vector3f _gravity;

		// End effector state variables
		Eigen::Vector3f _x;				// Current position [m] (3x1)
		Eigen::Vector4f _q;				// Current end effector quaternion (4x1)
		Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
		Eigen::Vector3f _v;
		Eigen::Vector3f _w;
		Eigen::Matrix<float,6,1> _wrench;
		Eigen::Matrix<float,6,1> _wrenchBias;
		Eigen::Matrix<float,6,1> _filteredWrench;
		float _filteredForceGain;
		int _wrenchCount = 0;

		// End effector desired variables
		Eigen::Vector4f _qd;				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _omegad;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _xd;				// Desired position [m] (3x1)
		Eigen::Vector3f _vd;				// Desired velocity [m/s] (3x1)
		Eigen::Vector3f _vdOrig;
		Eigen::Vector3f _vdR;
		float _targetForce;
		float _velocityLimit;
		float _targetVelocity;
		float _Fd;

		// Task variables
    Eigen::Vector3f _taskAttractor;
    Eigen::Vector3f _contactAttractor;
    Eigen::Vector3f _planeNormal;
    Eigen::Vector3f _e1;
    Eigen::Vector3f _e2;
    Eigen::Vector3f _e3;
    Eigen::Vector3f _p;
    Eigen::Vector3f _xProj;		
    Eigen::Vector3f _xAttractor;  
    float _vInit; 
    float _normalDistance;
    float _normalForce;
    double _duration;
    double _timeInit;
    Eigen::Vector3f _offset;

		// Control variables
    float _convergenceRate;       // Convergence rate of the DS
		Eigen::Vector3f _Fc;
		Eigen::Vector3f _Tc;

		float _integratorGain;
		Eigen::Vector3f _integrator;
		
    // Booleans
		bool _firstRobotPose;	// Monitor the first robot pose update
		bool _firstRobotTwist;	// Monitor the first robot pose update
		bool _firstWrenchReceived;
		bool _firstOptitrackRobotPose;
		bool _firstOptitrackP1Pose;
		bool _firstOptitrackP2Pose;
		bool _firstOptitrackP3Pose;
		bool _firstDampingMatrix;
		bool _optitrackOK;
		bool _wrenchBiasOK;
  		bool _stop;
  		bool _ensurePassivity;

    // Optitrack 
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;
		Eigen::Vector3f _p1;
		Eigen::Vector3f _p2;
		Eigen::Vector3f _p3;
		std::string _fileName;

		// Eigen value of passive ds controller
		float _lambda1;

		// Other variables
		SurfaceType _surfaceType;
		OriginalDynamics _originalDynamics;
		ModulationType _modulationType;
		Formulation _formulation;
		Constraint _constraint;
		uint32_t _sequenceID;
		uint32_t _averageCount = 0;

		static SurfacePolishing* me;
		std::mutex _mutex;

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<force_based_ds_modulation::surfacePolishing_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<force_based_ds_modulation::surfacePolishing_paramsConfig>::CallbackType _dynRecCallback;

		std::ifstream _inputFile;
		std::ofstream _outputFile;
		SVMGrad _svm;

		// Tank parameters
		Eigen::Matrix3f _D;
		float _s;
		float _smax;
		float _alpha;
		float _beta;
		float _betap;
		float _gamma;
		float _gammap;
		float _ut;
		float _vt;
		float _dW;
	public:

		// Class constructor
		SurfacePolishing(ros::NodeHandle &n, double frequency, std::string fileName, SurfaceType surfaceType, 
			          OriginalDynamics originalDynamics, ModulationType modulationType,
			          Formulation formulation, Constraint constraint, float targetVelocity, float targetForce);

		bool init();

		void run();

	private:
		
		static void stopNode(int sig);

    void computeCommand();

		void computeProjectionOnSurface();

		void computeOriginalDynamics();

		Eigen::Vector3f getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor);

		void rotatingDynamics();

		void updateTankScalars();

		void forceModulation();

		void forceModulation2();

		void computeDesiredOrientation();
    
    void logData();

    void publishData();
    
    void optitrackInitialization();

    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

		void updateOptitrackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updateOptitrackP1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updateOptitrackP2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updateOptitrackP3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 

		uint16_t checkTrackedMarker(float a, float b);

    void dynamicReconfigureCallback(force_based_ds_modulation::surfacePolishing_paramsConfig &config, uint32_t level);
};


#endif
