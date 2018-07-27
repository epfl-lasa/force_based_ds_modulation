#ifndef __OBJECT_GRASPING_H__
#define __OBJECT_GRASPING_H__

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

#include "force_based_ds_modulation/objectGrasping_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include "sg_filter.h"
#include "Workspace.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define NB_ROBOTS 2
#define TOTAL_NB_MARKERS 6

class ObjectGrasping 
{

	public:

    enum ContactDynamics {NONE = 0, LINEAR = 1};
		enum ROBOT {LEFT = 0, RIGHT = 1};
  	enum MarkersID {ROBOT_BASIS_LEFT = 0, ROBOT_BASIS_RIGHT = 1, P1 = 2, P2 = 3, P3 = 4, P4 = 5};

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRobotPose[NB_ROBOTS];						// Subscribe to robot current pose
		ros::Subscriber _subRobotTwist[NB_ROBOTS];						// Subscribe to robot current pose
		ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];				// Subscribe to robot current pose
		ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];
		ros::Subscriber _subDampingMatrix[NB_ROBOTS];

		ros::Publisher _pubDesiredTwist[NB_ROBOTS];				// Publish desired twist
		ros::Publisher _pubDesiredOrientation[NB_ROBOTS];  // Publish desired orientation
		ros::Publisher _pubDesiredWrench[NB_ROBOTS];				// Publish desired twist
		ros::Publisher _pubFilteredWrench[NB_ROBOTS];
		ros::Publisher _pubNormalForce[NB_ROBOTS];
    ros::Publisher _pubMarker;
    ros::Publisher _pubTaskAttractor;
    
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
    Eigen::Vector3f _objectDim;

    // End effector state variables
    Eigen::Vector3f _x[NB_ROBOTS];        // Current position [m] (3x1)
    Eigen::Vector4f _q[NB_ROBOTS];        // Current end effector quaternion (4x1)
    Eigen::Matrix3f _wRb[NB_ROBOTS];        // Current rotation matrix [m] (3x1)
    Eigen::Vector3f _v[NB_ROBOTS];
    Eigen::Vector3f _w[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _wrench[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _wrenchBias[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _filteredWrench[NB_ROBOTS];
    int _wrenchCount[NB_ROBOTS];

    // End effector desired variables
    Eigen::Vector4f _qd[NB_ROBOTS];       // Desired end effector quaternion (4x1)
    Eigen::Vector4f _qinit[NB_ROBOTS];       // Desired end effector quaternion (4x1)
    Eigen::Vector4f _qdPrev[NB_ROBOTS];       // Desired end effector quaternion (4x1)
    Eigen::Vector3f _omegad[NB_ROBOTS];   // Desired angular velocity [rad/s] (3x1)
    Eigen::Vector3f _xd[NB_ROBOTS];       // Desired position [m] (3x1)
    Eigen::Vector3f _vd[NB_ROBOTS];       // Desired velocity [m/s] (3x1)
    Eigen::Vector3f _vdOrig[NB_ROBOTS];
    Eigen::Vector3f _vdR[NB_ROBOTS];
    float _Fd[NB_ROBOTS];
    float _normalForce[NB_ROBOTS];
    Eigen::Vector3f _Fc[NB_ROBOTS];
    Eigen::Vector3f _Tc[NB_ROBOTS];
    Eigen::Vector3f _e1[NB_ROBOTS];

    // Task variables
    Eigen::Vector3f _taskAttractor;
    float _targetForce;
    float _targetVelocity;
    Eigen::Vector3f _xC;
    Eigen::Vector3f _xD;
    Eigen::Vector3f _xoC;
    Eigen::Vector3f _xoD;
    Eigen::Vector3f _xdC;
    Eigen::Vector3f _xdD;
    Eigen::Vector3f _vdC;
    Eigen::Vector3f _vdD;
    float _eD;
    float _eoD;
    float _eC;
    float _eoC;
    
    // Booleans
    bool _firstRobotPose[NB_ROBOTS];  // Monitor the first robot pose update
    bool _firstRobotTwist[NB_ROBOTS]; // Monitor the first robot pose update
    bool _firstWrenchReceived[NB_ROBOTS];
    bool _firstOptitrackPose[TOTAL_NB_MARKERS];
    bool _firstDampingMatrix[NB_ROBOTS];
    bool _optitrackOK;
    bool _wrenchBiasOK[NB_ROBOTS];
    bool _moveToAttractor;
    bool _stop;
    bool _objectGrabbed;
    bool _firstObjectPose;
    bool _ensurePassivity;
    bool _objectReachable;

    // Optitrack 
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;
    Eigen::Vector3f _p1;
    Eigen::Vector3f _p2;
    Eigen::Vector3f _p3;
    Eigen::Vector3f _p4;
    Eigen::Vector3f _leftRobotOrigin;

    // Eigen value of passive ds controller
    float _lambda1[NB_ROBOTS];

    // Other variables
    uint32_t _sequenceID;
    uint32_t _averageCount = 0;

    // Tank parameters
    Eigen::Matrix3f _D[NB_ROBOTS];
    float _s[NB_ROBOTS];
    float _smax;
    float _alpha[NB_ROBOTS];
    float _beta[NB_ROBOTS];
    float _betap[NB_ROBOTS];
    float _gamma[NB_ROBOTS];
    float _gammap[NB_ROBOTS];
    float _ut[NB_ROBOTS];
    float _vt[NB_ROBOTS];
    float _dW[NB_ROBOTS];
  
    float _convergenceRate;
    float _filteredForceGain;
    float _velocityLimit;
    float _grabbingForceThreshold;
    Eigen::Vector3f _offset;

    static ObjectGrasping* me;
    std::mutex _mutex;

    Workspace _workspace;

    SGF::SavitzkyGolayFilter _xCFilter;
    SGF::SavitzkyGolayFilter _xLFilter;
    SGF::SavitzkyGolayFilter _zDirFilter;

    ContactDynamics _contactDynamics;
    
    std::string _filename;
    std::ifstream _inputFile;
    std::ofstream _outputFile;

    // Dynamic reconfigure (server+callback)
    dynamic_reconfigure::Server<force_based_ds_modulation::objectGrasping_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<force_based_ds_modulation::objectGrasping_paramsConfig>::CallbackType _dynRecCallback;


  public:

    // Class constructor
		ObjectGrasping(ros::NodeHandle &n, double frequency, std::string filename, ContactDynamics contactDynamics, float targetVelocity, float targetForce);

		bool init();

		void run();

	private:
		
		static void stopNode(int sig);

    void computeObjectPose();

    void isObjectReachable();
		
    void computeCommand();

		void computeProjectionOnSurface();

    Eigen::Vector3f getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor);

		void computeOriginalDynamics();

		void updateTankScalars();

		void forceModulation();

		void computeDesiredOrientation();
    
    void logData();

    void publishData();
    
		void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k);
		    
		uint16_t checkTrackedMarker(float a, float b);
		
    void optitrackInitialization();

    void dynamicReconfigureCallback(force_based_ds_modulation::objectGrasping_paramsConfig &config, uint32_t level);
};


#endif
