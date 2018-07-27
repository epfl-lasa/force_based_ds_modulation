#ifndef __SURFACE_LEARNING_H__
#define __SURFACE_LEARNING_H__

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

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "armadillo"
#include "svm_grad.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define TOTAL_NB_MARKERS 4


class SurfaceLearning 
{
	public:

		enum Mode {COLLECTING_DATA = 0, LEARNING = 1, TESTING = 2};
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


		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
		ros::Publisher _pubDesiredWrench;				// Publish desired twist
		ros::Publisher _pubFilteredWrench;
		ros::Publisher _pubMarker;
		
		// Subsciber and publisher messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::Wrench _msgDesiredWrench;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		visualization_msgs::Marker _msgArrowMarker;

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
		float _normalDistance;

		// End effector desired variables
		Eigen::Vector4f _qd;				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _omegad;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _xd;				// Desired position [m] (3x1)
		Eigen::Vector3f _vd;				// Desired velocity [m/s] (3x1)
		Eigen::Vector3f _e1;
		Eigen::Vector3f _e2;
		Eigen::Vector3f _e3;
		Eigen::Vector3f _xAttractor;
		float _lambda1;
		float _Fd;
		float _forceThreshold;

		float _C;
		float _sigma;
		float _epsilonTube;


		// Control variables
    float _convergenceRate;       // Convergence rate of the DS
		Eigen::Vector3f _Fc;
		
    // Booleans
		bool _firstRobotPose;	// Monitor the first robot pose update
		bool _firstRobotTwist;	// Monitor the first robot pose update
		bool _firstWrenchReceived;
		bool _wrenchBiasOK;
  	bool _stop;
  	bool _processRawData;
  	bool _useFullData;
  	bool _useOptitrack;

    bool _firstOptitrackRobotPose;
    bool _firstOptitrackP1Pose;
    bool _firstOptitrackP2Pose;
    bool _firstOptitrackP3Pose;
		bool _optitrackOK;

    uint32_t _sequenceID;

		std::string _fileName;


		static SurfaceLearning* me;
		std::mutex _mutex;


		std::ofstream _outputFile;
		std::ifstream _inputFile;
		SVMGrad _svm;
		Mode _mode;

		Eigen::Vector3f _vdOrig;

		Eigen::Vector3f _vdR;

		std::vector<Eigen::Vector3f> surfaceData;

    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;
		uint32_t _averageCount = 0;




	public:

		// Class constructor
		SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName, Mode mode, float C, float sigma, float epsilonTube,bool processRawData, bool useFullData);

		bool init();

		void run();

	private:
		
	static void stopNode(int sig);
		
    void computeCommand();

		void computeDesiredOrientation();

		void learnSurfaceModel();

		void generateSVMGradModelFile();
    
    void logData();

    void processRawData();

    void publishData();


    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void optitrackInitialization();

		void updateOptitrackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updateOptitrackP1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updateOptitrackP2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updateOptitrackP3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		uint16_t checkTrackedMarker(float a, float b);

    Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

    Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

    Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

  	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

		void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);

  	Eigen::Vector4f slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t);
};


#endif
