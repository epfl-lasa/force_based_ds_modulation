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
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "armadillo"
#include "svm_grad.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define TOTAL_NB_MARKERS 4
#define DATASET_SIZE 30000


class SurfaceLearning 
{
	public:
		// Execution mode
		// COLLECTING_DATA: The user brings the robot end effector with force torque sensor mounted
		//                  in contact with the surface and swap the surface while applying a bit of force
		// LEARNING: Learn the surface model using SVM
		// TESTING: Test the learned model: The z axis of the end effector should align with the normal to the surface
		//                                  The normal distance is printed in the terminal with the normal vector learned                                     		
		enum Mode {COLLECTING_DATA = 0, LEARNING = 1, TESTING = 2};
		// Optitrack makers ID
    enum MarkersID {ROBOT_BASIS = 0, P1 = 1, P2 = 2, P3 = 3};

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers declaration
		ros::Subscriber _subRobotPose;												// robot pose
		ros::Subscriber _subRobotTwist;												// robot twist
		ros::Subscriber _subForceTorqueSensor;								// force torque sensor
		ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];	// optitrack markers pose

		// Publishers declaration
		ros::Publisher _pubDesiredTwist;						// Desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation;  		// Desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench;					// Filtered measured wrench
		ros::Publisher _pubMarker;						  		// Marker (RVIZ) 
		
		// Messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		visualization_msgs::Marker _msgMarker;
		visualization_msgs::Marker _msgArrowMarker;
    geometry_msgs::WrenchStamped _msgFilteredWrench;

		// Tool characteristics
		float _toolMass;														// Tool mass [kg]
		float _toolOffsetFromEE;										// Tool offset along z axis of end effector [m]							
		Eigen::Vector3f _toolComPositionFromSensor; // Offset of the tool [m]	(3x1)
		Eigen::Vector3f _gravity;										// Gravity vector [m/s^2] (3x1)

		// Tool state variables
		Eigen::Vector3f _x;													// Position [m] (3x1)
		Eigen::Vector4f _q;													// Quaternion (4x1)
		Eigen::Matrix3f _wRb;												// Orientation matrix (3x1)
		Eigen::Vector3f _v;													// Velocity [m/s] (3x1)
		Eigen::Vector3f _w;													// Angular velocity [rad/s] (3x1)
		Eigen::Matrix<float,6,1> _wrench;						// Wrench [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _wrenchBias;				// Wrench bias [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _filteredWrench;		// Filtered wrench [N and Nm] (6x1)
    float _normalDistance;											// Normal distance to the surface [m]
    float _normalForce;													// Normal force to the surface [N]

		// Tool control variables
		Eigen::Vector3f _xd;				// Desired position [m] (3x1)
		Eigen::Vector4f _qd;				// Desired quaternion (4x1)
		Eigen::Vector3f _omegad;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _vd;				// Desired modulated velocity [m/s] (3x1)
		float _targetForce;					// Target force in contact [N]
		float _targetVelocity;			// Velocity norm of the nominal DS [m/s]
		float _Fd;									// Desired force profile

		// Task variables
    Eigen::Vector3f _planeNormal;					// Normal vector to the surface (pointing outside the surface) (3x1)
    Eigen::Vector3f _e1;									// Normal vector to the surface (pointing towards the surface) (3x1)

    // Booleans
		bool _firstRobotPose;																// Monitor the first robot pose update
		bool _firstRobotTwist;															// Monitor the first robot twist update
		bool _firstWrenchReceived;													// Monitor first force/torque data update
    bool _firstOptitrackPose[TOTAL_NB_MARKERS];					// Monitor first optitrack markers update
		bool _optitrackOK;																	// Check if all markers position is received
		bool _wrenchBiasOK;																	// Check if computation of force/torque sensor bias is OK
		bool _stop;																					// Check for CTRL+C
		bool _useOptitrack;

    // Optitrack variables
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;			// Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;	// Markers sequence ID
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;			// Markers tracked state
		Eigen::Vector3f _p1;																						// First marker position in the robot frame
		Eigen::Vector3f _p2;																						// Second marker position in the robot frame
		Eigen::Vector3f _p3;																						// Third marker position in the robot frame

		// SVM parameters
		float _C;									// C value (penalty factor)
		float _sigma;							// Width of the gaussian kernel [m]
		float _epsilonTube;				// Epsilon tube width
  	bool _generateDataset;		// Generate dataset (input = position in the surface frame / output = normal distance)
  	bool _addDataOnSurface;		// Add data samples collected on the surface to the dataset
  	float _forceThreshold;    // Force threshold used to generate the dataset [N]
  	float _heightThreshold;   // Height threshold used to generate the dataset [m]
  	float _heightOffset; 			// Height offset used to generate the dataset [m]

		// Other variables
		Mode _mode;										// Execution mode
		SVMGrad _svm;									// SVM gradient object
		uint32_t _averageCount = 0;
		int _wrenchCount = 0;
		uint32_t _sequenceID;
		float _filteredForceGain;
		std::string _fileName;
		std::ifstream _inputFile;
		std::ofstream _outputFile;
		std::mutex _mutex;
		static SurfaceLearning* me;


	public:

		// Class constructor
		SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName, 
			              Mode mode, float C, float sigma, float epsilonTube,
			              bool generateDataset, bool addDataOnSurface);
		
		// Initialize node
		bool init();

		// Run node
		void run();

	private:		
		
		// Callback called when CTRL is detected to stop the node
		static void stopNode(int sig);
		
		// Compute command to be sent to the DS-impedance controller
    void computeCommand();

		// Compute desired orientation
		void computeDesiredOrientation();

		// Learn SVR model
		void learnSurfaceModel();

	  // Generate input file needed by SVMGrad library from output file generated by libsvm
		void generateSVMGradModelFile();
    
    // Log raw data
    void collectData();

    // Generate dataset from collected raw data
    void generateDataset();

    // Publish data to topics
    void publishData();

    // Compute inital markers positon
    void optitrackInitialization();

    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // Callback to update markers pose from Optitrack
		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

		// Check if the marker is tracked
		uint16_t checkTrackedMarker(float a, float b);
};


#endif
