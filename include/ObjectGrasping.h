#ifndef __OBJECT_GRASPING_H__
#define __OBJECT_GRASPING_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <dynamic_reconfigure/server.h>
#include "force_based_ds_modulation/objectGrasping_paramsConfig.h"
#include "Eigen/Eigen"
#include "sg_filter.h"
#include "Workspace.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define NB_ROBOTS 2
#define TOTAL_NB_MARKERS 6
#define WINDOW_SIZE 10

class ObjectGrasping 
{
	public:
    // Exection mode
    // REACHING_GRASPING_ONLY: The two robots reach and grasp the object
    // REACHING_GRASPING_MANIPULATING: The two robots reach, grasp and move the object
    //                                 to a predefined position
    enum Mode {REACHING_GRASPING = 0, REACHING_GRASPING_MANIPULATING = 1};

    // Robot ID, left or right
	enum ROBOT {LEFT = 0, RIGHT = 1};

    // Optitrack makers ID
  	enum MarkersID {ROBOT_BASIS_LEFT = 0, ROBOT_BASIS_RIGHT = 1, P1 = 2, P2 = 3, P3 = 4, P4 = 5};

	private:

		// ROS variables
		ros::NodeHandle _nh;
		ros::Rate _loopRate;
		float _dt;

    // Subscribers declarations
    ros::Subscriber _subRobotPose[NB_ROBOTS];             // robot pose
    ros::Subscriber _subRobotTwist[NB_ROBOTS];            // robot twist
    ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];     // force torque sensor
    ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];  // optitrack markers pose
    ros::Subscriber _subDampingMatrix[NB_ROBOTS];         // Damping matrix of DS-impedance controller

    // Publisher declaration
    ros::Publisher _pubDesiredTwist[NB_ROBOTS];         // Desired twist to DS-impdedance controller
    ros::Publisher _pubDesiredOrientation[NB_ROBOTS];   // Desired orientation to DS-impedance controller
    ros::Publisher _pubFilteredWrench[NB_ROBOTS];       // Filtered measured wrench
    ros::Publisher _pubNormalForce[NB_ROBOTS];          // Measured normal force to the surface
    ros::Publisher _pubMarker;                          // Marker (RVIZ) 
    ros::Publisher _pubTaskAttractor;                   // Attractor on surface (RVIZ)
    
    // Subsciber and publisher messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    visualization_msgs::Marker _msgMarker;
    visualization_msgs::Marker _msgArrowMarker;
    geometry_msgs::PointStamped _msgTaskAttractor;
    geometry_msgs::WrenchStamped _msgFilteredWrench;
    
    // Tool characteristics
    float _toolMass;                            // Tool mass [kg]
    float _toolOffsetFromEE;                    // Tool offset along z axis of end effector [m]             
    Eigen::Vector3f _toolComPositionFromSensor; // Offset of the tool [m] (3x1)
    Eigen::Vector3f _gravity;                   // Gravity vector [m/s^2] (3x1)
    Eigen::Vector3f _objectDim;                 // Object dimensions [m] (3x1)
   
    // Tool state variables
    Eigen::Vector3f _x[NB_ROBOTS];                         // Position [m] (3x1)
    Eigen::Vector4f _q[NB_ROBOTS];                         // Current quaternion (4x1)
    Eigen::Vector4f _qinit[NB_ROBOTS];                     // Initial quaternion (4x1)
    Eigen::Matrix3f _wRb[NB_ROBOTS];                       // Orientation matrix (3x1)
    Eigen::Vector3f _v[NB_ROBOTS];                         // Velocity [m/s] (3x1)
    Eigen::Vector3f _w[NB_ROBOTS];                         // Angular velocity [rad/s] (3x1)
    Eigen::Matrix<float,6,1> _wrench[NB_ROBOTS];           // Wrench [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _wrenchBias[NB_ROBOTS];       // Wrench bias [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _filteredWrench[NB_ROBOTS];   // Filtered wrench [N and Nm] (6x1)
    float _normalForce[NB_ROBOTS];                         // Normal force to the surface [N] 
    int _wrenchCount[NB_ROBOTS];

    // Tool control variables
    Eigen::Vector3f _xd[NB_ROBOTS];        // Desired position [m] (3x1)
    Eigen::Vector4f _qd[NB_ROBOTS];        // Desired quaternion (4x1)
    Eigen::Vector4f _qdPrev[NB_ROBOTS];    // Desired previous quaternion (4x1)
    Eigen::Vector3f _omegad[NB_ROBOTS];    // Desired angular velocity [rad/s] (3x1)
    Eigen::Vector3f _fx[NB_ROBOTS];        // Nominal DS [m/s] (3x1)
    Eigen::Vector3f _fxc[NB_ROBOTS];       // Desired conservative part of the nominal DS [m/s] (3x1)
    Eigen::Vector3f _fxr[NB_ROBOTS];       // Desired non-conservative part of the nominal DS [m/s] (3x1)
    Eigen::Vector3f _fxt[NB_ROBOTS];       // Modulation velocity term along tangential direction [m/s] (3x1)
    Eigen::Vector3f _fxn[NB_ROBOTS];       // Modulation velocity term along normal direction [m/s] (3x1)
    Eigen::Vector3f _fxp[NB_ROBOTS];       // Corrected nominal DS to ensure passivity [m/s] (3x1)
    Eigen::Vector3f _fxtp[NB_ROBOTS];      // Corrected modulation term along tangential direction to ensure passivity [m/s] (3x1)
    Eigen::Vector3f _fxnp[NB_ROBOTS];      // Corrected modulation term along normal direction to ensure passivity [m/s] (3x1)
    Eigen::Vector3f _vd[NB_ROBOTS];        // Desired modulated DS [m/s] (3x1)
    float _targetForce;                    // Target force in contact [N]
    float _Fd[NB_ROBOTS];                                  // Desired force profile [N]
    float _Fdp[NB_ROBOTS];                                 // Corrected desired force profile to ensure passivity [N]
    float _sigmac;

    // Task variables
    Eigen::Vector3f _taskAttractor;   // Attractor position for the object [m] (3x1)
    Eigen::Vector3f _n[NB_ROBOTS];   // Normal vector to surface object for each robot (3x1)
    Eigen::Vector3f _xC;              // Center position between the two robots [m] (3x1)
    Eigen::Vector3f _xD;              // Distance vector between the two robots [m] (3x1)
    Eigen::Vector3f _xoC;             // Measured object center position [m] (3x1)
    Eigen::Vector3f _xoD;             // Measured object dimension vector [m] (3x1)
    Eigen::Vector3f _xhC;             // Home robots' center positon [m] (3x1)
    Eigen::Vector3f _xhD;             // Home robots' distance vector [m] (3x1)
    Eigen::Vector3f _xdC;             // Desired center position [m] (3x1)
    Eigen::Vector3f _xdD;             // Desired distance vector [m] (3x1)
    Eigen::Vector3f _vdC;             // Desired center position dynamics [m/s] (3x1)
    Eigen::Vector3f _vdD;             // Desired distance vector dynamics [m/s] (3x1)
    float _eD;                        // Error to desired distance vector [m]                       
    float _eoD;                       // Error to object dimension vector [m]                       
    float _eC;                        // Error to desired center position [m]
    float _eoC;                       // Error to object center position [m]  
 
    // Booleans
    bool _firstRobotPose[NB_ROBOTS];                    // Monitor the first robot pose update
    bool _firstRobotTwist[NB_ROBOTS];                   // Monitor the first robot twist update
    bool _firstWrenchReceived[NB_ROBOTS];               // Monitor first force/torque data update
    bool _firstOptitrackPose[TOTAL_NB_MARKERS];         // Monitor first optitrack markers update
    bool _firstDampingMatrix[NB_ROBOTS];                // Monitor first damping matrix update
    bool _firstObjectPose;                              // Monitor first object pose update
    bool _optitrackOK;                                  // Check if all markers position is received
    bool _wrenchBiasOK[NB_ROBOTS];                      // Check if computation of force/torque sensor bias is OK
    bool _stop;                                         // Check for CTRL+C
    bool _objectGrasped;                                // Check if the object is grasped
    bool _objectReachable;                              // Check if object is reachable by both robots
    bool _goHome;                                       // check for goHome state (object not reachable+ not grasped)
    bool _useForceSensor;                               // check for goHome state (object not reachable+ not grasped)
    bool _adaptNormalModulation;
    // Optitrack variables
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;      // Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;  // Markers sequence ID
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;     // Markers tracked state
    Eigen::Vector3f _p1;                                            // First marker position in the right robot frame
    Eigen::Vector3f _p2;                                            // Second marker position in the right robot frame
    Eigen::Vector3f _p3;                                            // Third marker position in the right robot frame
    Eigen::Vector3f _p4;                                            // Fourth marker position in the right robot frame
    Eigen::Vector3f _leftRobotOrigin;                               // Left robot basis position in the right robot frame

    // Tank parameters
    float _s[NB_ROBOTS];         // Current tank level
    float _smax;                 // Max tank level
    float _alpha[NB_ROBOTS];           // Scalar variable controlling the dissipated energy flow
    float _betar[NB_ROBOTS];           // Scalar variable controlling the energy flow due to the non-conservative part of the nominal DS
    float _betarp[NB_ROBOTS];      // Scalar variable correcting the non-conservative part of the nominal DS to ensure passivity
    float _betat[NB_ROBOTS];         // Scalar variable controlling the energy flow due to the modulation term along the tangential direction to the surface       
    float _betatp[NB_ROBOTS];      // Scalar variable correcting the modulation term along the tangential direction to the surface to ensure passivity
    float _betan[NB_ROBOTS];     // Scalar variable controlling the energy flow due to the modulation term along the normal direction to the surface
    float _betanp[NB_ROBOTS];    // Scalar variable correcting the modulation term along the normal direction to the surface to ensure passivity
    float _pr[NB_ROBOTS];              // Power due to the non-conservative part of the nominal DS
    float _pt[NB_ROBOTS];              // Power due to the modulation term along the tangential direction to the surface
    float _pn[NB_ROBOTS];              // Power due to the modulation term along the normal direction to the surface
    float _pd[NB_ROBOTS];              // Dissipated power
    float _dW[NB_ROBOTS];              // Robot's power flow

    // User variables
    float _velocityLimit;           // Velocity limit [m/s]
    float _filteredForceGain;       // Filtering gain for force/torque sensor
    Eigen::Vector3f _offset;        // Object attractor offset on surface [m] (3x1)
    float _graspingForceThreshold;  // Grasping force threshold [N]

    // Other variables
    float _d1[NB_ROBOTS];
    Eigen::Matrix3f _D[NB_ROBOTS];
    uint32_t _sequenceID;
    uint32_t _averageCount = 0;
    Mode _mode;
    Workspace _workspace;
    SGF::SavitzkyGolayFilter _xCFilter;
    SGF::SavitzkyGolayFilter _xDFilter;
    SGF::SavitzkyGolayFilter _zDirFilter;
    std::mutex _mutex;
    std::string _filename;
    std::ifstream _inputFile;
    std::ofstream _outputFile;
    static ObjectGrasping* me;

    // Dynamic reconfigure (server+callback)
    dynamic_reconfigure::Server<force_based_ds_modulation::objectGrasping_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<force_based_ds_modulation::objectGrasping_paramsConfig>::CallbackType _dynRecCallback;

    float _deltaF[NB_ROBOTS];
    float _epsilonF;
    float _epsilonF0;
    float _gammaF;

    std::deque<float> _normalForceWindow[NB_ROBOTS];


  public:

    // Class constructor
		ObjectGrasping(ros::NodeHandle &n, double frequency, std::string filename, Mode mode, float targetForce, bool adaptNormalModulation);

    // initialize node
		bool init();

    // Run node
		void run();

	private:

    // Callback called when CTRL is detected to stop the node		
		static void stopNode(int sig);

    // Compute command to be sent to the DS-impedance controller
    void computeCommand();

    // Compute object pose (position+orientation)
    void computeObjectPose();

    // Check if object is reachable
    void isObjectReachable();

    // Update contact state with the surface
    void updateContactState();
                
    // Compute desired contact force profile
    void computeDesiredContactForceProfile();

    // Compute modulation terms along tangential and normal direction to the surface
    void computeModulationTerms();
    
    // Compute nominal DS
    void computeNominalDS();

    // Update scalar variables controlling the tank dynamics
    void updateTankScalars();

    // Compute modulated DS
    void computeModulatedDS();

    // Compute desired orientation
    void computeDesiredOrientation();
    
    // Log data to text file
    void logData();

    // Publish data to topics
    void publishData();

      // Compute inital markers positon
    void optitrackInitialization();

    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    // Callback to update markers pose from Optitrack
    void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    // Check if the marker is tracked
    uint16_t checkTrackedMarker(float a, float b);

    // Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(force_based_ds_modulation::objectGrasping_paramsConfig &config, uint32_t level);
};


#endif
