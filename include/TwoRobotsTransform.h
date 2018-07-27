#ifndef __TWO_ROBOTS_TRANSFORM_H__
#define __TWO_ROBOTS_TRANSFORM_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <signal.h>
#include "Eigen/Eigen"


#define NB_ROBOTS 2
#define AVERAGE_COUNT 100

using namespace std;

class TwoRobotsTransform
{
  public:
    enum Mode {SIM = 0, REAL = 1};

  private:
    enum ROBOT {LEFT = 0, RIGHT = 1};

  //!ros variables

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

  //!subscribers and publishers declaration
    ros::Subscriber _subOptitrackPose[NB_ROBOTS];          
    
    Eigen::Matrix<float,3,NB_ROBOTS> _markersPosition;
    Eigen::Matrix<float,3,NB_ROBOTS> _markersPosition0;
    Eigen::Matrix<uint32_t,NB_ROBOTS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,NB_ROBOTS,1> _markersTracked;

    //!boolean variables
    bool _stop;
    bool _firstOptitrackPose[NB_ROBOTS];
    bool _optitrackOK;

    static TwoRobotsTransform* me;
    
    Mode _mode;
    uint32_t _sequenceID;
    uint32_t _averageCount = 0;

    tf::TransformBroadcaster _br;
    tf::Transform _transform;

    //std::mutex _mutex;
    //ros::WallTime _last_commanded_time;
    
  public:
  TwoRobotsTransform(ros::NodeHandle &n, double frequency, Mode mode);
  ~TwoRobotsTransform();
  bool  init();
  void run();
  void updateTf();
  
  private:
  
  static void stopNode(int sig);

  void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k);
        
  uint16_t checkTrackedMarker(float a, float b);
    
  void optitrackInitialization();


};
#endif  // __TWO_ROBOTS_TRANSFORM_H__
