#include "SurfacePolishing.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_polishing");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;
  SurfacePolishing::SurfaceType surfaceType;  
  float targetVelocity;
  float targetForce;

  std::ostringstream ss;
  std::string temp;

  // rosrun force_based_ds_modulation surface_polishing fileName -s p/n  
  if(argc == 4) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "p")
    {
      surfaceType = SurfacePolishing::SurfaceType::PLANAR;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "n")
    {
      surfaceType = SurfacePolishing::SurfaceType::NON_FLAT;
    }
    else
    {
      ROS_ERROR("Wrong surface type arguments, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value");
      return 0;
    }
    targetVelocity = 0.2f;
    targetForce = 10.0f;
  }
  else if(argc == 8) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "p")
    {
      surfaceType = SurfacePolishing::SurfaceType::PLANAR;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "n")
    {
      surfaceType = SurfacePolishing::SurfaceType::NON_FLAT;
    }
    else
    {
      ROS_ERROR("Wrong surface type arguments, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[4]) == "-v" && atof(argv[5])> 0.0f)
    {
      targetVelocity = atof(argv[5]);
    }
    else
    {
      ROS_ERROR("Wrong target velocity arguments, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value");
      return 0;
    }  

    if(std::string(argv[6]) == "-f" && atof(argv[7])> 0.0f)
    {
      targetForce = atof(argv[7]);
    }
    else
    {
      ROS_ERROR("Wrong target force arguments, the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value");
      return 0;
    } 
  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName -s(surface type) p(planar) or n(non flat) -v(target velocity) value -f(target force) value");
    return 0;
  }

  ss << "_" << targetVelocity << "_" << targetForce;
  fileName += "_"+std::string(argv[3])+ss.str();
  
  SurfacePolishing surfacePolishing(n,frequency,fileName,surfaceType,targetVelocity,targetForce);

  if (!surfacePolishing.init()) 
  {
    return -1;
  }
  else
  {
   
    surfacePolishing.run();
  }

  return 0;
}

