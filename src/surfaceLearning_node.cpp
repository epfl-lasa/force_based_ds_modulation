#include "SurfaceLearning.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_learning");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;

  SurfaceLearning::Mode mode;

  float C = 100.0f, sigma = 0.2f, epsilonTube = 0.015f;

  bool processRawData = false;
  bool useFullData = false;


  if(argc  >= 4) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "c")
    {
      mode = SurfaceLearning::Mode::COLLECTING_DATA;
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "l" && argc == 14)
    {
      mode = SurfaceLearning::Mode::LEARNING;
      
      if(std::string(argv[4]) == "-c")
      {
        C = atof(argv[5]);
      }
      else
      {
        ROS_ERROR("Wrong C arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing) -c(C) C -s(sigma) s -e(epsilon tube) e -p(process raw data) y(yes) or n(no) -u(use full data) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[6]) == "-s")
      {
        sigma = atof(argv[7]);
      }
      else
      {
        ROS_ERROR("Wrong sigma arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing) -c(C) C -s(sigma) s -e(epsilon tube) e -p(process raw data) y(yes) or n(no) -u(use full data) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[8]) == "-e")
      {
        epsilonTube = atof(argv[9]);
      }
      else
      {
        ROS_ERROR("Wrong epsilon tube arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing) -c(C) C -s(sigma) s -e(epsilon tube) e -p(process raw data) y(yes) or n(no) -u(use full data) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[10]) == "-p" && std::string(argv[11]) == "y")
      {
        processRawData = true;
      }
      else if(std::string(argv[10]) == "-p" && std::string(argv[11]) == "n")
      {
        processRawData = false; 
      }
      else
      {
        ROS_ERROR("Wrong process raw data arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing) -c(C) C -s(sigma) s -e(epsilon tube) e -p(process raw data) y(yes) or n(no) -u(use full data) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[12]) == "-u" && std::string(argv[13]) == "y")
      {
        useFullData = true;
      }
      else if(std::string(argv[12]) == "-u" && std::string(argv[13]) == "n")
      {
        useFullData = false; 
      }
      else
      {
        ROS_ERROR("Wrong process raw data arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing) -c(C) C -s(sigma) s -e(epsilon tube) e -p(process raw data) y(yes) or n(no) -u(use full data) y(yes) or n(no)");
        return 0;
      }
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "l" && argc != 10)
    {

        ROS_ERROR("Wrong process learning mode arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing) -c(C) C -s(sigma) s -e(epsilon tube) e -p(process raw data) y(yes) or n(no) -u(use full data) y(yes) or n(no)");
        return 0;  
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "t")
    {
      mode = SurfaceLearning::Mode::TESTING;
    }
    else
    {
      ROS_ERROR("Wrong mode arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Wrong number of arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing)");
    return 0;
  }

  SurfaceLearning surfaceLearning(n,frequency,fileName,mode,C,sigma,epsilonTube,processRawData,useFullData);

  if (!surfaceLearning.init()) 
  {
    return -1;
  }
  else
  {
    surfaceLearning.run();
  }

  return 0;
}

