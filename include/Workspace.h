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


class Workspace 
{
	public:


	private:

	    std::ifstream _muFile;
	    std::ifstream _priorsFile;
	    std::ifstream _sigmasFile;
	    std::ifstream _thresholdFile;

	    std::vector<Eigen::Vector3f> _mus;
	    std::vector<Eigen::Matrix3f> _sigmas;
	    std::vector<float> _priors;
	    float _threshold;

	public:

		// Class constructor
		Workspace();

		bool init();

		bool isReachable(Eigen::Vector3f);

	private:
	
		float getPdf(Eigen::Vector3f x, Eigen::Vector3f mu, Eigen::Matrix3f sigma);


};


#endif
