 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "DroneKalmanFilter.h"
#include "EstimationNode.h"

// constants (variances assumed to be present)
const double varSpeedObservation_xy = 2*2;
const double varPoseObservation_xy = 0.2*0.2;
const double varAccelerationError_xy = 8*8;

const double varPoseObservation_z_PTAM = 0.08*0.08;
const double varPoseObservation_z_IMU = 0.25*0.25;
const double varPoseObservation_z_IMU_NO_PTAM = 0.1*0.1;
const double varAccelerationError_z = 1*1;

const double varPoseObservation_rp_PTAM = 3*3;
const double varPoseObservation_rp_IMU = 1*1;
const double varSpeedError_rp = 360*360 * 16;	// increased because prediction based on control command is damn inaccurate.

const double varSpeedObservation_yaw = 5*5;
const double varPoseObservation_yaw = 3*3;
const double varAccelerationError_yaw = 360*360;


// constants (assumed delays in ms).
// default ping values: nav=25, vid=50
int DroneKalmanFilter::delayRPY = 0;		// always zero
int DroneKalmanFilter::delayXYZ = 40;		// base_delayXYZ, but at most delayVideo
int DroneKalmanFilter::delayVideo = 75;		// base_delayVideo + delayVid - delayNav
int DroneKalmanFilter::delayControl = 100;	// base_delayControl + 2*delayNav

const int DroneKalmanFilter::base_delayXYZ = 40;		// t_xyz - t_rpy = base_delayXYZ
const int DroneKalmanFilter::base_delayVideo = 50;		// t_cam - t_rpy = base_delayVideo + delayVid - delayNav
const int DroneKalmanFilter::base_delayControl = 50;	// t_control + t_rpy - 2*delayNav

// constants (some more parameters)
const double max_z_speed = 2.5;	// maximum height speed tolerated (in m/s, everything else is considered to be due to change in floor-height).
const double scaleUpdate_min_xyDist = 0.5*0.5*0.5*0.5;
const double scaleUpdate_min_zDist = 0.1*0.1;

using namespace std;

// transform degree-angle to satisfy min <= angle < sup
double angleFromTo(double angle, double min, double sup)
{
	while(angle < min) angle += 360;
	while(angle >=  sup) angle -= 360;
	return angle;
}

pthread_mutex_t DroneKalmanFilter::filter_CS = PTHREAD_MUTEX_INITIALIZER;

DroneKalmanFilter::DroneKalmanFilter(EstimationNode* n)
{
	scalePairs = new std::vector<ScaleStruct>();
	navdataQueue = new std::deque<ardrone_autonomy::Navdata>();
	velQueue = new std::deque<geometry_msgs::TwistStamped>();

	useScalingFixpoint = false;

	this->node = n;

	pthread_mutex_lock( &filter_CS );
	reset();
	pthread_mutex_unlock( &filter_CS );

	c1 = 0.58;
	c2 = 17.8;
	c3 = 10;
	c4 = 35;
	c5 = 10;
	c6 = 25;
	c7 = 1.4;
	c8 = 1.0;
}
