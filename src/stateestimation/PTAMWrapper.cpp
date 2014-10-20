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

#include "PTAMWrapper.h"

pthread_mutex_t PTAMWrapper::navInfoQueueCS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMWrapper::shallowMapCS = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t PTAMWrapper::logScalePairs_CS = PTHREAD_MUTEX_INITIALIZER;

PTAMWrapper::PTAMWrapper(DroneKalmanFilter* f, EstimationNode* nde)
{
	filter = f;
	node = nde;

	mpMap = 0;
	mpMapMaker = 0;
	mpTracker = 0;
	predConvert = 0;
	predIMUOnlyForScale = 0;
	mpCamera = 0;
	newImageAvailable = false;

	mapPointsTransformed = std::vector<tvec3>();
	keyFramesTransformed = std::vector<tse3>();


	predConvert = new Predictor();
	predIMUOnlyForScale = new Predictor();
	imuOnlyPred = new Predictor();

	drawUI = UI_PRES;
	frameWidth = frameHeight = 0;

	minKFDist = 0;
	minKFWiggleDist = 0;
	minKFTimeDist = 0;

	maxKF = 60;

	logfileScalePairs = 0;
}


void PTAMWrapper::setPTAMPars(double minKFTimeDist, double minKFWiggleDist, double minKFDist){



}
