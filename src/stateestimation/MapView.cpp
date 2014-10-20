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
 
#include "MapView.h"
#include "../HelperFunctions.h"
#include <cvd/gl_helpers.h>
#include <gvars3/GStringUtil.h>
#include "Predictor.h"
#include <gvars3/instances.h>
#include "DroneKalmanFilter.h"
#include "PTAMWrapper.h"
#include "EstimationNode.h"

pthread_mutex_t MapView::trailPointsVec_CS = PTHREAD_MUTEX_INITIALIZER; //pthread_mutex_lock( &cs_mutex );

MapView::MapView(DroneKalmanFilter* f, PTAMWrapper* p, EstimationNode* nde)
{
	filter = f;
	ptamWrapper = p;
	node = nde;
	drawUI = UI_PRES;
	resetRequested = false;
	trailPoints = std::vector<TrailPoint>();
	predConvert = new Predictor();
	clearTrail = false;
	resetMapViewFlag = true;
}

