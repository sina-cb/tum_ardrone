#include "EstimationNode.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "../HelperFunctions.h"
#include "DroneKalmanFilter.h"
#include <ardrone_autonomy/Navdata.h>
#include "deque"
#include "tum_ardrone/filter_state.h"
#include "PTAMWrapper.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "MapView.h"
#include <sys/stat.h>
#include <string>

EstimationNode::EstimationNode(){


}

void EstimationNode::dynConfCb(tum_ardrone::StateestimationParamsConfig &config, uint32_t level)
{
	if(!filter->allSyncLocked && config.PTAMSyncLock)
		ROS_WARN("Ptam Sync has been disabled. This fixes scale etc.");

	if(!ptamWrapper->mapLocked && config.PTAMMapLock)
		ROS_WARN("Ptam Map has been locked.");


	filter->useControl =config.UseControlGains;
	filter->usePTAM =config.UsePTAM;
	filter->useNavdata =config.UseNavdata;

	filter->useScalingFixpoint = config.RescaleFixOrigin;

	ptamWrapper->maxKF = config.PTAMMaxKF;
	ptamWrapper->mapLocked = config.PTAMMapLock;
	filter->allSyncLocked = config.PTAMSyncLock;

	//TODO: Uncomment
	//ptamWrapper->setPTAMPars(config.PTAMMinKFTimeDiff, config.PTAMMinKFWiggleDist, config.PTAMMinKFDist);


	filter->c1 = config.c1;
	filter->c2 = config.c2;
	filter->c3 = config.c3;
	filter->c4 = config.c4;
	filter->c5 = config.c5;
	filter->c6 = config.c6;
	filter->c7 = config.c7;
	filter->c8 = config.c8;

}
