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
#include <iostream>

using namespace std;

pthread_mutex_t EstimationNode::logIMU_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EstimationNode::logPTAM_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EstimationNode::logFilter_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EstimationNode::logPTAMRaw_CS = PTHREAD_MUTEX_INITIALIZER;

EstimationNode::EstimationNode()
{
    navdata_channel = nh_.resolveName("ardrone/navdata");
    control_channel = nh_.resolveName("cmd_vel");
    output_channel = nh_.resolveName("ardrone/predictedPose");
    video_channel = nh_.resolveName("ardrone/image_raw");
    command_channel = nh_.resolveName("tum_ardrone/com");
	packagePath = ros::package::getPath("tum_ardrone");

	std::string val;
	float valFloat = 0;

	predTime = ros::Duration(25*0.001);

	ros::param::get("~publishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 30;
	publishFreq = valFloat;
	cout << "set publishFreq to " << valFloat << "Hz"<< endl;

	ros::param::get("~calibFile", calibFile);
	if(calibFile.size()>0)
		cout << "set calibFile to " << calibFile << endl;
	else
		cout << "set calibFile to DEFAULT" << endl;


	navdata_sub       = nh_.subscribe(navdata_channel, 10, &EstimationNode::navdataCb, this);
	vel_sub          = nh_.subscribe(control_channel,10, &EstimationNode::velCb, this);
	vid_sub          = nh_.subscribe(video_channel,10, &EstimationNode::vidCb, this);

	dronepose_pub	   = nh_.advertise<tum_ardrone::filter_state>(output_channel,1);

	tum_ardrone_pub	   = nh_.advertise<std_msgs::String>(command_channel,50);
	tum_ardrone_sub	   = nh_.subscribe(command_channel,50, &EstimationNode::comCb, this);

	//tf_broadcaster();

	// other internal vars
	logfileIMU = logfilePTAM = logfileFilter = logfilePTAMRaw = 0;
	currentLogID = 0;
	lastDroneTS = 0;
	lastRosTS = 0;
	droneRosTSOffset = 0;
	lastNavStamp = ros::Time(0);
	filter = new DroneKalmanFilter(this);
	ptamWrapper = new PTAMWrapper(filter, this);
	mapView = new MapView(filter, ptamWrapper, this);
	arDroneVersion = 0;
	//memset(&lastNavdataReceived,0,sizeof(ardrone_autonomy::Navdata));
}

EstimationNode::~EstimationNode(){

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

	ptamWrapper->setPTAMPars(config.PTAMMinKFTimeDiff, config.PTAMMinKFWiggleDist, config.PTAMMinKFDist);

	filter->c1 = config.c1;
	filter->c2 = config.c2;
	filter->c3 = config.c3;
	filter->c4 = config.c4;
	filter->c5 = config.c5;
	filter->c6 = config.c6;
	filter->c7 = config.c7;
	filter->c8 = config.c8;

}

void EstimationNode::Loop(){
	ros::Rate pub_rate(publishFreq);

	ros::Time lastInfoSent = ros::Time::now();

	while (nh_.ok()){
		// -------------- 1. put nav & control in internal queues. ---------------
		ros::spinOnce();

		// -------------- 3. get predicted pose and publish! ---------------
		// get filter state msg

		//TODO: Uncomment
		/*pthread_mutex_lock( &filter->filter_CS );
		tum_ardrone::filter_state s = filter->getPoseAt(ros::Time().now() + predTime);
		pthread_mutex_unlock( &filter->filter_CS );

		// fill metadata
		s.header.stamp = ros::Time().now();
		s.scale = filter->getCurrentScales()[0];
		s.scaleAccuracy = filter->getScaleAccuracy();
		s.ptamState = ptamWrapper->PTAMStatus;
		s.droneState = lastNavdataReceived.state;
		s.batteryPercent = lastNavdataReceived.batteryPercent;

		// publish!
		dronepose_pub.publish(s);

		// --------- if need be: add fake PTAM obs --------
		// if PTAM updates hang (no video or e.g. init), filter is never permanently rolled forward -> queues get too big.
		// dont allow this to happen by faking a ptam observation if queue gets too big (500ms = 100 observations)
		if((getMS(ros::Time().now()) - filter->predictdUpToTimestamp) > 500)
			filter->addFakePTAMObservation(getMS(ros::Time().now()) - 300);


		// ---------- maybe send new info --------------------------
		if((ros::Time::now() - lastInfoSent) > ros::Duration(0.4))
		{
			reSendInfo();
			lastInfoSent = ros::Time::now();
		}
		 */
		// -------------- 4. sleep until rate is hit. ---------------
		pub_rate.sleep();
	}
}

void EstimationNode::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{

}

void EstimationNode::velCb(const geometry_msgs::TwistConstPtr velPtr)
{

}

void EstimationNode::comCb(const std_msgs::StringConstPtr str){

}

void EstimationNode::vidCb(const sensor_msgs::ImageConstPtr img)
{

}

