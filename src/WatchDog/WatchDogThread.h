#pragma once
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
#ifndef __WATCHDOG_THREAD_H
#define __WATCHDOG_THREAD_H

#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "cvd/thread.h"

class WatchDogThread
{
public:
    WatchDogThread();
    ~WatchDogThread();

    void velCb(const geometry_msgs::TwistConstPtr vel);
    void navdata_Cb(const ardrone_autonomy::Navdata &navdata_msg_in);

    void start();

    void sendLand(const std_msgs::Empty msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber vel_sub;
    ros::Subscriber navdata_sub;

    ros::Subscriber watchdog_sub;
    ros::Publisher watchdog_pub;
    ros::Publisher land_pub;

    ros::Time now;
    ros::Time last;

    int drone_state;
    bool received;
    double wait_time;
    int stop_sending;

    pthread_mutex_t send_CS;
};

#endif /* __WATCHDOG_THREAD_H */
