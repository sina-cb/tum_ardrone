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

#include "WatchDogThread.h"
#include <unistd.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "stdio.h"
#include "std_msgs/Empty.h"



WatchDogThread::WatchDogThread()
{

    ros::master::V_TopicInfo ti;
    if(ros::master::getTopics(ti)) {
        for(ros::master::V_TopicInfo::iterator it = ti.begin(); it != ti.end(); it++) {
//            if(it->datatype == _topicType)
                ROS_ERROR(it->name.c_str());
        }
    }

    navdata_sub = nh_.subscribe(nh_.resolveName("ardrone/navdata"), 1000, &WatchDogThread::navdata_Cb, this);
    vel_sub = nh_.subscribe(nh_.resolveName("cmd_vel"), 100, &WatchDogThread::velCb, this);
    received = false;

    wait_time = 1.0;
}

WatchDogThread::~WatchDogThread()
{

}

void WatchDogThread::sendLand(){
    ROS_ERROR("Send Land");

    ros::Publisher land_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"), 1);

    pthread_mutex_lock(&send_CS);
    land_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&send_CS);
}

void WatchDogThread::start(){
    ROS_ERROR("Started Watch Dog!");

    last = ros::Time::now();

    while(nh_.ok()){
        now = ros::Time::now();

        if (!now.isZero() && !last.isZero()){
            if (now - last > ros::Duration(wait_time) && received){
                if (drone_state == 3 || drone_state == 4 || drone_state == 6 || drone_state == 7){
                    sendLand();
                }

                if (drone_state == 2){
                    received = false;
                }
            }
        }

        ros::spinOnce();
    }

}

void WatchDogThread::navdata_Cb(const ardrone_autonomy::Navdata &navdata_msg_in){
    drone_state = navdata_msg_in.state;
}

void WatchDogThread::velCb(const geometry_msgs::TwistConstPtr vel){
    last = ros::Time::now();
    received = true;
}
