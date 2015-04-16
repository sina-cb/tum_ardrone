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
    watchdog_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("/watchdog"), 1);
    watchdog_sub = nh_.subscribe(nh_.resolveName("/watchdog"), 10, &WatchDogThread::sendLand, this);
    land_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"), 1);

    navdata_sub = nh_.subscribe(nh_.resolveName("ardrone/navdata"), 1, &WatchDogThread::navdata_Cb, this);
    vel_sub = nh_.subscribe(nh_.resolveName("cmd_vel"), 1, &WatchDogThread::velCb, this);
    received = false;

    wait_time = 1.0;
    stop_sending = 0;
}

WatchDogThread::~WatchDogThread()
{

}

void WatchDogThread::sendLand(const std_msgs::Empty msg){
    if (stop_sending < 3){
        ROS_ERROR_ONCE("Send Land");
        {
            land_pub.publish(std_msgs::Empty());
            watchdog_pub.publish(std_msgs::Empty());
            stop_sending++;
        }
    }
}

void WatchDogThread::start(){
    ROS_ERROR("Started Watch Dog!");

    last = ros::Time::now();

    while(nh_.ok()){
        now = ros::Time::now();

        if (!now.isZero() && !last.isZero()){
            if (now - last > ros::Duration(wait_time) && received){
                if ((drone_state == 3 || drone_state == 4 || drone_state == 6 || drone_state == 7) &&
                        drone_state != 2){
                    // Clear commands and then send a land command
                    {
                        ros::Publisher tum_ardrone_pub = nh_.advertise<std_msgs::String>(nh_.resolveName("tum_ardrone/com"),50);
                        std_msgs::String s;
                        s.data = "c clearCommands";
                        tum_ardrone_pub.publish(s);
                    }

                    sendLand(std_msgs::Empty());
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
