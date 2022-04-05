/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     LinZhanhui
 ********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include "rubbish/Near.h"
// #include <wpb_home_apps/action_manager.h>
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <geometry_msgs/Twist.h>
#include <wpb_home_behaviors/Coord.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
// static CActionManager action_manager;
static ros::ServiceClient cliGetWPIndex;
static ros::ServiceClient cliGetNum;
static waterplus_map_tools::GetWaypointByName srvName;
static waterplus_map_tools::GetWaypointByIndex srvI;
static float speedRate = 1.0;
static float human_x = 0;
static float human_y = 0;
static float human_z = 0;
static float max_linear_vel = 0.5;
static float max_angular_vel = 1.5;
static float keep_dist = 0.4; //到物体所在平面的垂直距离
// static int exist_rubbish=1;
// #define ROOM_SIZE 5
static int ROOM_SIZE;
static bool bActive = false;
static int naerSuccess = 0;
static int exist_rubbish=1;
static int successDelay = 0;

static int room_index = 0;
static ros::Publisher vel_pub;
static vector<string> arWaypoint;

string strGoto;
bool near_start(rubbish::Near::Request &req, rubbish::Near::Response &res)
{
    naerSuccess = 0;
    speedRate = (float)req.thredhold;
    ROS_INFO("Near_start");
    bActive = true;
    while (naerSuccess == 0)
    {
    }
    if (naerSuccess == 0)
        ROS_ERROR("naerSuccess == 0");
    assert(naerSuccess != 0);
    res.result = naerSuccess;
    return true;
}

bool near_stop()
{
    ROS_WARN("[Near stop]");
    geometry_msgs::Twist vel_cmd;
    bActive = false;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);
    return true;
}

void GotoRubbish(const wpb_home_behaviors::Coord::ConstPtr &msg)
{
    ros::param::get("/wpb_tutorial_near", naerSuccess);
    if (naerSuccess != 1)
        return;
    else
    {
        ROS_INFO("Get close to rubbish..");

        human_x = (msg->x)[0];
        human_y = (msg->y)[0];
        human_z = (msg->y)[0];
        float d_angle = (msg->probability)[0];
        
        geometry_msgs::Twist vel_cmd;

        // float flw_dist = sqrt(human_x * human_x + human_y * human_y);
        float flw_dist = human_z;
        // flw_dist = sqrt(human_z * human_z - flw_dist * flw_dist);
        float diff_dist = flw_dist - keep_dist;
        ROS_INFO("the diatance is %.2f",diff_dist);
        float flw_linear = diff_dist;
        if (fabs(flw_linear) > 0.05 * speedRate)
        {
            flw_linear *=0.15;
            if(fabs(flw_linear)<0.25*0.15)
            {
                if(flw_linear>0)
                {
                    flw_linear=0.25*0.15;
                }
                else
                {
                    flw_linear=-1*0.25*0.15;
                }
            }
            vel_cmd.linear.x = flw_linear;
            if (vel_cmd.linear.x > max_linear_vel)
                vel_cmd.linear.x = max_linear_vel;
            if (vel_cmd.linear.x < -max_linear_vel)
                vel_cmd.linear.x = -max_linear_vel;
            if (vel_cmd.linear.x < 0)
                vel_cmd.linear.x *= 0.3;
        }
        else
        {
            vel_cmd.linear.x = 0;
        }
        // ROS_INFO("human_x:%f, human_y:%f, diff_dist:%f, d_angle:%f", human_x, human_y, diff_dist, d_angle);
        float flw_turn = d_angle * 0.5;
        if (fabs(flw_turn) > 0.05)
        {
            vel_cmd.angular.z = flw_turn;
            if (vel_cmd.angular.z > max_angular_vel)
                vel_cmd.angular.z = max_angular_vel;
            if (vel_cmd.angular.z < -max_angular_vel)
                vel_cmd.angular.z = -max_angular_vel;
        }
        else
        {
            vel_cmd.angular.z = 0;
        }
        vel_cmd.angular.z *= speedRate;
        vel_cmd.linear.x *= speedRate;
        if ((fabs(diff_dist)<0.05) && (fabs(d_angle) < 0.1))
        {
            if (successDelay > 50)
            {
                successDelay = 0;
                naerSuccess = 2;
                near_stop();
                ros::param::set("/wpb_tutorial_near", naerSuccess);
            }
            else
                {
                successDelay += 10;
                }
            ROS_INFO("successDelay=%d", successDelay);
        }
        else
        {
            successDelay = max(successDelay - 10, 0);
        }
        exist_rubbish=0;
        ros::param::get("/rubbish_exist",exist_rubbish);
        ROS_INFO(".....%d......",exist_rubbish);
        if (exist_rubbish==0) // near false
        {
            ROS_INFO("not found ............");
            naerSuccess = 0;
            near_stop();
            ros::param::set("/wpb_tutorial_near", naerSuccess);
        }
        vel_pub.publish(vel_cmd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_near");
    // action_manager.Init();
    ros::NodeHandle n;
    
    ros::Subscriber pose_sub = n.subscribe<wpb_home_behaviors::Coord>("/rubbishes", 1, GotoRubbish);
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    ros::Rate r(10);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();

    return 0;
}