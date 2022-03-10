#include <ros/ros.h>
#include <std_msgs/String.h>
#include "rubbish/Near.h"
#include "rubbish/Explore.h"
#include <lifter/Lifter.h>
#include <geometry_msgs/Twist.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define STATE_INIT 0
#define STATE_EXPLORE 1
#define STATE_NEAR 2
#define STATE_RETURN 3
#define STATE_GRAB 4
#define STATE_PLACE 5
#define STATE_EMMERGENCY 6

#define MAX_RUBBISH 3

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static string strGoto;
static sound_play::SoundRequest spk_msg;
static ros::Publisher spk_pub;
static ros::Publisher vel_pub;
static string strToSpeak = "";
static string strKeyWord = "";
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static ros::Publisher add_waypoint_pub;

static ros::ServiceClient explore_start;
static ros::ServiceClient explore_stop;
static ros::ServiceClient arm;
static rubbish::Explore srvEpl;
static lifter::Lifter srvArm;
static rubbish::Near srvNear;

static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;

static ros::Subscriber grab_result_sub;
static ros::Subscriber place_result_sub;
static bool bGrabDone = false;
static bool bPlaceDone;

static int nState = STATE_INIT;
static int nDelay = 0;

static int nIndexWant = 0;

// 语音说话
void Speak(string inStr)
{
    spk_msg.arg = inStr;
    spk_msg.volume = 1.0f; //indigo(Ubuntu 14.04)需要注释掉这一句才能编译
    spk_pub.publish(spk_msg);
    ros::spinOnce();
}

// 物品抓取模式开关
static void GrabSwitch(bool inActive)
{
    if (inActive == true)
    {
        behavior_msg.data = "grab start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "grab stop";
        behaviors_pub.publish(behavior_msg);
    }
}

// Placement开关
static void PlaceSwitch(bool inActive)
{
    if (inActive == true)
    {
        behavior_msg.data = "place start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "place stop";
        behaviors_pub.publish(behavior_msg);
    }
}

// 物品抓取状态
void GrabResultCallback(const std_msgs::String::ConstPtr &res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if (nFindIndex >= 0)
    {
        bGrabDone = true;
    }
}

// 物品递给状态
void PlaceResultCallback(const std_msgs::String::ConstPtr &res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if (nFindIndex >= 0)
    {
        bPlaceDone = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_shopping");

    ros::NodeHandle n;

    explore_start = n.serviceClient<rubbish::Explore>("wpb_home_explore/start");
    explore_stop = n.serviceClient<rubbish::Explore>("wpb_home_explore/stop");
    arm = n.serviceClient<lifter::Lifter>("/wpb_home_arm");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    add_waypoint_pub = n.advertise<waterplus_map_tools::Waypoint>("/waterplus/add_waypoint", 1);
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    spk_msg.sound = sound_play::SoundRequest::SAY;
    spk_msg.command = sound_play::SoundRequest::PLAY_ONCE;
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 30);
    grab_result_sub = n.subscribe<std_msgs::String>("/wpb_home/grab_result", 30, &GrabResultCallback);
    place_result_sub = n.subscribe<std_msgs::String>("/wpb_home/place_result", 30, &PlaceResultCallback);
    srvArm.request.height = 0.2;
    srvArm.request.gap = 0.1;

    ROS_WARN("[main] wpb_home_rubbish");
    ros::Rate r(30);
    while (ros::ok())
    {
        // 1、Initiallization
        if (nState == STATE_INIT)
        {
            nDelay++;
            if (nDelay > 1000)
            {
                nDelay = 0;
                nState = STATE_EXPLORE;
            }
        }

        // 2、Explore to find the rubbish
        if (nState == STATE_EXPLORE)
        {
            ROS_INFO("[STATE_EXPLORE]");
            if (nDelay == 0)
            {
                if (!explore_start.call(srvEpl))
                {
                    ROS_WARN("[CActionManager] - explore start failed...");
                }
                else
                {
                    if (srvEpl.response.result){
                        nDelay = 0;
                        nState = STATE_NEAR;}
                    else
                    {
                        nDelay = 0;
                        nState = STATE_EMMERGENCY;}
                }
            }
        }

        // 4、Get close to the rubbish
        if (nState == STATE_NEAR)
        {
            if (nDelay == 0)
            {
                ROS_INFO("[STATE_NEAR]");
                int near_switch = 1;
                ros::param::set("/wpb_tutorial_near", near_switch);
                while (near_switch == 1)
                {
                    ros::param::get("/wpb_tutorial_near", near_switch);
                    sleep(0.5);
                }
                if (!near_switch)
                    nState = STATE_EXPLORE;
                else
                    nState = STATE_GRAB;
                nDelay = 0;
                nDelay++;
            }
        }

        // 5、抓取物品
        if (nState == STATE_GRAB)
        {

            ROS_INFO("[STATE_GRAB]");
            srvArm.request.state = 1;
            if (!arm.call(srvArm))
            {
                ROS_WARN("[CActionManager] - explore start failed...");
            }
            else
            {
                if (srvArm.response.result)
                    nState = STATE_RETURN;
                else
                    nState = STATE_EMMERGENCY;
            }
            nDelay++;
        }

        // 6、抓完物品回来
        if (nState == STATE_RETURN)
        {
            ROS_INFO("[STATE_RETURN]");

            move_base_msgs::MoveBaseGoal goal;

            MoveBaseClient ac("move_base", true);
            if (!ac.waitForServer(ros::Duration(5.0)))
                ROS_INFO("The move_base action server is no running. action abort...");
            else
            {
                srvName.request.name = "final";
                if (cliGetWPName.call(srvName))
                {
                    float x = srvName.response.pose.position.x;
                    float y = srvName.response.pose.position.y;
                    ROS_INFO("Go to final place.");
                }
                else
                {
                    ROS_ERROR("Failed to call service get_wp_name");
                }

                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = srvName.response.pose;
                ac.sendGoal(goal);
                ac.waitForResult();
                if (!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
                {
                    ROS_ERROR("Failed to get to %s ...", strGoto.c_str());
                    nState = STATE_EMMERGENCY;
                }
                else
                {
                    ROS_INFO("Arrived at the final basket.");
                    nDelay = 0;
                    nState = STATE_PLACE;
                }
            }
        }

        // 7、将物品
        if (nState == STATE_PLACE)
        {
            
            ROS_INFO("[STATE_PLACE]");

            srvArm.request.state = 0;
            if (!arm.call(srvArm))
            {
                ROS_WARN("[CActionManager] - explore start failed...");
            }
            else
            {
                if (nIndexWant < MAX_RUBBISH)
                {
                    nIndexWant++;
                    ROS_INFO("OK. Finding the %d rubbish", nIndexWant);
                    nDelay = 0;
                    nState = STATE_EXPLORE;
                }
                else
                {
                    ROS_INFO("All is done!");
                    sleep(10);
                }
            }
        }
        if (nState == STATE_EMMERGENCY)
        {
            // ROS_WARN("State_EMERGENCY");
            if (nDelay == 0)
            {
                ROS_WARN("State_EMERGENCY: end?");
                nDelay++;
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}