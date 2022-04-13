#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <rubbish/Lifter.h>

#define state_grip 1
#define state_throw 0

ros::Publisher mani_ctrl_pub;
ros::Publisher vel_pub;
ros::ServiceServer service;

void move(double vel, double duration)
{
	geometry_msgs::Twist vel_cmd;
	double begin, end;

	vel_cmd.linear.x = vel;
	vel_cmd.linear.y = 0;
	vel_cmd.linear.z = 0;
	vel_cmd.angular.x = 0;
	vel_cmd.angular.y = 0;
	vel_cmd.angular.z = 0;
	vel_pub.publish(vel_cmd);

	vel_cmd.linear.x = 0;
	begin=ros::Time::now().toSec();
    end=ros::Time::now().toSec();

	while ((end-begin)<duration)
    {
        sleep(0.01);
		end=ros::Time::now().toSec();
    }
	vel_pub.publish(vel_cmd);
	sleep(1);

	return;

}

bool grip(rubbish::Lifter::Request &req, rubbish::Lifter::Response &res)
{
	static sensor_msgs::JointState ctrl_msg;
	

    ctrl_msg.name.resize(2);
    ctrl_msg.position.resize(2);
    ctrl_msg.velocity.resize(2);
    ctrl_msg.name[0] = "lift";
    ctrl_msg.name[1] = "gripper";
	ctrl_msg.velocity[0] = 0.5;     //lift velocity       meter/second
	ctrl_msg.velocity[1] = 5;       //grip velocity       rad/second
	

	switch(req.state)
	{
		case 1:
		{
			
    		ROS_INFO("[lifter_grip] height:%f , gap: %f",float(0),float(req.gap));
			//go backward and unfold the lifter
			// move(-0.2,3);
			ctrl_msg.position[0] = 0;     //height   /meter
    		ctrl_msg.position[1] = req.gap;     //gap      /meter
    		mani_ctrl_pub.publish(ctrl_msg);
    		// sleep(10);
            
			//go forward and grep
			// // move(0.2,3);
			// ctrl_msg.position[1] = req.gap;
    		// mani_ctrl_pub.publish(ctrl_msg);
    		sleep(2);

			//lift
			// ctrl_msg.position[0] = 0;
			// mani_ctrl_pub.publish(ctrl_msg);
			sleep(3);


			res.result=true;
    		break;
		}
		case 0:
		{
    		ROS_INFO("[lifter_throw] height:%f , gap: %f",float(0),float(req.gap+0.1));
    		ctrl_msg.position[0] = 0; 
			ctrl_msg.position[1] = req.gap+0.05;     //gap      /meter
    		mani_ctrl_pub.publish(ctrl_msg);
    		sleep(2);
    		
    		// ctrl_msg.position[0] = 0;
    		// ctrl_msg.position[1] = req.gap+0.1;
    		// mani_ctrl_pub.publish(ctrl_msg);7
    		sleep(4);
			res.result=true;
    		break;
		}
	}
	
	return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lifter");
    ROS_INFO("service started");

	ros::NodeHandle n;
	mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	service = n.advertiseService("/wpb_home_arm", grip);

    ros::spin();
    return 0;
}

