#include <ros/ros.h> 
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

void robot_odom(const nav_msgs::Odometry msg);

// variable declaration
ros::Publisher robot_vel_pub,errors_pub,desired_traj_pub; // node publishers
tf::Point Odom_pos;    //odometry position (x,y)
double Odom_yaw;    //odometry orientation (yaw)

geometry_msgs::Pose2D qd,robot_pose,err;
geometry_msgs::Twist vel_msg;

double vel_desired,angular_desired,vel_robot,angular_robot,u1,u2,saturation_sigma,t,ts;
double a = 0.5,cc = 100.0;
double k = 5;
double h = 0.1;
double xd,yd,xdd,ydd;
double xh,yh,uu1 = 0,uu2 = 0;
double barx1 = 0, barx2 = 0, bary1 = 0, bary2 = 0;
// constraints functions
double g, dg, phix, phiy, hphix, hphiy, epsilonx, epsilony, alphax, alphay, betax, betay;
// double freq=2*M_PI/30;
double freq=M_PI/60;
double max_vel_robot = 0.22; // TuR_barlebot3 maximum linear velocity
double max_angular_robot = 2.8;  // TuR_barlebot3 maximum angular velocity


// function to get robot position and orientation from the odom topic
void robot_odom(const nav_msgs::Odometry msg)
{
    tf::pointMsgToTF(msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.pose.pose.orientation);
	robot_pose.x = msg.pose.pose.position.x;
	robot_pose.y = msg.pose.pose.position.y;

    robot_pose.theta = Odom_yaw;   
}

int main(int argc, char **argv)
{

	// ROS NODE INITIALISATION
    ros::init(argc, argv, "ESO_Node");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/odom", 1000 , robot_odom);
    robot_vel_pub = n.advertise <geometry_msgs::Twist> ("/cmd_vel",1000);
	errors_pub = n.advertise <geometry_msgs::Pose2D> ("/errors_pub",1000);
	desired_traj_pub = n.advertise <geometry_msgs::Pose2D> ("/desired_traj_pub",1000);
	ros::Rate loop_rate(10); // 10Hz

	// All subscribers have to start
	while (errors_pub.getNumSubscribers() == 0 || desired_traj_pub.getNumSubscribers() == 0 
		|| robot_vel_pub.getNumSubscribers() == 0 )
	{
		loop_rate.sleep();
	}

	t = 0.0;
	ts = 0.1;				// Sampling time 

	VectorXd u_ref(2),uf(2),e(3),c(2),Bcc(2),Lcc(2),bar_x(2),bar_y(2),U(2);
	
	MatrixXd P(2,2),
			Acc(2,2);

	bar_x << barx1 , barx2;
	bar_y << bary1 , bary2;

	while(t<=120 && ros::ok())
	{
		
		// calculate the desired trajectory coordinates and velocities
		// Oval Path 

/* 		qd.x = -1.0 + cos(freq*t);
		qd.y = 0.0 + sin(freq*t);
		// xd = -freq*sin(freq*t); yd = freq*cos(freq*t);
		xd = -freq*sin(freq*t); yd = freq*cos(freq*t);
		xdd = -freq*freq*cos(freq*t);ydd = -freq*freq*sin(freq*t);
		qd.theta = atan2(yd,xd);
		vel_desired = (sqrt(pow(xd,2) + pow(yd,2)));
		angular_desired = ( (xd * ydd) - (yd * xdd) )/ (pow(xd,2) + pow(yd,2));
		u_ref << vel_desired,angular_desired; */

		// Lisajous path

		qd.x = 1.1 + 2*sin(freq*t);
		qd.y = 0.9 + 0.7*sin(2*freq*t);
		xd = freq*2*cos(freq*t); 
		yd = 2*freq*0.7*cos(2*freq*t);
		xdd = -freq*freq*2*sin(freq*t);
		ydd = 4*-freq*freq*0.7*sin(2*freq*t);
		qd.theta = atan2(yd,xd);
		vel_desired = (sqrt(pow(xd,2) + pow(yd,2)));
		angular_desired = ( (xd * ydd) - (yd * xdd) )/ (pow(xd,2) + pow(yd,2));
		u_ref << vel_desired,angular_desired;

		desired_traj_pub.publish(qd); // publish the desired trajectory coordinates to the plotter node
		
		// EESO design
		xh = robot_pose.x + h * cos(robot_pose.theta);
		yh = robot_pose.y + h * sin(robot_pose.theta);
		Acc << 0,1,
		       0,0;
		Bcc << 1,0;
		Lcc << 10,0.001;
		bar_x += ts * (Acc * bar_x + Bcc * uu1 + Lcc * (xh - bar_x(0)- qd.x - h * cos(qd.theta)));
		bar_y += ts * (Acc * bar_y + Bcc * uu2 + Lcc * (yh - bar_y(0)- qd.y - h * sin(qd.theta)));
		// controller design
/* 		uu1 = - k * (robot_pose.x-qd.x) - bar_x(1);
		uu2 = - k * (robot_pose.y-qd.y) - bar_y(1); */
		if (t > 0 && t <= 4){
			g = pow((1/4 - 1/t),4) + 0.01;
			dg = -4*pow((1/4 - 1/(t)),3) * (-1/(pow(t,2)));
		}
		else{
			g = 0.01;
			dg = 0;
		}
		phix = a*(g-(robot_pose.x - qd.x))*(g+(robot_pose.x - qd.x));
		phiy = a*(g-(robot_pose.y - qd.y))*(g+(robot_pose.y - qd.y));

		if (phix > cc){
			hphix = 1;
		}
		else{
			hphix = 1 - pow((phix/cc -1 ),4);
		}

		if (phiy > cc){
			hphiy = 1;
		}
		else{
			hphiy = 1 - pow((phiy/cc -1 ),4);
		}

		epsilonx = (robot_pose.x - qd.x)/hphix;
		epsilony = (robot_pose.y - qd.y)/hphiy;

		if (phix > cc){
			alphax = 0;
		}
		else{
			alphax = 1/hphix - 4*a*2/(cc*h*phix*phix)*pow((phix/cc -1),3)*pow((robot_pose.x - qd.x),2);
		}

		if (phiy > cc){
			alphay = 0;
		}
		else{
			alphay = 1/hphiy - 4*a*2/(cc*h*phiy*phiy)*pow((phiy/cc -1),3)*pow((robot_pose.y - qd.y),2);
		}

		if (phix > cc){
			betax = 0;
		}
		else{
			betax = 4*a*2/(cc*h*phix*phix)*pow((phix/cc -1),3)*g*dg*(robot_pose.x - qd.x);
		}

		if (phiy > cc){
			betay = 0;
		}
		else{
			betay = 4*a*2/(cc*h*phiy*phiy)*pow((phiy/cc -1),3)*g*dg*(robot_pose.y - qd.y);
		}

		uu1 = -k*epsilonx/alphax - bar_x(1) - betax/alphax;
		uu2 = -k*epsilony/alphay - bar_y(1) - betay/alphay;

		//uu1 =  -k * (bar_x(0) ) - bar_x(1);
		//uu2 =  -k * (bar_y(0) ) - bar_y(1);

		P << cos(robot_pose.theta), sin(robot_pose.theta),
		     -sin(robot_pose.theta)/h, cos(robot_pose.theta)/h;

		U << uu1 , uu2;

		// calculate Tracking errors between the robot and desired trajectory coordinates
		err.x = (qd.x -robot_pose.x);
		err.y = (qd.y -robot_pose.y);
		err.theta = qd.theta - robot_pose.theta;
		err.theta = atan2(sin(err.theta),cos(err.theta));// Wrap theta in -Pi to Pi 
		errors_pub.publish(err); // publish errors to the plotter node
		uf << vel_desired*cos(err.theta), angular_desired;

		e << err.x , err.y , err.theta;


		c = P * U;
		vel_robot = c(0);
		angular_robot = c(1);

		if (t>=20 && t <= 40)
		{vel_robot += 0.3;
		}
	
		if (t>= 95 && t<= 105)
		{angular_robot += 0.5;
		}
		// Saturation to prevent slipping 
		saturation_sigma = std::max((fabs(vel_robot)/max_vel_robot),std::max((fabs(angular_robot)/max_angular_robot),1.0));

		if (saturation_sigma == (fabs(vel_robot)/max_vel_robot))
		{	
			u1 = std::copysignf(1.0,vel_robot)*max_vel_robot;
			u2 = angular_robot/saturation_sigma;			
		}

		if (saturation_sigma == (fabs(angular_robot)/max_angular_robot))
		{		
			u2 = std::copysignf(1.0,angular_robot)*max_angular_robot;
			u1 = vel_robot/saturation_sigma;
		}

		if (saturation_sigma == 1)
		{			
			u1 = vel_robot;
			u2 = angular_robot;
		}			
		
	
		vel_msg.linear.x = u1;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =u2;
		robot_vel_pub.publish(vel_msg);
		t = t + 0.1;

		ros::spinOnce();
		loop_rate.sleep();	

	}

		vel_msg.linear.x = 0;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =0;
		robot_vel_pub.publish(vel_msg);
    return 0;
}


