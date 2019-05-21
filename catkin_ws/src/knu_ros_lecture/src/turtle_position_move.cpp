/*
 * name: Minjeong Kang
 * student id: 2016116545
 * date: 2019/04/14
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <stdio.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex;
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;
double x[20], y[20];	// robot path

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex.lock(); {
		g_odom = msg;
	} mutex.unlock();
	
	// print g_odom here!
	//std::cout << "odom_info: [position: " << g_odom.pose.pose.position.x << ", " 
	//		<< g_odom.pose.pose.position.y << ", " << g_odom.pose.pose.position.z << "]\n";

	//std::cout << "           [orientation: " << g_odom.pose.pose.orientation.x << ", " 
	//					<< g_odom.pose.pose.orientation.y << ", " 
	//					<< g_odom.pose.pose.orientation.z << ", " 
	//					<< g_odom.pose.pose.orientation.w << "]\n\n";

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Return current transformation matrix from odom
tf::Transform 
getCurrentTransformation(void)
{
	tf::Transform transformation;
	nav_msgs::Odometry odom;

	mutex.lock(); {
		odom = g_odom;
	} mutex.unlock();

	// Save position
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, 
					     odom.pose.pose.position.y, 
					     odom.pose.pose.position.z));

	// Save rotation angle
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, 
						  odom.pose.pose.orientation.y, 
						  odom.pose.pose.orientation.z, 
						  odom.pose.pose.orientation.w));

	return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Save position when robot stopped(initial state)
tf::Transform
getInitialTransformation(void)
{
	tf::Transform transformation;
	ros::Rate loopRate(1000.0);

	while(ros::ok()) {
		ros::spinOnce();

		transformation = getCurrentTransformation();

		// Break if you got message
		if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. 
							   && transformation.getOrigin().getZ() != 0.) {
			break;
		} else {
			loopRate.sleep();
		}
	}

	return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Execute rotation
bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	//the command will be to turn at 'rotationSpeed' rad/s
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;

	if(dRotation < 0.) {
		baseCmd.angular.z = -dRotationSpeed;
	} else {
		baseCmd.angular.z = dRotationSpeed;
	}

	// Get odometry messages of current position while moving
	bool bDone = false;
	ros::Rate loopRate(1000.0);



	while(ros::ok() && !bDone) {
		// Get callback messages

		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();

		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
		tf::Quaternion rotationQuat = relativeTransformation.getRotation();



		double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

		// Check termination condition
		if( fabs(dAngleTurned) > fabs(dRotation) || (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)) 
		{
			bDone = true;
			break;
		} else {
			pre_dAngleTurned = dAngleTurned;
			//send the drive command
			pubTeleop.publish(baseCmd);
			//printf("doRotation dAngleTurned : %lf######\n", dAngleTurned);/////
			// sleep!
			loopRate.sleep();
		}
	}

	// Initialization
	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Move
bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
	//the command will be to go forward at 'translationSpeed' m/s
	geometry_msgs::Twist baseCmd;

	if(dTranslation < 0) {
		baseCmd.linear.x = -dTranslationSpeed;
	} else {
		baseCmd.linear.x = dTranslationSpeed;
	}

	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;

	// Get odometry messages of current position while moving
	bool bDone = false;
	ros::Rate loopRate(1000.0);

	while(ros::ok() && !bDone) {
		// Get callback messages
		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();

		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
		double dDistMoved = relativeTransformation.getOrigin().length();

		// Check termination condition
		if(fabs(dDistMoved) >= fabs(dTranslation)) {
			bDone = true;
			break;
		} else {
			//send the drive command
			pubTeleop.publish(baseCmd);

			// sleep!
			loopRate.sleep();
		}
	}

	//  Initialization
	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}



/////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_position_move");
	ros::NodeHandle nhp, nhs;
	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	
	/* 1. Read robot path from txt */
	printf("Let's read txt file.\n");

	char txtPath[100] = "/home/sunny/catkin_ws/src/knu_ros_lecture/src/input_path_2.txt";
	char outPath[100] = "home/sunny/catkin_ws/src/knu_ros_lecture/src/turtle_position_move.txt";
	FILE *fp = fopen(txtPath, "r");

	printf("SUCCESS: file opened.\n");

	int num;
	fscanf(fp, "%d", &num);
	printf("num : %d\n",num);
	for(int i = 0; i < num; i++){
		fscanf(fp, "%lf %lf", &x[i], &y[i]);
		printf("x, y : %lf , %lf\n", x[i],y[i]);
	}

	fclose(fp);

	double angle_vel = 0.75;
	double linear_vel = 0.25;

	
	/* 2. Get initial position */
	tf::Transform nextTransformation = getInitialTransformation();

	printf("############# current location (%.2lf, %.2lf)################\n", nextTransformation.getOrigin().getX(),nextTransformation.getOrigin().getY());

	
	// 2-1. Sleep for 1 sec
	ros::Duration(1, 0).sleep();

	// 2-3. Rotate robot.
	double angle = atan2(y[0]-nextTransformation.getOrigin().getY(), x[0]-nextTransformation.getOrigin().getX());
	doRotation(pub, nextTransformation, angle, angle_vel);
	
	// 2-4. Move robot forward.
	double distance = sqrt((pow(x[0]-nextTransformation.getOrigin().getX(), 2.0)) + (pow(y[0]-nextTransformation.getOrigin().getY(), 2.0)));
	doTranslation(pub, nextTransformation, distance, linear_vel);

	printf("###########################################################\n");

	for(int i = 0; i < num-1; i++){
		nextTransformation = getInitialTransformation();
		printf("############# move to (%.2lf, %.2lf)################\n", x[i+1], y[i+1]);
		printf("############# current (%.2lf, %.2lf)################\n", nextTransformation.getOrigin().getX(), nextTransformation.getOrigin().getY());
		// 2-1. Sleep for 1 sec
		ros::Duration(1, 0).sleep();

		// 2-2. Get current point
		//tf::Transform initialTransformation=getInitialTransformation();
		nextTransformation = getInitialTransformation();
		
		tf::Quaternion currentQ = nextTransformation.getRotation();

		// 2-3. Rotate robot.
		angle = atan2((y[i+1]-nextTransformation.getOrigin().getY()) , (x[i+1]-nextTransformation.getOrigin().getX()));
		printf("### angle ###: %lf\n", angle);
		
		doRotation(pub, nextTransformation, angle, angle_vel);
		
		printf("########FINISHED ROTATION######\n");
		//ros::Duration(1,0a).sleep();

		// 2-4. Move robot forward.
		double distance = sqrt((pow(x[i+1]-x[i], 2.0)) + (pow(y[i+1]-y[i], 2.0)));
		doTranslation(pub, nextTransformation, distance, linear_vel);
		
		printf("###########################################################\n");
	}

	return 0;
}
