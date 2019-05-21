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

#include<opencv2/core.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/core/matx.hpp>
#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/LaserScan.h>
#include<iomanip>
#include<vector>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

using namespace std;
using namespace cv;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Global variable
boost::mutex mutex[2];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;

float pre_dAngleTurned;
double x[20], y[20];	// robot path


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** callback function **/
void odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();

	// print g_odom here!
	//std::cout << "odom_info: [position: " << g_odom.pose.pose.position.x << ", " 
	//		<< g_odom.pose.pose.position.y << ", " << g_odom.pose.pose.position.z << "]\n";

	//std::cout << "           [orientation: " << g_odom.pose.pose.orientation.x << ", " 
	//					<< g_odom.pose.pose.orientation.y << ", " 
	//					<< g_odom.pose.pose.orientation.z << ", " 
	//					<< g_odom.pose.pose.orientation.w << "]\n\n";

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Return current transformation matrix from odom **/
tf::Transform getCurrentTransformation(void)
{
	tf::Transform transformation;
	nav_msgs::Odometry odom;

	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();

	// Save position
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	// Save rotation angle
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Save position when robot stopped(initial state) **/
tf::Transform getInitialTransformation(void)
{
	tf::Transform transformation;
	ros::Rate loopRate(1000.0);

	while(ros::ok()) {
		ros::spinOnce();

		transformation = getCurrentTransformation();

		// Break if you got message
		if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) 
		{
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
		if( fabs(dAngleTurned) > fabs(dRotation) || (dRotation == 0)) 
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

/////////////////////////********lidar scan*********///////////////////////////////

template<typename T>
inline bool isnan(T value){
	return value != value;
}

	void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	XYZs.clear();
	XYZs.resize(nRangeSize);
	for(int i=0; i<nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];
		if(isnan(dRange)) {
			XYZs[i] = Vec3d(0., 0., 0.);
		} else {
			double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
			XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
		}
	}
}


void
initGrid(Mat &display, int nImageSize)
{
	const int nImageHalfSize = nImageSize/2;
	const int nAxisSize = nImageSize/16;
	const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
	display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
	line(display, Point(imageCenterCooord[0], imageCenterCooord[1]),
			Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
	line(display, Point(imageCenterCooord[0], imageCenterCooord[1]),
			Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}


void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	// receive a '/odom' message with the mutex
	mutex[1].lock(); {
		g_scan = msg;
	} mutex[1].unlock();
}

void
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
	int nRangeSize = (int)laserScanXY.size();
	for(int i=0; i<nRangeSize; i++) {
		int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);
		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows)
		{
			display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
		}
	}
}
////////////////////

void
displayRoute(Mat &display) {
	sensor_msgs::LaserScan scan;
	vector<Vec3d> laserScanXY;
	const double dGridMaxDist = 4.5;


	ros::spinOnce();
	// callback 함수 call!
	// receive the global '/scan' message with the mutex
	mutex[1].lock(); {
		scan = g_scan;
	} mutex[1].unlock();
	// scan으로부터 Cartesian X-Y scan 획득
	convertScan2XYZs(scan, laserScanXY);
	// 현재 상황을 draw할 display 이미지를 생성
	initGrid(display, 801);
	drawLRFScan(display, laserScanXY, dGridMaxDist);
	// 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
	//transpose(display, display); // X-Y축 교환
	//flip(display, display, 0); // 수평방향 반전
	//flip(display, display, 1); // 수직방향 반전
	imshow("KNU ROS Lecture >> turtle_kinect_lrf_view", display);
	// image 출력
	

	// 사용자의 키보드 입력을 받음!
//	int nKey = waitKey(30) % 255;
//	if(nKey == 27) {
//		break;
//	}
	// 종료
}

/////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	/***auto move***/
	ros::init(argc, argv, "turtle_auto_move");
	ros::NodeHandle nhp, nhs, nh;
	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	/***lidar***/
	ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);
	Mat display;
	initGrid(display, 801);//openCV display buffer

	/* 1. Read robot path from txt */
	char txtPath[100] = "/home/sunny/catkin_ws/src/knu_ros_lecture/src/input.txt";
	FILE *fp = fopen(txtPath, "r");
	int num;
	fscanf(fp, "%d", &num);
	for(int i = 0; i < num; i++){
		fscanf(fp, "%lf %lf", &x[i], &y[i]);
		printf("%d: (%.2lf , %.2lf)\n", (i+1), x[i],y[i]);
	}

	fclose(fp);

	double angle_vel = 0.75;
	double linear_vel = 0.25;
	double prev_angle = 0;

	
	//move 10 times
	for(int i = 0; i < num; i++){
	
		/* 2. Get initial position */
		tf::Transform nextTransformation = getInitialTransformation();

		printf("\n## start point: (%.2lf, %.2lf)\n", nextTransformation.getOrigin().getX(),nextTransformation.getOrigin().getY());
		printf("\n## goal point: (%.2lf, %.2lf)\n", x[i], y[i]);

		// 2-1. Sleep for 1 sec
		ros::Duration(1, 0).sleep();

		// 2-3. Rotate robot.
		double angle = atan2(y[i]-nextTransformation.getOrigin().getY(), x[i]-nextTransformation.getOrigin().getX());

		printf("## current angle: %lf\n", toDegree(angle));


		doRotation(pub, nextTransformation, angle-prev_angle, angle_vel);
		printf("## command angle: %lf\n", toDegree(angle-prev_angle));

		// 2-4. Move robot forward.
		double distance = sqrt((pow(x[i]-nextTransformation.getOrigin().getX(), 2.0)) + (pow(y[i]-nextTransformation.getOrigin().getY(), 2.0)));
		doTranslation(pub, nextTransformation, distance, linear_vel);
		prev_angle = angle;


		/* 3. lidar scan  */
		displayRoute(display);

	}
	return 0;
}
