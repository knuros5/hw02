/*
 * Date: 2019-05-15
 * Title: opencv_path.cpp
 */

#include<ros/ros.h>
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
#include<boost/thread/mutex.hpp>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<vector>

using namespace std;
using namespace cv;

#define MAP_SIZE 800

boost::mutex mutex[2];
sensor_msgs::LaserScan g_scan;
nav_msgs::Odometry g_odom;

template<typename T>
inline bool isnan(T value){
	return value != value;
}


////////////////////////////////////////////////////
//********* callback function ********//

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}

	void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	// receive a '/scan' message with the mutex
	mutex[1].lock(); {
		g_scan = msg;
	} mutex[1].unlock();
}

////////////////////////////////////////////////

// odom으로부터 받은 좌표를 평면좌표로 변환하는 function
void convertOdom2XYZ(nav_msgs::Odometry &odom, Vec3d &curPos, Vec3d &curRot)
{
	// 이동 저장
	curPos[0] = odom.pose.pose.position.x;
	curPos[1] = odom.pose.pose.position.y;
	curPos[2] = odom.pose.pose.position.z;

	// 회전 저장
	tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3(rotationQuat).getEulerYPR(curRot[2], curRot[1], curRot[0]);

}

// scan으로부터 받은 좌표를 평면좌표로 변환하는 function
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

//3차원 point인 laserScanXY를 world coordinate로 변환
void transformScanXY(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
	Vec3d newPoint;
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	int nRangeSize = (int)laserScanXY.size();

	for(int i = 0 ; i < nRangeSize ; i++) {
		newPoint[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
		newPoint[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
		newPoint[2];
		laserScanXY[i] = newPoint;
	}
}

////////////////////////////////////////////////

// opencv display 초기화
void initGrid(Mat &display, Vec3d &curPos, int nImageSize)
{
	const double nImageHalfSize = nImageSize/2; // 800/2 == 400
	const double nAxisSize = nImageSize/16; // 800/16 == 50
	const Vec2d imageCenterCooord = Vec2d(nImageHalfSize, nImageHalfSize); //center of display == center of world

	display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
	//x축
//	line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
	line(display, Point(curPos[0], curPos[1]), Point(curPos[0]+nAxisSize, curPos[1]), Scalar(0, 0, 255), 10);
	//y축	
	line(display, Point(curPos[0], curPos[1]), Point(curPos[0], curPos[1]+nAxisSize), Scalar(0, 255, 0), 10);

}

//장애물 scan 후 blue point를 찍어주는 fuction
void drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
	int nRangeSize = (int)laserScanXY.size();
	for(int i=0; i<nRangeSize; i++) {
		int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
		int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);
		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
			display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
		}
	}
}

//initial point부터 current position까지 line을 그려주는 function
void drawOdom(Mat &display, Vec3d &curPos, Vec3d &initPos, double dMaxDist)
{
	Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

	// init location
	int initX = imageHalfSize[0] + cvRound((initPos[0]/dMaxDist)*imageHalfSize[0]);
	int initY = imageHalfSize[1] + cvRound((initPos[1]/dMaxDist)*imageHalfSize[1]);

	// current location
	int curX = imageHalfSize[0] + cvRound((curPos[0]/dMaxDist)*imageHalfSize[0]);
	int curY = imageHalfSize[1] + cvRound((curPos[1]/dMaxDist)*imageHalfSize[1]);

	//draw line
	line(display, Point(initX, initY), Point(curX, curY), Scalar(255,0,0), 3);

}

////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "opencv_path");
	ros::NodeHandle nh_scan, nh_odom;
	ros::Subscriber subScan = nh_scan.subscribe("/scan", 10, &scanMsgCallback);
	ros::Subscriber subOdom = nh_odom.subscribe("/odom", 10, &odomMsgCallback);
	bool flag = false;

	// Odometry buffer
	nav_msgs::Odometry odom;

	// Scan buffer
	sensor_msgs::LaserScan scan; 

	// LRF scan 정보
	vector<Vec3d> laserScanXY;
	// Odom location 정보
	Vec3d curPos, initPos, curRot;

	// Mat distance for grid
	const double dGridMaxDist = 4.5;

	// OpenCV Display buffer
	Mat display;
	//initGrid(display, MAP_SIZE);


	/*
	   ros::spinOnce();
	   mutex.lock(); {
	   initOdom = g_odom;
	   } mutex.unlock();

	   printf("*********** init : %lf %lf\n", initOdom.pose.pose.position.x, initOdom.pose.pose.position.y);
	 */

	/*** main loop ***/
	while(ros::ok()) {
		// callback 함수 call!
		ros::spinOnce();

		//--------------------------------
		/* odom */
		mutex[0].lock(); {
			odom = g_odom;
		} mutex[0].unlock();
		// odom으로부터 이동정보 획득
		convertOdom2XYZ(odom, curPos, curRot);
		if(flag == false && curPos[0]!=0 && curPos[1]!=0) {
			initPos[0] = curPos[0];
			initPos[1] = curPos[1];
			flag = true;
			printf("***************** init : %lf , %lf *****************\n", initPos[0], initPos[1]);
		}
		printf("******* cur : %lf , %lf *******\n", curPos[0], curPos[1]);
		/* scan */
		mutex[1].lock(); {
			scan = g_scan;
		} mutex[1].unlock();
		// scan으로부터 Cartesian X-Y scan 획득
		convertScan2XYZs(scan, laserScanXY);
		// convert laserScan into 2D coordinate
		transformScanXY(laserScanXY, curPos[0], curPos[1], curRot[2]);


		//--------------------------------
		// 현재 상황을 draw할 display 이미지를 생성
		initGrid(display, curPos, MAP_SIZE);
		drawLRFScan(display, laserScanXY, dGridMaxDist);
		drawOdom(display, initPos, curPos, dGridMaxDist);
		


		// 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
		//transpose(display, display); // X-Y축 교환
		//flip(display, display, 0); // 수평방향 반전
		//flip(display, display, 1); // 수직방향 반전




		//		printf("*********** curOdom : %lf %lf / curPos : %lf %lf\n", odom.pose.pose.position.x, odom.pose.pose.position.y, curPos[0], curPos[1]);		

		//		initPos[0] = (MAP_SIZE/2) - (odom.pose.pose.position.x - initOdom.pose.pose.position.x) * MAPSIZE/7;
		//		initPos[1] = (MAP_SIZE/2) - (odom.pose.pose.position.y - initOdom.pose.pose.position.y) * MAPSIZE/7;
		//		line(display, Point(initPos[0],initPos[1]), Point(curPos[0],curPos[1]), Scalar(255, 255, 0), 2);


		// image 출력
		imshow("KNU ROS Lecture >> turtle_kinect_lrf_view", display);
		// 사용자의 키보드 입력을 받음!
		int nKey = waitKey(30) % 255;
		if(nKey == 27) {
			break; //종료
		}
	}
	return 0;
}



