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

boost::mutex mutex;
sensor_msgs::LaserScan g_scan;
nav_msgs::Odometry g_odom;
vector<Vec2i> position;

int flag = 0;
Vec2d initPos, curPos;

template<typename T>
inline bool isnan(T value){
	return value != value;
}

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
        // receive a '/odom' message with the mutex
        mutex.lock(); {
                g_odom = msg;
        } mutex.unlock();
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
	const double nImageHalfSize = nImageSize/2;
	const double nAxisSize = nImageSize/16;
	const Vec2d imageCenterCooord = Vec2d(nImageHalfSize, nImageHalfSize);
	display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
	line(display, Point(imageCenterCooord[0], imageCenterCooord[1]),
			Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
	line(display, Point(imageCenterCooord[0], imageCenterCooord[1]),
			Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);

	curPos = imageCenterCooord;
}


void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	// receive a '/odom' message with the mutex
	mutex.lock(); {
		g_scan = msg;
	} mutex.unlock();
}

void
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "opencv_path");
	ros::NodeHandle nh;
	ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);
	ros::Subscriber subOdom = nh.subscribe("/odom", 10, &odomMsgCallback);
	nav_msgs::Odometry odom;
	nav_msgs::Odometry initOdom;	

	Mat display;
	initGrid(display, 801); // OpenCV Display buffer
	// display size is 801 x 801

	sensor_msgs::LaserScan scan; // Scan buffer
	vector<Vec3d> laserScanXY; // LRF scan 정보
	const double dGridMaxDist = 4.5; // Mat distance for grid

	ros::spinOnce();
	mutex.lock(); {
		initOdom = g_odom;
	} mutex.unlock();

	printf("*********** init : %lf %lf\n", initOdom.pose.pose.position.x, initOdom.pose.pose.position.y);

	while(ros::ok()) {
		
		ros::spinOnce();
		// callback 함수 call!
		// receive the global '/scan' message with the mutex
		mutex.lock(); {
			scan = g_scan;
			odom = g_odom;
		} mutex.unlock();

		// scan으로부터 Cartesian X-Y scan 획득
		convertScan2XYZs(scan, laserScanXY);
		// 현재 상황을 draw할 display 이미지를 생성
		initGrid(display, 801);
		drawLRFScan(display, laserScanXY, dGridMaxDist);

		
		// 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
		//transpose(display, display); // X-Y축 교환
		//flip(display, display, 0); // 수평방향 반전
		//flip(display, display, 1); // 수직방향 반전
		Vec2d initPos;

		printf("*********** cur : %lf %lf\n", odom.pose.pose.position.x, odom.pose.pose.position.y);		

		initPos[0] = 400.5 - (odom.pose.pose.position.x - initOdom.pose.pose.position.x) * 801/7;
		initPos[1] = 400.5 - (odom.pose.pose.position.y - initOdom.pose.pose.position.y) * 801/7;
		line(display, Point(initPos[0],initPos[1]), Point(curPos[0],curPos[1]), Scalar(255, 255, 0), 2);
		
		imshow("KNU ROS Lecture >> turtle_kinect_lrf_view", display);
		// image 출력
		// 사용자의 키보드 입력을 받음!
		int nKey = waitKey(30) % 255;
		if(nKey == 27) {
			break;
		}
		// 종료
	}
	return 0;
}



