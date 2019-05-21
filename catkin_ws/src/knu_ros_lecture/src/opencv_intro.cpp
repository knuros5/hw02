#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;
int main(int argc, char **argv)
{
	// Ros initialization
	ros::init(argc, argv, "opencv_intro");
	// Initialize a buffer image
	Mat display = Mat::zeros(480, 640, CV_8UC3);
	// Draw a line
	line(display, Point(10, 10), Point(30, 75), CV_RGB(255, 0, 0), 2, CV_AA);
	// Draw a Rectangle
	rectangle(display, Rect(100, 100, 40, 80), CV_RGB(0, 255, 0), 4, CV_AA);
	// Draw a circle
	circle(display, Point(200, 300), 100, CV_RGB(0, 0, 255), 3, CV_AA);
	// Draw a text
	putText(display, "Hello ROS!", Point(400, 200), FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255, 0, 255));
	// Change a pixel value
	for(int r=0; r<50; r++) {
		for(int c=0; c<70; c++) {
			display.at<Vec3b>(100+r, 200+c) = Vec3b(255, 255, 0);
		}
	}
	// B, G, R
	// Counter
	int nCount = 0;
	// main loop
	while(ros::ok()) {
		// draw the images
		imshow("ROS-OpenCV Preview", display);
		// wait a user message
		int nKey = waitKey(30) % 255;
		if(nKey == 27) {
			// escape
			break;
		} else if(nKey == ' ') {
			printf("Count = %d\n", nCount++);
		} else {
		}
	}
	return 0;
}
