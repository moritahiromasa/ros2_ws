#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc.hpp>


using namespace std;
using namespace cv;

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("rectangletest");

	string input_file = "/home/morita/ros2_ws/output/canny.jpg";
	string output_file = "/home/morita/ros2_ws/output/rectangle.jpg";
	
	Mat img = imread(input_file, IMREAD_COLOR); 
	
	int size = int(img.cols / 4);
	int x = img.cols / 8;
	int y = img.rows / 3;
	
	// 長方形を描画
	rectangle(img, Point(x, y), Point(x + size, y + size), Scalar(0,0,255), 3);
	
	imwrite(output_file, img);

	return 0;
}
