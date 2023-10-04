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

void demosaicing(Mat img){
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("robotvision_2023");
	
	string input = "/home/hiromasa/ros2_ws/image/img13.png";
	string output = "/home/hiromasa/ros2_ws/image/output/robotvision_2023/demosaicing.png";
	
	Mat img = imread(input, 3); // カラーで読み込み
	if(img.empty())
		cerr << "ERROR:Imge file not found." << endl;

	demosaicing(img);

	return 0;
}
