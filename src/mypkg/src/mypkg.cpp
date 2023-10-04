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

#define NUM 500
#define XY 2

using namespace std;
using namespace cv;

int height, width;

void create2DbinaryImg(int** binaryImg, Mat thinned_Img){
	for(int y = 0; y < height; y++){
		for(int x = 0; x < width; x++){
			if(thinned_Img.at<uchar>(y, x) <  65 )	// 1(黒), 0(白)
				binaryImg[y][x] = 1;
			else
				binaryImg[y][x] = 0;
		}
	}
}

void print_binaryImg(int** binaryImg){
	for(int y = 0; y < height; y++){
		for(int x = 0; x < width; x++)
			cout << binaryImg[y][x];
		cout << "\n";
	}
	cout << "\n" << "=================" << "\n" << endl;
}

void print_endPoint(int *count, int point[NUM][XY]){
	cout << "############################################" << endl;
	if(*count == 0)
		cout << "Not found end point." << endl;
	else{
		cout << "(x, y)" << endl;
		for(int i = 0; i < *count; i++)
			cout << point[i][1]  << ", " << point[i][0] << endl;
	}
}
void kernel(int x, int y, int p[9], int** binaryImg){
	p[0] = binaryImg[y][x];
	p[1] = binaryImg[y-1][x];
	p[2] = binaryImg[y-1][x+1];
	p[3] = binaryImg[y][x+1];
	p[4] = binaryImg[y+1][x+1];
	p[5] = binaryImg[y+1][x];
	p[6] = binaryImg[y+1][x-1];
	p[7] = binaryImg[y][x-1];
	p[8] = binaryImg[y-1][x-1];
}
/*
void kernels_5x5(int x, int y, int p[25], int** binaryImg){
	p[0] = binaryImg[y][x];	
	p[1] = binaryImg[y-2][x-2];		p[2] = binaryImg[y-2][x-1];
	p[3] = binaryImg[y-2][x];		p[4] = binaryImg[y-2][x+1];
	p[5] = binaryImg[y-2][x+2];		p[6] = binaryImg[y-1][x-2];
	p[7] = binaryImg[y-1][x-1];		p[8] = binaryImg[y-1][x];
	p[9] = binaryImg[y-1][x+1];		p[10] = binaryImg[y-1][x+2];
	p[11] = binaryImg[y][x-2];	p[12] = binaryImg[y][x-1];
	p[13] = binaryImg[y][x+1];	p[14] = binaryImg[y][x+2];
	p[15] = binaryImg[y+1][x-2];	p[16] = binaryImg[y+1][x-1];
	p[17] = binaryImg[y+1][x];	p[18] = binaryImg[y+1][x+1];
	p[19] = binaryImg[y+1][x+2];	p[20] = binaryImg[y+2][x-2];
	p[21] = binaryImg[y+2][x-1];	p[22] = binaryImg[y+2][x];
	p[23] = binaryImg[y+2][x+1];	p[24] = binaryImg[y+2][x+2];
}
*/
void find_endPoint(int *count, int p[9], int end_point[NUM][XY], int** binaryImg){
	for(int y = 1; y < height-1; y++){
		for(int x = 1; x < width-1; x++){
			kernel(x, y, p, binaryImg);
			
			if( (p[0]+p[5] == 0 ) && (p[1]*p[2]*p[3]*p[7]*p[8] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "1" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[7] == 0 ) && (p[1]*p[2]*p[3]*p[4]*p[5] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "2" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[1] == 0 ) && (p[3]*p[4]*p[5]*p[6]*p[7] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "3" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[3] == 0 ) && (p[5]*p[6]*p[7]*p[8]*p[1] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "4" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[6] == 0 ) && (p[1]*p[2]*p[3]*p[4]*p[5]*p[7]*p[8] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "5" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[8] == 0 ) && (p[1]*p[2]*p[3]*p[4]*p[5]*p[6]*p[7] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "6" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[2] == 0 ) && (p[1]*p[3]*p[4]*p[5]*p[6]*p[7]*p[8] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "7" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
			else if( (p[0]+p[4] == 0 ) && (p[1]*p[2]*p[3]*p[5]*p[6]*p[7]*p[8] == 1) ){
				end_point[*count][0] = y;
				end_point[*count][1] = x;
				(*count)++;
/*				cout << "8" << endl;
				cout << p[8] << p[1] << p[2] << endl;
				cout << p[7] << p[0] << p[3] << endl;
				cout << p[6] << p[5] << p[4] << endl;
				cout << "\n";
*/			}
		}
	}
}
void first(int p[9], int* x_pixels, int* y_pixels, int* one_before){
	if(p[1] == 0){
		*one_before = 5;	*y_pixels -= 1;
		return;
	}
	else if(p[3] == 0){
		*one_before = 7;	*x_pixels += 1;
		return;
	}
	else if(p[5] == 0){
		*one_before = 1;	*y_pixels += 1;
		return;
	}
	else if(p[7] == 0){
		*one_before = 3;	*x_pixels -= 1;
		return;
	}
	else if(p[2] == 0){
		*one_before = 6;	*x_pixels += 1;
							*y_pixels -= 1;
		return;
	}
	else if(p[4] == 0){
		*one_before = 8;	*x_pixels += 1;
							*y_pixels += 1;
		return;
	}
	else if(p[6] == 0){
		*one_before = 2;	*x_pixels -= 1;
							*y_pixels += 1;
		return;
	}
	else if(p[8] == 0){
		*one_before = 4;	*x_pixels -= 1;
							*y_pixels -= 1;
		return;
	}
}
void pattern_one(int p[9], int* x_pixels, int* y_pixels, int* one_before, int* two_before){
	if( *one_before == 1){
		if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;
			*two_before = 8;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			*two_before = 2;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		}
	}
	else if( *one_before == 3){
		if(p[1] == 0){
			*one_before = 5;	
			*two_before = 4;	*y_pixels -= 1;
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	
			*two_before = 2;	*y_pixels -= 1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			*two_before = -1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		}
	} 
	if( *one_before == 5){
		if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;
			*two_before = 6;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			*two_before = 4;
			return;
		} 
		else if(p[1] == 0){
			*one_before = 5;	
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		}
	}
	if( *one_before == 7){
		if(p[1] == 0){
			*one_before = 5;	
			*two_before = 6;	*y_pixels -= 1;
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	
			*two_before = 8;	*y_pixels += 1;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;
			*two_before = -1;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
	}
}

void pattern_two(int p[9], int* x_pixels, int* y_pixels, int* one_before, int* two_before){
	*two_before = -1;
	if( *one_before == 2){
		if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
								*y_pixels += 1;
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	*y_pixels += 1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
								*y_pixels += 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
								*y_pixels -= 1;
			return;
		}
	}
	else if( *one_before == 4){
		if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1; 
								*y_pixels += 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
								*y_pixels -= 1;
			return;
		} 
		else if(p[1] == 0){
			*one_before = 5;	*y_pixels -= 1;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
								*y_pixels -= 1;
			return;
		}
	} 
	if( *one_before == 6){
		if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
								*y_pixels -= 1;
			return;
		} 
		else if(p[1] == 0){
			*one_before = 5;	*y_pixels -= 1;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
								*y_pixels -= 1;
			return;
		} 
		else if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
								*y_pixels += 1;
			return;
		}
	}
	if( *one_before == 8){
		if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;	
								*y_pixels -= 1;
			return;
		} 
		else if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
								*y_pixels += 1;
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	*y_pixels += 1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
								*y_pixels += 1;
			return;
		} 
	}
}
void pattern_three(int p[9], int* x_pixels, int* y_pixels, int* one_before, int* two_before){
	if( *one_before == 1 && *two_before == 2){
		if(p[5] == 0){
			*one_before = 1;	
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			*two_before = 2;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
	}
	else if( *one_before == 1 && * two_before == 8){
		if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;	
			*two_before = 8;	
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
	} 
	if( *one_before == 3 && *two_before == 2){
		if(p[5] == 0){
			*one_before = 1;	
			*two_before = 2;	*y_pixels += 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			*two_before = -1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
	}
	if( *one_before == 3 && *two_before == 4){
		if(p[1] == 0){
			*one_before = 5;	
			*two_before = 4;	*y_pixels += 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;
			*two_before = -1;
			return;
		} 
		else if(p[6] == 0){
			*one_before = 2;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
	}
	if( *one_before == 5 && *two_before == 4){
		if(p[1] == 0){
			*one_before = 5;	
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[7] == 0){
			*one_before = 3;	*x_pixels -= 1;	
			*two_before = 4;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;	
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
	}
	else if( *one_before == 5 && * two_before == 6){
		if(p[1] == 0){
			*one_before = 5;	
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;
			*two_before = 8;	
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;	
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[8] == 0){
			*one_before = 4;	*x_pixels -= 1;	
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
	} 
	if( *one_before == 7 && *two_before == 6){
		if(p[1] == 0){
			*one_before = 5;	
			*two_before = 6;	*y_pixels -= 1;
			return;
		} 
		else if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;	
			*two_before = -1;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
	}
	if( *one_before == 7 && *two_before == 8){
		if(p[3] == 0){
			*one_before = 7;	*x_pixels += 1;	
			*two_before = -1;
			return;
		} 
		else if(p[5] == 0){
			*one_before = 1;	
			*two_before = 8;	*y_pixels += 1;
			return;
		} 
		else if(p[2] == 0){
			*one_before = 6;	*x_pixels += 1;
			*two_before = -1;	*y_pixels -= 1;
			return;
		} 
		else if(p[4] == 0){
			*one_before = 8;	*x_pixels += 1;
			*two_before = -1;	*y_pixels += 1;
			return;
		} 
	}
}
void print_coordinate(int x, int y, int z, int one_before_kernel){
	cout << x << "\t" << y << "\t" << z << "\t" << one_before_kernel << endl;
}

void minimum_distance(int* x_pixels, int* y_pixels, int* count, int* min_num, int point[NUM][XY], int* check){
	float min = 5000.0;
	
	for(int i = 0; i < *count; i++){ // 通過した端点の値を-1にする
		if(*x_pixels == point[i][1] && *y_pixels == point[i][0])
			check[i] = -1;
	}
	
	for(int i = 0; i < *count; i++){ // 現在いる端点から最も近いまだ通過してない端点を探す
		if(check[i] != -1 &&  min > sqrt( pow(*x_pixels - point[i][1], 2.0) + pow(*y_pixels - point[i][0], 2.0) ) ){ 
			min = sqrt( pow(*x_pixels - point[i][1], 2.0) + pow(*y_pixels - point[i][0], 2.0) );
			*min_num = i;
		}
	}
	check[*min_num] = -1; // 次の通過する端点を-1
}
void line_color(Mat img, int* x, int* y){
			img.at<Vec3b>(*y, *x)[0] =  0; // B
			img.at<Vec3b>(*y, *x)[1] =  0; // G
			img.at<Vec3b>(*y, *x)[2] =  255; // R
}
void isolate_point(int p[9], int** binaryImg){
	for(int y = 1; y < height-1; y++){
		for(int x = 1; x < width-1; x++){
			kernel(x, y, p, binaryImg);
			if(p[0] == 0 && p[1]*p[2]*p[3]*p[4]*p[5]*p[6]*p[7]*p[8] == 1)
				binaryImg[y][x] = 1;
		}
	}
}

int xy_rotate(int xy){
	return xy + 60;
}

Mat Img_Resize(Mat img){
	// 長方形の画像を正方形にリサイズ
	if(img.cols != img.rows){
		if(img.cols < img.rows)	// タテ長の場合
			resize(img, img, Size(), 94.0/ img.cols, 94.0/ img.cols);
		else if(img.cols < img.rows) // ヨコ長の場合
			resize(img, img, Size(), 94.0/ img.rows, 94.0/ img.rows);
	}

	if(img.cols > 96) 
		resize(img, img, Size(), 94.0/ img.rows, 94.0/ img.rows);

	return img;
}

void route_search(int* lines_count, int* one, int* two, int* count, int p[9], int point[NUM][XY], int** binaryImg){
	FILE* fp = fopen("/home/hiromasa/ros2_ws/CSV/normal_coordinate.csv", "w");
	FILE* gp = fopen("/home/hiromasa/ros2_ws/CSV/correct_coordinate.csv", "w");
	
	string input_file = "/home/hiromasa/ros2_ws/image/img10.png";
	Mat route_img = imread(input_file, IMREAD_COLOR);
	
	// 長方形の画像を正方形にリサイズ
	route_img = Img_Resize(route_img);
	
	string output_file = "/home/hiromasa/ros2_ws/image/output/mypkg/route_0.png";
	route_img.setTo(Scalar(255, 255, 255));
	imwrite(output_file, route_img);
	
	srand(static_cast<unsigned int>(time(nullptr)));

	int n = rand() % (*count); // 最初の端点、スタート位置, または次の端点の位置を決める
	int x_pixels, y_pixels, z_pixels; // 注目画素を格納する変数
	int one_before = *one, two_before = *two;
	int old_x_pixels, old_y_pixels;
	int* check;
	check = new int[*count]; // 動的に配列を確保する, 端点を通過したか確認する配列
	
	if(fp == NULL)
		cerr << "ERROR:test.csv not found." << endl;

	for(int i = 0; i < *count; i++)
		check[i] = i; // 各端点に番号を振る	
	check[n] = -1; // 端点を通過したら-1に変更する

	// 端点から線をたどるとき
	for(int j = 1; j <= (*count) / 2; j++){
		string output_file = "/home/hiromasa/ros2_ws/image/output/mypkg/route_" + to_string(j) + ".png";
		x_pixels = point[n][1];
		y_pixels = point[n][0];
		
		if(j > 0){
			fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, 10); // 端点に辿り着いたら,持ち上げたアームを紙にくっつける
			fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), 10); // 端点に辿り着いたら,持ち上げたアームを紙にくっつける
		}
		
		z_pixels = 0;
		
		fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, z_pixels);
		fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), z_pixels); // 
		
		line_color(route_img, &x_pixels, &y_pixels);
		
		cout << "(x, y, z)" << endl;
		cout << x_pixels << "," << y_pixels  << "," << z_pixels << "\t" << one_before << "," << two_before << endl;
		kernel(x_pixels, y_pixels, p, binaryImg);
		binaryImg[y_pixels][x_pixels] = 1;
		first(p, &x_pixels, &y_pixels, &one_before);
	
		fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, z_pixels);
		fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), z_pixels); // 
		line_color(route_img, &x_pixels, &y_pixels);
		
		for(int k = 0; k < 1000; k++){
			kernel(x_pixels, y_pixels, p, binaryImg);
			binaryImg[y_pixels][x_pixels] = 1;
		
			// 3x3のカーネル内にいくつ線の画素が存在するかを識別する条件
			if((one_before % 2 == 1)  && two_before == -1)
				pattern_one(p, &x_pixels, &y_pixels, &one_before, &two_before);
			else if((one_before % 2 == 0) && two_before == -1)
				pattern_two(p, &x_pixels, &y_pixels, &one_before, &two_before);
			else if(two_before > 0)
				pattern_three(p, &x_pixels, &y_pixels, &one_before, &two_before);
	
			if(old_x_pixels == x_pixels && old_y_pixels == y_pixels){
				*lines_count = j;
				break;
			}
	
		cout << x_pixels << "," << y_pixels  << "," << z_pixels << "\t" << one_before << "," << two_before << endl;
		
		fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, z_pixels);
		fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), z_pixels); // 
		
		line_color(route_img, &x_pixels, &y_pixels);
		old_x_pixels = x_pixels;
		old_y_pixels = y_pixels;
	}
		minimum_distance(&x_pixels, &y_pixels, &(*count), &n, point, check);
	
		fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, 10); // 端点に辿り着いたら一度持ち上げる動作
		fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), 10); // 
		
		imwrite(output_file, route_img);
	}

	// 不要な孤立点を削除する
	isolate_point(p, binaryImg);
//	print_binaryImg(binaryImg); // Print	

	*lines_count += 1;
	// 端点がない線をたどる
	cout << "it will follow no end point line " << endl;
	for(int y = 1; y < height-1; y++){
		for(int x = 1; x < width-1; x++){
			string output_file = "/home/hiromasa/ros2_ws/image/output/mypkg/route_" + to_string(*lines_count) + ".png";
			
			kernel(x, y, p, binaryImg);
			if(p[0] == 0){
				x_pixels = x;
				y_pixels = y;
				z_pixels = 0;
				
				first(p, &x_pixels, &y_pixels, &one_before);
				
				cout << "(x, y, z)" << endl;
				cout << x_pixels << "," << y_pixels  << "," << z_pixels << "\t" << one_before << "," << two_before << endl;
				
				fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, z_pixels);
				fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), z_pixels); // 
				
				line_color(route_img, &x_pixels, &y_pixels);
			
				for(int i = 0; i < 10000; i++){
					kernel(x_pixels, y_pixels, p, binaryImg);
					binaryImg[y_pixels][x_pixels] = 1;
		
					// 3x3のカーネル内にいくつ線の画素が存在するかを識別する条件
					if((one_before % 2 == 1)  && two_before == -1)
						pattern_one(p, &x_pixels, &y_pixels, &one_before, &two_before);
					else if((one_before % 2 == 0) && two_before == -1)
						pattern_two(p, &x_pixels, &y_pixels, &one_before, &two_before);
					else if(two_before > 0)
						pattern_three(p, &x_pixels, &y_pixels, &one_before, &two_before);
		
					if(old_x_pixels == x_pixels && old_y_pixels == y_pixels){
						break;
					}
			
					cout << x_pixels << "," << y_pixels  << "," << z_pixels << "\t" << one_before << "," << two_before << endl;
					
					fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, z_pixels);
					fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), z_pixels); // 
					
					line_color(route_img, &x_pixels, &y_pixels);
					old_x_pixels = x_pixels;
					old_y_pixels = y_pixels;
				}
			
				(*lines_count) += 1;
				imwrite(output_file, route_img); // 線一つ分を画像に一枚保存する		
				fprintf(fp, "%d,%d,%d\n", x_pixels, y_pixels, 10); // 端点に辿り着いたら一度持ち上げる動作
				fprintf(gp, "%d,%d,%d\n", xy_rotate(x_pixels) , xy_rotate(y_pixels), 10); // 
			}
		}
	}	
//	print_binaryImg(binaryImg); // Print	
	delete[] check;
	fclose(fp);
	fclose(gp);
}


void Normalize(int count[256]){
	int max = count[0], min = count[0];

	// 最大、最小値を探す
	for(int i = 1; i < 256; i++){
		if(count[i] < min)
			min = count[i];
		if(count[i] > max)
			max = count[i];
	}
}

void pixels_total_number(Mat grayImg, int* threshold_n){
	
	// 画素数を数える
	int upper = 0, lower = 0;
	int count[256];
	for(int i = 0; i < 256; i++) // 数を0で初期化
		count[i] = 0;
	for(int y = 0; y < grayImg.rows; y++)
		for(int x = 0; x < grayImg.cols; x++)
			count[grayImg.at<uchar>(y, x)] += 1;
	
	for(int i = 0; i < 256; i++){
//		cout << i   << "\t" << count[i] << endl;
		if(i < 22)	lower += count[i];
		else if(i >= 22)	upper += count[i];
	}
	
	cout << "less than 22 " << lower << endl;
	cout << "more than 22 " << upper << endl;
	
	// しきい値0.078の場所(画素値)を探す
	int pixels_sum = 0;
	double target_value = 0.0783 * (width*height);
	for(int i = 255; i >= 0; i--){
		if(  pixels_sum > target_value ){
			if( ( target_value - pixels_sum )  <  (count[i]+pixels_sum) - target_value  ){
				*threshold_n = i-1;
				return;
			}
			else{
				*threshold_n = i;
				return;
			}
		}
		pixels_sum += count[i];
	}
}

Mat drawHist(const Mat& grayImg, int* threshold_n){
	
	// 画素数を数える
	int count[256];
	for(int i = 0; i < 256; i++) // 数を0で初期化
		count[i] = 0;
	for(int y = 0; y < grayImg.rows; y++)
		for(int x = 0; x < grayImg.cols; x++)
			count[grayImg.at<uchar>(y, x)] += 1;

	// 正規化
	Normalize(count);

	// ヒストグラム生成用の画像を作成
	Mat image_hist = Mat(Size(550, 413), CV_8UC3, Scalar(255, 255, 255));

	// 背景を見やすくグレーにする
	rectangle(image_hist, Point(20, 20), Point(530, 393), Scalar(0, 0, 0), 2);
	rectangle(image_hist, Point(20, 20), Point(530, 393), Scalar(230, 230, 230), -1);
	// ヒストグラムを描画
	for(int i = 0; i < 256; i++){
		// しきい値を赤線で引く
		if(i == *threshold_n)
			line(image_hist, Point(20+i*2, 393), Point(20+i*2, 20), Scalar(0, 0, 255), 2, 8, 0);
		// それ以外は通常の濃淡ヒストグラムにする
		else if(count[i] > 0)
			line(image_hist, Point(20+i*2, 393), Point(20+i*2, 393-count[i]/10), Scalar(0, 255, 0), 2, 8, 0);
		
		// 横軸縦軸に10ずつラインを引く
		if(i%20 == 0)
			line(image_hist, Point(20+i*2, 20), Point(20+i*2, 393), Scalar(170, 170, 170), 1, 8, 0);

		if(i%50 == 0)
			line(image_hist, Point(20, 393-i*2), Point(530, 393-i*2), Scalar(50, 50, 50), 1, 8, 0);
	}
	return image_hist;

}

void Edge_Emphasis(Mat img, int* threshold_n){
	for(int y = 0; y < height; y++){
		for(int x = 0; x < width; x++){
			if( img.at<uchar>(y, x) > *threshold_n )
				img.at<uchar>(y, x) = 255;
			else  
				img.at<uchar>(y, x) = 0;
		}
	}
}
int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("mypkg");
	
	// ラスタスキャンに使う3x3のフィルタ(セグメントというらしい)
	int p[9];
	// 検出した座標を格納するための配列
	int end_point[NUM][XY]; 
	
	string input_filename = "/home/hiromasa/ros2_ws/image/img10.png";
	string output_filename0 = "/home/hiromasa/ros2_ws/image/output/mypkg/gaussian_laplacian.png";
	string output_filename1 = "/home/hiromasa/ros2_ws/image/output/mypkg/thinning.png";
	string output_filename2 = "/home/hiromasa/ros2_ws/image/output/mypkg/histogram.png";
	string output_filename3 = "/home/hiromasa/ros2_ws/image/output/mypkg/edge_emphasis.png";
	string output_filename4 = "/home/hiromasa/ros2_ws/image/output/mypkg/shapeness.png";

	Mat img = imread(input_filename, IMREAD_GRAYSCALE);
	Mat grayImg = imread(input_filename, IMREAD_GRAYSCALE);
	
	if(img.empty())
		cerr << "ERROR:Imge file not found." << endl;

	img = Img_Resize(img);
	
	::width = img.cols; //  x
	::height = img.rows; // y
	cout << "(width, height)=" << width << ", " << height << endl;

	// ガウシアンラプラシアンフィルタ
	const int filterSize = 3;

	GaussianBlur(img, img, Size(filterSize, filterSize), 5);
	Laplacian(img, img, 0);
	imwrite(output_filename0, img);
	
	// 2値化
	int threshold_n = 35;
	Mat thinned_Img;
	Edge_Emphasis(img, &threshold_n);
	imwrite(output_filename3, img);
	
	// シャープ化 4近傍, 8近傍
	const float k = -1.0;
	Mat sharpningKernel4 = (Mat_<float>(3, 3) << 0.0, k, 0.0, k, 5.0, k, 0.0, k, 0.0);
	// Mat sharpningKernel8 = (Mat_<float>(3, 3) << k, k, k, k, 9.0, k, k, k, k);
	filter2D(img, img, img.depth(), sharpningKernel4);
	imwrite(output_filename4, img);

	// 2値化する前の濃淡ヒストグラムを指定先のディレクトリに保存
	pixels_total_number(img, &threshold_n);
	imwrite(output_filename2, drawHist(img, &threshold_n) );
	// cout << threshold_n << "\tis close to 0.783" << endl;

	// 細線化
	// threshold(img, img, 35, 255, THRESH_BINARY);
	ximgproc::thinning(img, thinned_Img, ximgproc::THINNING_ZHANGSUEN);
	imwrite(output_filename1, thinned_Img);
	
	// 動的に配列を確保する
	int** binaryImg = new int* [height];  // binaryImg[x][y]   1(黒)、0(白)で出力
	for(int y = 0; y < height; y++)
		binaryImg[y] = new int [width];
	
	create2DbinaryImg(binaryImg, thinned_Img); // 0, 1に2値化を行う関数
	//print_binaryImg(binaryImg); // Print	
	
	// #### 端点の検出 ####
	int count = 0;
	
	find_endPoint(&count, p, end_point, binaryImg);
	//print_endPoint(&count, end_point);
	cout << "The number of end point is " << count << "." << endl; 
	

	// 経路を求めてファイルに書き込む
	int one_before, two_before = -1; // 3x3のカーネル内で一つ前と２つ前の画素の位置
	int lines_count; // 線の本数をカウントするための変数
	cout << "############################################" << endl;
	route_search(&lines_count, &one_before, &two_before, &count, p, end_point, binaryImg);
	

	// 動的に確保した配列を解放
	for(int y = 0; y < height; y++)
		delete[] binaryImg[y];
	delete[] binaryImg;

	cout << "Completed. route image and gray histogram saved. " << endl;

	return 0;
}

