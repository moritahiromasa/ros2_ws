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
	return xy + 74;
}

Mat Img_Resize(Mat img){
	// 長方形の画像を正方形にリサイズ
	if(img.cols != img.rows){
		if(img.cols < img.rows)	// タテ長の場合
			resize(img, img, Size(), 64.0/ img.cols, 64.0/ img.cols);
		else if(img.cols < img.rows) // ヨコ長の場合
			resize(img, img, Size(), 64.0/ img.rows, 64.0/ img.rows);
	}

	if(img.cols > 64) 
		resize(img, img, Size(), 64.0/ img.rows, 64.0/ img.rows);

	return img;
}

void route_search(int* lines_count, int* one, int* two, int p[9], int** binaryImg){
	FILE* fp = fopen("/home/morita/ros2_ws/CSV/normal_coordinate.csv", "w");
	FILE* gp = fopen("/home/morita/ros2_ws/CSV/correct_coordinate.csv", "w");
	
	string input_file = "/home/morita/ros2_ws/output/canny.jpg";
	Mat route_img = imread(input_file, IMREAD_COLOR);
	
	// 長方形の画像を正方形にリサイズ
	route_img = Img_Resize(route_img);
	
	
	string output_file = "/home/morita/ros2_ws/output/route/route_0.jpg";
	route_img.setTo(Scalar(255, 255, 255));
	imwrite(output_file, route_img);
	

	int x_pixels, y_pixels, z_pixels; // 注目画素を格納する変数
	int one_before = *one, two_before = *two;
	int old_x_pixels, old_y_pixels;
	
	if(fp == NULL)
		cerr << "ERROR:test.csv not found." << endl;

	// 不要な孤立点を削除する
	isolate_point(p, binaryImg);


	// 端点がない線をたどる
	cout << "it will follow no end point line " << endl;
	for(int y = 1; y < height-1; y++){
		for(int x = 1; x < width-1; x++){
			string output_file = "/home/morita/ros2_ws/output/route/route_" + to_string(*lines_count) + ".jpg";
			
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
	fclose(fp);
	fclose(gp);
}



int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("rasta");
	
	// ラスタスキャンに使う3x3のフィルタ(セグメントというらしい)
	int p[9];
	
	string input_filename = "/home/morita/ros2_ws/output/canny.jpg";

	Mat img = imread(input_filename, IMREAD_GRAYSCALE);
	
	if(img.empty())
		cerr << "ERROR:Imge file not found." << endl;

	img = Img_Resize(img);
	
	::width = img.cols; //  x
	::height = img.rows; // y
	cout << "(width, height)=" << width << ", " << height << endl;

	
	// 動的に配列を確保する
	int** binaryImg = new int* [height];  // binaryImg[x][y]   1(黒)、0(白)で出力
	for(int y = 0; y < height; y++)
		binaryImg[y] = new int [width];
	
	create2DbinaryImg(binaryImg, img); // 0, 1に2値化を行う関数
	//print_binaryImg(binaryImg); // Print	
	
	
	// 経路を求めてファイルに書き込む
	int one_before, two_before = -1; // 3x3のカーネル内で一つ前と２つ前の画素の位置
	int lines_count; // 線の本数をカウントするための変数
	cout << "############################################" << endl;
	route_search(&lines_count, &one_before, &two_before, p, binaryImg);
	

	// 動的に確保した配列を解放
	for(int y = 0; y < height; y++)
		delete[] binaryImg[y];
	delete[] binaryImg;

	cout << "Completed. Route derived from img3.png and CSV file saved. " << endl;

	return 0;
}
