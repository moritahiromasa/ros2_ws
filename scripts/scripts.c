#include <stdio.h>
#include <math.h>


#define NUM 1000
#define PI 3.14159265359 

#define H 56
#define L_1 40
#define L_2 65
#define L_3 130
#define L_4 96

#define Z 96

#define ORIGIN_X 155

void Cardioid(double theta, double *px, double *py){
	 double x = 15*cos(theta)*(1+cos(theta)) + ORIGIN_X;
	 double y = 15*sin(theta)*(1+cos(theta));
	 //rotate to 45 degree
	 *px = x*cos(PI/4) - y*sin(PI/4);
	 *py = x*sin(PI/4) + y*cos(PI/4);
}

double theta_1(double x, double y){
	return atan2(y, x);
}

double theta_3(double x, double y){
	return acos(( pow(x, 2)+pow(y, 2)+ pow( Z -( H + L_1), 2) + pow(L_2,2) + pow(L_3,2)) / (2*L_2*L_3) );
}

double theta_2(double theta3, double px, double py){
	double y =  L_3*sin(theta3)*(sqrt( pow(px, 2)+pow(py, 2) )) +(L_2+L_3*cos(theta3))*Z;
 
     double x = ( -L_3*sin(theta3))*(sqrt( pow(px, 2)+pow(py, 2) )) + ( L_2+L_3*cos(theta3))*Z;
 	
	return atan2(y, x);
}

void Inverse_Kinematics(double *px, double *py, double *servo1, double *servo2, double *servo3){
	*servo1 = theta_1(*px, *py) * 180 / PI;
	*servo3 = theta_3(*px, *py) * 180 / PI;
	*servo2 = theta_2( theta_3(*px, *py), *px, *py );
}

void angle_conversion(int *angle1, int *angle2, int *angle3, double *servo1, double *servo2, double *servo3){
	*angle1 = 500 + (int)((*servo1) * 11.1);
	*angle2 = 500 + (int)((*servo2) * 11.1);
	*angle3 = 500 + (int)((*servo3) * 11.1);
}

void theta_print(double degree, double *px, double *py, int *angle1, int *angle2, int *angle3){
	double Theta1 = theta_1(*px, *py);
	double Theta3 = theta_3(*px, *py);
	double Theta2 = theta_2( theta_3(*px, *py), *px, *py );

	printf("degree=%3.1lf\tx=%3.7lf\ty=%3.7lf\ttheta_1=%3.7lf\ttheta_2=%3.7lf\ttheta_3=%3.7lf\tdegree_1=%3.7lf\tdegree_2=%3.7lf\tdegree_3=%3.7lf", degree, *px, *py, Theta1, Theta2, Theta3, Theta1 * 180 / PI, Theta2 * 180 / PI, Theta3 * 180 / PI);
	printf("\tangle1=%d\tangle2=%d\tangle3=%d\n", *angle1, *angle2, *angle3);
}

void distance(int i, double *px, double *py){
	if( sqrt(pow(*px, 2) + pow(*py, 2) ) > 195)
		printf("%3d\t", i);
}

int main(void){
	double theta1 = atan2(136.63818, 117.3522);
	printf("atan2=%lf\n", atan2(136.63818, 117.3522));
	printf("degree=%lf\n", theta1 * 180 / PI);

	return 0;
}	
