#include <stdio.h>
#include <math.h>

#define NUM 1000
#define PI 3.14159265359 

#define H 56
#define L_1 40
#define L_2 65
#define L_3 130
#define L_4 93

#define ORIGIN_X 155

void Cardioid(double theta, double *px, double *py){
	 double x = 15*cos(theta)*(1+cos(theta)) + ORIGIN_X;
	 double y = 15*sin(theta)*(1+cos(theta));
	 // rotate to 45 degree
	 *px = x*cos(PI/4) - y*sin(PI/4);
	 *py = x*sin(PI/4) + y*cos(PI/4);
}

double theta_1(double x, double y){
	return atan2(y, x);
}

double theta_3(double x, double y){
	return atan2( 568.125, ( pow(x, 2)+pow(y, 2)+pow(- H - L_1, 2 ) ) / L_3 );
}

double theta_2(double theta3, double px, double py){
	double y = ( L_2+L_3*cos(theta3)-L_4*sin(theta3) )*(sqrt( pow(px, 2)+pow(py, 2) )) -(L_3*sin(theta3) + L_4*cos(theta3) )*(-H - L_1);
 
     double x = ( L_3*sin(theta3)+L_4*cos(theta3) )*(sqrt( pow(px, 2)+pow(py, 2) )) + ( L_2+L_3*cos(theta3) - L_4*sin(theta3) )*(-H - L_1);
 	
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

void theta_print(double degree, double *px, double *py){
	double Theta1 = theta_1(*px, *py);
	double Theta3 = theta_3(*px, *py);
	double Theta2 = theta_2( theta_3(*px, *py), *px, *py );

	printf("%3.1lf\t%3.7lf\t%3.7lf\t%3.7lf\t%3.7lf\t%3.7lf\t%3.7lf\t%3.7lf\t%3.7lf\n", degree, *px, *py, Theta1, Theta2, Theta3, Theta1 * 180 / PI, Theta2 * 180 / PI, Theta3 * 180 / PI);
//	printf("degree=%3.1lf\tx=%3.7lf\ty=%3.7lf\ttheta_1=%3.7lf\ttheta_2=%3.7lf\ttheta_3=%3.7lf\tdegree_1=%3.7lf\tdegree_2=%3.7lf\tdegree_3=%3.7lf\n", degree, *px, *py, Theta1, Theta2, Theta3, Theta1 * 180 / PI, Theta2 * 180 / PI, Theta3 * 180 / PI);
}

void distance(int i, double *px, double *py){
	if( sqrt(pow(*px, 2) + pow(*py, 2) ) > 195)
		printf("%3d\t", i);
}

int main(void){
	int i = 0;
	double degree = 0.0;
	double orbit_x[NUM], orbit_y[NUM];
	double servo1[NUM], servo2[NUM], servo3[NUM];
	int angle1[NUM], angle2[NUM], angle3[NUM];

	while(degree < 360){
		Cardioid(degree * PI / 180, &orbit_x[i], &orbit_y[i++]);
		degree += 1.0;
	}
	degree = 0.0;
	for(int j = 0; j < i; j++){
		Inverse_Kinematics(&orbit_x[j], &orbit_y[j], &servo1[j], &servo2[j], &servo3[j]);
		angle_conversion(&angle1[j], &angle2[j], &angle3[j], &servo1[j], &servo2[j], &servo3[j]);
		theta_print(degree, &orbit_x[j], &orbit_y[j]);
//		printf("degree=%lf\tx=%lf\ty=%lf\n", degree, orbit_x[j], orbit_y[j]);
		degree+=1.0;
	}

	for(int j = 0; j < i; j++)
		distance(j, &orbit_x[j], &orbit_y[j]);
	putchar('\n');
	
	return 0;
}
