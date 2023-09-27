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

double theta_1(int* px, int* py){
	return ( arccos( (x^2 - y^2) / (x^2 + y^2 )) / 2);
}

double theta_3(int* px, int* py, int* pz){
	double a = ( 2*L_2*L_4 / (L_4^2+l_3^2+L_2^2 - (*px^2+*py^2+ (*pz - L_1)^2 )));
	double b = ( 2*L_2*L_3 / (L_4^2+l_3^2+L_2^2 - (*px^2+*py^2+ (*pz - L_1)^2 )));

	return (arccos( 1 / pow( a^2 + b^2, 0.5) ) + arccos( b / pow( a^2 + b^2, 0.5))); 

}

double theta_2(int* px, int* py, int* pz){
	double c =	( (L_2*sin( theta_3(*px, *py, *pz )) - L_4)/ ( cos(theta_3(*px, *py, *pz)*(*pz - L_1)) + sin(theta_3(*px, *py, *pz)*( x / cos(theta_1(*px, *py)) )) ) );
	double d =	( (L_2*sin( theta_3(*px, *py, *pz) ) + L_3) / ( cos(theta_3(*px, *py, *pz)*(*pz - L_1)) + sin( theta_3(*px, *py, *pz)*(x / cos(theta_1(*px, *py)))) ) );

	return (arccos( 1 / pow( c^2 + d^2, 0.5) ) + arccos( d / pow( c^2 + d^2, 0.5))); 
}

void InverseKinematics(){
	FILE *fp;
	int x, y, z;

	// パスは適宜、変更が必要
	if( (fp = fopen("/home/hiromasa/ros2_ws/CSV/HelloKitty.csv", "rt")) == NULL){
		printf("not found file.\n");
		return;
	}

	while(fscanf(fp, "%d", "%d", "%d", &x, &y, &z) != EOF){
		if(z == 10)
			time
	}
	
	fclose(fp);

	printf("");
	
}

int main(void){

	return 0;
}
