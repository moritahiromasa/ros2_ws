#!/usr/bin/env python3

import time
import sys
import math as m
import csv
import pprint

# 各サーボモータのGPIOピン
SERVO_PIN_2 = 17
SERVO_PIN_1 = 18
SERVO_PIN_3 = 19

# 各リンクの長さ
h = 56 
l1 = 40
l2 = 65
l3 = 130
l4 = 40

##### theta1 ~ theta3までの角度を求める ####
def THETA_1(x, y):
	return ( 0.5*m.acos( (x**2 - y**2) / ( x**2 + y**2) ))

def THETA_2(x, y, z):
	return m.acos( (-l2**2 + l3**2 + l4**2 - (x**2 + y**2) - (z - l1)**2) / ( 2*l2*m.sqrt(x**2 + y**2 + (z - l1)**2) ) ) + m.atan(-m.sqrt(x**2 + y**2) / (l1 - z))

def THETA_3( x, y, z):
	return m.acos( ( x**2 + y**2 + (z - l1)**2 - l4**2 - l3**2 - l2**2) / (2*l2*m.sqrt(l3**2 + l4**2)) ) + m.atan(-l4 / l3)


#### 逆運動学の解を求める  ####


def InverseKinematics():
	for x in range(1, 196):	# 1 <= x <= 195
		# 各サーボのtheta1~3を求める
		y = m.sqrt(195**2 - x**2)
		z = float(0)
		theta1 = THETA_1( float(x), float(y))
		theta2 = THETA_2( float(x), float(y), z)
		theta3 = THETA_3( float(x), float(y), z)
		
		degree1 = theta1*180/m.pi
		degree2 = theta2*180/m.pi
		degree3 = theta3*180/m.pi
				
		#print("theta1: {}, theta2: {}, theta3: {}".format(theta1, theta2, theta3))
		print("degree1: {}, degree2: {}, degree3: {}".format(round(degree1), round(degree2), round(degree3)))
		#print("servo1: {}, servo2: {}, servo3: {}".format(servo_degree1, servo_degree2, servo_degree3))


# main関数
def main():
	InverseKinematics()
	return 0

if __name__ == '__main__':
	sys.exit(main())




