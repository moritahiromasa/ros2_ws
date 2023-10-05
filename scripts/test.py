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
	return ( m.acos( (x**2 - y**2) / ( x**2 + y**2) ) / 2)

def THETA_3( x, y, z, theta1):
	return m.acos( (l4**2 + l3**2 + l2**2 - (y / m.sin(theta1))**2 - (z - l1)**2 ) / ( 2*l2*m.sqrt( l4**2 + l3**2 ) )) + m.acos( -l3 / m.sqrt( l4**2 + l3**2 ))  

def THETA_2(x, y, z, theta1, theta3):
	return m.acos( ( -l4*m.sin(theta3) + l3*m.cos(theta3) + l2) / m.sqrt( (y / m.sin(theta1))**2 + (z - l1)**2) ) + m.acos( (z -l1) / m.sqrt( (y / m.sin(theta1) )**2 + (z -l1)**2 ) ) 

#### 逆運動学の解を求める  ####


def InverseKinematics():
	with open('/home/hiromasa/ros2_ws/CSV/correct_plane.csv') as file:
		reader = csv.reader(file) # for文で行ごとのデータをリストで取得
		for row in reader:
			# 各サーボのtheta1~3を求める
			theta1 = THETA_1( int(row[0]), int(row[1]) )
			servo_degree1 = m.degrees( theta1 ) % 180
			
			theta3 = THETA_3( int(row[0]), int(row[1]), int(row[2]), theta1 )
			servo_degree3 = m.degrees( theta3 ) % 180
			
			theta2 = THETA_2( int(row[0]), int(row[1]), int(row[2]), theta1, theta3 )
			servo_degree2 = m.degrees( theta2 ) % 180

				
#			print("theta1: {}, theta2: {}, theta3: {}".format(theta1, theta2, theta3))
			print("servo1: {}, servo2: {}, servo3: {}".format(servo_degree1, servo_degree2, servo_degree3))


# main関数
def main():
	InverseKinematics()
	return 0

if __name__ == '__main__':
	sys.exit(main())




