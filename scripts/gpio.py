#!/usr/bin/env python3

import pigpio
import time
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
l4 = 96

pi = pigpio.pi()

with open('/home/hiromasa/ros2_ws/CSV/HelloKitty.csv') as f:
	# print(f.read())
	render = csv.render(f) # for文で行ごとのデータをリストで取得
	position = [row for row in render] # 2次元配列をして取得
# print(position)

# 初期姿勢
def init_position( ):
	pi.set_servo_pulsewidth( SERVO_PIN_1, 1750 ) # SERVO1はtheta1に相当
	time.sleep( 1 )

	pi.set_servo_pulsewidth( SERVO_PIN_2, 500 ) # SERVO2はtheta2に相当
	time.sleep( 1 )
	
	pi.set_servo_pulsewidth( SERVO_PIN_3, 500 ) # SERVO3はtheta3に相当
	time.sleep( 1 )

##### theta1 ~ theta3までの角度を求める ####
def THETA_1(x, y):
	return ( m.acos( (x**2 - y**2) / ( x**2 + y**2) ) / 2)

def THETA_3( x, y, z):
	a = 2*l2*l4 / (l4**2 + l3**2 + l2**2 - (x**2 + y**2 + (z - l1)**2 ) ) 
	b = 2*l2*l3 / (l4**2 + l3**2 + l2**2 - (x**2 + y**2 + (z - l1)**2 ) ) 
	return ( m.acos( 1/ pow(a**2 + b**2, 0.5)) + m.acos(b / pow(a**2 + b**2, 0.5)) )

def THETA_2(x, y, z):
	c = (l2*m.sin(theta_3(x, y, z)) - l4) / ( m.cos(theta_3(x, y, z))*(z - l1) + m.sin(theta_3(x, y, z))*(x / m.cos(theta_1(x, y)) ))
	d = (l2*m.sin(theta_3(x, y, z)) + l3) / ( m.cos(theta_3(x, y, z))*(z - l1) + m.sin(theta_3(x, y, z))*(x / m.cos(theta_1(x, y)) ))
	return ( m.acos( 1/ pow(c**2 + d**2, 0.5)) + m.acos(d / pow(c**2 + d**2, 0.5)) )

#### 逆運動学の解を求める  ####


def InverseKinematics():
	with open('/home/hiromasa/ros2_ws/CSV/HelloKitty.csv') as file:
		render = csv.render(file) # for文で行ごとのデータをリストで取得
		position = [row for row in render] # 2次元配列をして取得
		
		# 各サーボのtheta1~3を求める
		theta1 = THETA_1( (int)(position[row][0]*11.1+500), (int)(position[row][1]*11.1+500))
		theta2 = THETA_2( (int)(position[row][0]*11.1+500), (int)(position[row][1]*11.1+500), (int)(position[row][2]*11.1+500) )
		theta3 = THETA_3( (int)(position[row][0]*11.1+500), (int)(position[row][1]*11.1+500), (int)(position[row][2]*11.1+500) )
		 
		# 各サーボの制御角度を求める
		pi.set_servo_pulsewidth( SERVO_PIN_1, theta1 ) # SERVO1はtheta1に相当
		pi.set_servo_pulsewidth( SERVO_PIN_2, theta2 ) # SERVO2はtheta2に相当
		pi.set_servo_pulsewidth( SERVO_PIN_3, theta3 ) # SERVO3はtheta3に相当
		time.sleep( 0.5 )


# main関数
def main(argv):
	init_position():
#	InverseKinematics():
	
	return 0

if __name__ == '__main__':
	sys.exit(main(sys.argv))




