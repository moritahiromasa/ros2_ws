#!/usr/bin/env python3

import pigpio
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
l1 = 40
l2 = 65
l3 = 130
l4 = 96

pi = pigpio.pi()

# 初期姿勢
def init_position( ):
	pi.set_servo_pulsewidth( SERVO_PIN_2, 500 ) # SERVO1はtheta1に相当
	time.sleep( 1 )

	pi.set_servo_pulsewidth( SERVO_PIN_1, 500 ) # SERVO2はtheta2に相当
	time.sleep( 1 )
	
	pi.set_servo_pulsewidth( SERVO_PIN_3, 2500 ) # SERVO3はtheta3に相当
	time.sleep( 1 )

def test_position( ):
	pi.set_servo_pulsewidth( SERVO_PIN_1, 500 ) # SERVO1はtheta1に相当
	time.sleep( 1 )
	pi.set_servo_pulsewidth( SERVO_PIN_2, 830 ) # SERVO2はtheta2に相当
	time.sleep( 0.5 )
	pi.set_servo_pulsewidth( SERVO_PIN_3, 2000 ) # SERVO3はtheta3に相当
	time.sleep( 0.5 )


##### theta1 ~ theta3までの角度を求める ####
def THETA_1(x, y):
	return ( 0.5*m.acos( (x**2 - y**2) / ( x**2 + y**2) ))

def THETA_2(x, y, z):
	return m.acos( (-l2**2 + l3**2 + l4**2 - (x**2 + y**2) - (z - l1)**2) / ( 2*l2*m.sqrt(x**2 + y**2 + (z - l1)**2) ) ) + m.atan(-m.sqrt(x**2 + y**2) / (l1 - z))

def THETA_3( x, y, z):
	return m.acos( ( x**2 + y**2 + (z - l1)**2 - l4**2 - l3**2 - l2**2) / (2*l2*m.sqrt(l3**2 + l4**2)) ) + m.atan(-l4 / l3)

def InverseKinematics():
	count = int(0)
	
	with open('/home/ubuntu/ros2_ws/CSV/correct_coordinate.csv') as file:
		reader = csv.reader(file) # for文で行ごとのデータをリストで取得		
		for row in reader:
			x = float(row[0])
			y = float(row[1])
			z = float(row[2])
			
			# 各サーボのtheta1~3を求め
			theta1 = THETA_1(x, y)
			theta2 = THETA_2(x, y, z)
			theta3 = THETA_3(x, y, z)
			
			degree1 = theta1*180/m.pi 
			degree2 = theta2*180/m.pi
			degree3 = theta3*180/m.pi
			
			servo1_degree = round(2000/180*degree1)+ 500
			servo2_degree = round(2000/180*degree2)+ 500
			servo3_degree = 2500 - round(2000/180*degree3)
			
			if count == 0:
				iterate1 = round(180/2000*(servo1_degree-500))
				iterate2 = round(180/2000*(servo2_degree-500))
				iterate3 = round(180/2000*(2500 - servo3_degree))
				for i in range(iterate1):
					pi.set_servo_pulsewidth( SERVO_PIN_1, round(2000/180*i)+ 500 ) 
					time.sleep( 0.05 )
				for i in range(iterate2):
					pi.set_servo_pulsewidth( SERVO_PIN_2, round(2000/180*i)+ 500 ) 
					time.sleep( 0.05 )
				for i in range(iterate3):
					pi.set_servo_pulsewidth( SERVO_PIN_3, 2500 - round(2000/180*i) ) 
					time.sleep( 0.1 )
				count = int(1)


			
			#print("servo1: {}, servo2: {}, servo3: {}".format(servo1_degree, servo2_degree, servo3_degree))

			# 各サーボの制御角度を求める
			if z == 10:
				time.sleep(1.4)
				pi.set_servo_pulsewidth( SERVO_PIN_1, servo1_degree ) # SERVO1はtheta1に相当
				time.sleep(1.4)
				pi.set_servo_pulsewidth( SERVO_PIN_3, servo3_degree ) # SERVO2はtheta2に相当
				time.sleep(1.4)
				pi.set_servo_pulsewidth( SERVO_PIN_2, servo2_degree ) # SERVO3はtheta3に相当
			time.sleep( 0.5 )
			pi.set_servo_pulsewidth( SERVO_PIN_3, servo3_degree ) # SERVO2はtheta2に相当
			time.sleep( 0.5 )
			pi.set_servo_pulsewidth( SERVO_PIN_2, servo2_degree ) # SERVO3はtheta3に相当
			time.sleep( 0.5 )


# main関数
def main(argv):
	#test_position()
	InverseKinematics()
	init_position()
	
	return 0

if __name__ == '__main__':
	sys.exit(main(sys.argv))



