#!/usr/bin/env python

import rospy
import numpy as np
from sympy import *
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3				#vector3 nuevo
from vector_3_array.msg import Vector3DArray			#vector3 nuevo

t2=Symbol('t2') #theta2
t3=Symbol('t3') #theta3
l2=Symbol('l2') #distancia l2
l3=Symbol('l3') #distancia l3

def DH_matrix(t,d,a,alph):
	T=np.array([[cos(t),-cos(alph)*sin(t),sin(alph)*sin(t),a*cos(t)],
		[sin(t),cos(alph)*cos(t),-sin(alph)*cos(t),a*sin(t)],
		[0,sin(alph),cos(alph),d],[0,0,0,1]])
	return T

def subs_dh(T,t2,t3,l2,l3,par):
	[m,n]=T.shape 
	T_subs=np.zeros(T.shape)
	for i in range(0,m):
		for j in range (0,n):
			T_subs[i,j]=T[i,j].subs([(t2,par[0,0]),(t3,par[0,1]),(l2,par[0,2]),(l3,par[0,3])])
	return T_subs

T01 = DH_matrix(0,0,0,np.pi/2)
T12 = DH_matrix(t2,0,l2,0)
T23 = DH_matrix(t3,0,l3,0)
T02=np.dot(T01,T12)
T03=np.dot(T02,T23)
print T03
#print("--------------Px------------")
#print T03[0,3]
#print("--------------Py------------")
#print T03[1,3]
#print("--------------Pz------------")
#print T03[2,3]

angles = np.array([[0,0],[np.pi/2,np.pi/2],[0,np.pi/2],[np.pi/2,0],[np.pi,0],[0,np.pi],[np.pi/4,np.pi/2],[np.pi/4,np.pi/4],[-np.pi/2,np.pi/2],[-np.pi/4,np.pi]])
covers = np.array([[0,0,0],[np.pi/2,np.pi/2,np.pi/2],[0,0,0],[np.pi/2,np.pi/2,np.pi/2],[0,0,0],[np.pi/2,np.pi/2,np.pi/2],[0,0,0],[np.pi/2,np.pi/2,np.pi/2]])		#vector3 nuevo

def talker():
#    pub_angle = rospy.Publisher('angles', Point, queue_size=10)	#quitar el numeral
    pub_angle = rospy.Publisher('covers', Vector3, queue_size=10)	#vector3 nuevo borrar
    pub_position = rospy.Publisher('positions', Quaternion, queue_size=10)

    rospy.init_node('angle_sender', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    point_message=Point()
    quaternion_message= Quaternion()
    vector3_message=Vector3()			#vector3 nuevo	
    index=0
    while not rospy.is_shutdown():
#	parameters = np.array([[angles[index][0],angles[index][1],0.4,0.3]]) 	#quitar el numeral
	parameters = np.array([[angles[index][index][0],angles[index][index][1],0.4,0.3]]) 	#vector3 nuevo
	position_matrix = subs_dh(T03,t2,t3,l2,l3,parameters)
	quaternion_message.x = position_matrix[0][3]	
	quaternion_message.y = position_matrix[1][3]		
	quaternion_message.z = position_matrix[2][3]		
	quaternion_message.w = 0	
#	point_message.x= angles[index][0]		#quitar el numeral
#	point_message.y= angles[index][1]		#quitar el numeral
#	point_message.z= 0				#quitar el numeral
	vector3_message.x= covers[index][index][0]	#vector3 nuevo
	vector3_message.y= covers[index][index][0]	#vector3 nuevo
	vector3_message.z= covers[index][index][0]	#vector3 nuevo
#       hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo('hello')
#       pub_angle.publish(point_message)		#quitar el numeral
	pub_angle.publish(vector3_message)
	pub_position.publish(quaternion_message)
        rate.sleep()
	
	index=index+1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

