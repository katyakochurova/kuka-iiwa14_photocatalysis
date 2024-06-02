import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sympy as sym
sym.init_printing()
from sympy import *
import numpy as np
from numpy import *
import math
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = sym.symbols("\\theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7")
d1, d3, d5, d7 = sym.symbols("d_1,d_3,d_5,d_7")
d1 = 360
d2 = 0
d3 = 420
d4 = 0
d5 = 399.5
d6 = 0
d7 = 205.5
m1 = 4
m2 = 4
m3 = 4
m4 = 4
m5 = 4
m6 = 4
m7 = 0
g = -9.81
al1 = -pi/2
al2 = pi/2
al3 = pi/2
al4 = -pi/2
al5 = -pi/2
al6 = pi/2
al7 = 0
a1 = 0
a2 = 0
a3 = 0
a4 = 0
a5 = 0
a6 = 0
a7 = 0
A11 = [sym.cos(theta1), -sym.cos(al1) * sym.sin(theta1), sin(al1) * sym.sin(theta1), a1 * sym.cos(theta1)]
A12 = [sym.sin(theta1), cos(al1) * sym.cos(theta1), -sin(al1) * sym.cos(theta1), a1 * sym.sin(theta1)]
A13 = [0, sin(al1), cos(al1), d1]
A14 = [0, 0, 0, 1]
A1 = sym.Matrix([A11, A12, A13, A14])
A21 = [sym.cos(theta2), -sym.cos(al2) * sym.sin(theta2), sin(al2) * sym.sin(theta2), a2 * sym.cos(theta2)]
A22 = [sym.sin(theta2), cos(al2) * sym.cos(theta2), -sin(al2) * sym.cos(theta2), a2 * sym.sin(theta2)]
A23 = [0, sin(al2), cos(al2), d2]
A24 = [0, 0, 0, 1]
A2 = sym.Matrix([A21, A22, A23, A24])
A31 = [sym.cos(theta3), -sym.cos(al3) * sym.sin(theta3), sin(al3) * sym.sin(theta3), a3 * sym.cos(theta3)]
A32 = [sym.sin(theta3), cos(al3) * sym.cos(theta3), -sin(al3) * sym.cos(theta3), a3 * sym.sin(theta3)]
A33 = [0, sin(al3), cos(al3), d3]
A34 = [0, 0, 0, 1]
A3 = sym.Matrix([A31, A32, A33, A34])
A41 = [sym.cos(theta4), -sym.cos(al4) * sym.sin(theta4), sin(al4) * sym.sin(theta4), a4 * sym.cos(theta4)]
A42 = [sym.sin(theta4), cos(al4) * sym.cos(theta4), -sin(al4) * sym.cos(theta4), a4 * sym.sin(theta4)]
A43 = [0, sin(al4), cos(al4), d4]
A44 = [0, 0, 0, 1]
A4 = sym.Matrix([A41, A42, A43, A44])
A51 = [sym.cos(theta5), -sym.cos(al5) * sym.sin(theta5), sin(al5) * sym.sin(theta5), a5 * sym.cos(theta5)]
A52 = [sym.sin(theta5), cos(al5) * sym.cos(theta5), -sin(al5) * sym.cos(theta5), a5 * sym.sin(theta5)]
A53 = [0, sin(al5), cos(al5), d5]
A54 = [0, 0, 0, 1]
A5 = sym.Matrix([A51, A52, A53, A54])
A61 = [sym.cos(theta6), -sym.cos(al6) * sym.sin(theta6), sin(al6) * sym.sin(theta6), a6 * sym.cos(theta6)]
A62 = [sym.sin(theta6), cos(al6) * sym.cos(theta6), -sin(al6) * sym.cos(theta6), a6 * sym.sin(theta6)]
A63 = [0, sin(al6), cos(al6), d6]
A64 = [0, 0, 0, 1]
A6 = sym.Matrix([A61, A62, A63, A64])
A71 = [sym.cos(theta7), -sym.cos(al7) * sym.sin(theta7), sin(al7) * sym.sin(theta7), a7 * sym.cos(theta7)]
A72 = [sym.sin(theta7), cos(al7) * sym.cos(theta7), -sin(al7) * sym.cos(theta7), a7 * sym.sin(theta7)]
A73 = [0, sin(al7), cos(al7), d7]
A74 = [0, 0, 0, 1]
A7 = sym.Matrix([A71, A72, A73, A74])
A12 = A1 * A2
A23 = A12 * A3
A34 = A23 * A4
A45 = A34 * A5
A56 = A45 * A6
A67 = A56 * A7
A = A1 * A2 * A3 * A4 * A5 * A6 * A7
Z0 = sym.Matrix([0,0,1])
Z1 = A1[:3,2]
Z2 = A12[:3,2]
Z3 = A23[:3,2]
Z4 = A34[:3,2]
Z5 = A45[:3,2]
Z6 = A56[:3,2]
Z7 = A67[:3,2]
O0 = sym.Matrix([0, 0, 0])
O1 = A1[:3,3]
O2 = A12[:3,3]
O3 = A23[:3,3]
O4 = A34[:3,3]
O5 = A45[:3,3]
O6 = A56[:3,3]
O7 = A67[:3,3]
px = A[0,3]; py = A[1,3]; pz = A[2,3];
a11 = sym.diff(px, theta1)
a12 = sym.diff(px, theta2)
a13 = sym.diff(px, theta3)
a14 = sym.diff(px, theta4)
a15 = sym.diff(px, theta5)
a16 = sym.diff(px, theta6)
a17 = sym.diff(px, theta7)
a21 = sym.diff(py, theta1)
a22 = sym.diff(py, theta2)
a23 = sym.diff(py, theta3)
a24 = sym.diff(py, theta4)
a25 = sym.diff(py, theta5)
a26 = sym.diff(py, theta6)
a27 = sym.diff(py, theta7)
a31 = sym.diff(pz, theta1)
a32 = sym.diff(pz, theta2)
a33 = sym.diff(pz, theta3)
a34 = sym.diff(pz, theta4)
a35 = sym.diff(pz, theta5)
a36 = sym.diff(pz, theta6)
a37 = sym.diff(pz, theta7)
J = sym.Matrix([[a11, a12, a13, a14, a15, a16, a17], [a21, a22, a23, a24, a25, a26, a27],[a31, a32, a33, a34, a35, a36, a37],[Z1, Z2, Z3, Z4, Z5, Z6, Z7], [0, 0, 0, 0, 0, 0, 1]])
P1 = -1*m1*g*(O1[2]+O0[2])*0.5
P2 = -1*(m1+m2)*g*(O2[2]+O1[2])*0.5
P3 = -1*(m1+m2+m3)*g*(O3[2]+O2[2])*0.5
P4 = -1*(m1+m2+m3+m4)*g*(O4[2]+O3[2])*0.5
P5 = -1*(m1+m2+m3+m4+m5)*g*(O5[2]+O4[2])*0.5
P6 = -1*(m1+m2+m3+m4+m5+m6)*g*(O6[2]+O5[2])*0.5
P7 = -1*(m1+m2+m3+m4+m5+m6+m7)*g*(O7[2]+O6[2])*0.5
P = sym.Matrix([[P1], [P2], [P3], [P4], [P5], [P6], [P7]])
Fw = sym.Matrix([[0], [-5], [0], [0], [0], [0], [0]])
def plotcircle(x, y, z):
    rospy.init_node('publish_node', anonymous=True)
    turning = rospy.Publisher('turn', Float64MultiArray, queue_size=10) 
    rate = rospy.Rate(10)
    rospy.loginfo("Analysing the Robot!!!")
    while not rospy.is_shutdown():
        theta_joint = sym.Matrix([0,90,0,-90,-90,90,0]) * (pi/180)
        N = 40
        th = linspace(float(1), float(80),num=N)
        twist = Float64MultiArray()
        V = Matrix([x, y, z, 0.0, 0.0, 0.0, 0.0])
        J_inv = J.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta3:theta_joint[2],theta4:theta_joint[3],theta5:theta_joint[4],theta6:theta_joint[5],theta7:theta_joint[6]}).inv()
        theta_dot = J_inv*V
        theta_joint = theta_joint + (theta_dot*(5/N))
        print(theta_joint)
        twist.data = theta_joint
        turning.publish(twist)
        rate.sleep()
        return(theta_joint)
if __name__ == '__main__':
    try:
        plotcircle(0,0,0)
        plotcircle(0,0,700)
        plotcircle(52,285,700)
        plotcircle(52,285,7)
        rate.sleep(3)
        plotcircle(52,285,700)
        plotcircle(96,400,700)
        plotcircle(96,400,54)
        rate.sleep(3)
        plotcircle(96,400,700)
        rate.sleep(10)
        plotcircle(96,400,54)
        rate.sleep(3)
        plotcircle(96,400,700)
        plotcircle(692,0,700)
        plotcircle(692,0,150)
        plotcircle(692,110,150)
        plotcircle(692,110,145)
        rate.sleep(3)
        plotcircle(692,0,145)
        rate.sleep(3600)
        plotcircle(692,110,145)
        rate.sleep(3)
        plotcircle(692,110,150)
        plotcircle(692,0,150)
        plotcircle(692,0,700)
        plotcircle(96,400,700)
        plotcircle(96,400,54)
        rate.sleep(3)
        plotcircle(96,400,700)
        plotcircle(0,0,700)
    except rospy.ROSInterruptException: 
        pass
