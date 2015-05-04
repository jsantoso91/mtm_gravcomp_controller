#!/usr/bin/env python
from math import cos, sin

g = 9.81
m1 = 0.9; m2 = 1; m3 = 0.9; m4 = 0.8; m5 = 0.4; m6 = 0.2; m7 = 0.2; m8 = 0.1
theta1 = 0; theta2 = 0; theta3 = 0
theta4 = 0; theta5 = 0; theta6 = 0; theta7 = 0;

torque1 = 0;

torque2 = ((g*(3645*m5*cos(theta2 + theta3) - 390*m8*cos(theta2 + theta3 + theta4 + theta5) + 
    3645*m6*cos(theta2 + theta3) + 3645*m7*cos(theta2 + theta3) + 3645*m8*cos(theta2 + theta3) - 
    1560*m5*sin(theta2 + theta3) - 1560*m6*sin(theta2 + theta3) - 1560*m7*sin(theta2 + theta3) - 
    1560*m8*sin(theta2 + theta3) + 2794*m4*sin(theta2) + 2794*m5*sin(theta2) + 2794*m6*sin(theta2) + 
    2794*m7*sin(theta2) + 2794*m8*sin(theta2)))/10000);


torque3 = (-(3*g*(26*m8*cos(theta2 + theta3 + theta4 + theta5) - 243*m5*cos(theta2 + theta3) - 
    243*m6*cos(theta2 + theta3) - 243*m7*cos(theta2 + theta3) - 243*m8*cos(theta2 + theta3) + 
    104*m5*sin(theta2 + theta3) + 104*m6*sin(theta2 + theta3) + 104*m7*sin(theta2 + theta3) + 
    104*m8*sin(theta2 + theta3)))/2000);

torque4 = -(39*g*m8*cos(theta2 + theta3 + theta4 + theta5))/1000;


torque5 = -(39*g*m8*cos(theta2 + theta3 + theta4 + theta5))/1000;

torque6 = 0;

torque7 = 0;

print(torque1)
print(torque2)
print(torque3)
print(torque4)
print(torque5)
print(torque6)
print(torque7)