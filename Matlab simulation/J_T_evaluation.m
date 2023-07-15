clear all, close all, clc

syms q1 q2 q3;

shoulderLenght = 50.5; 
femurLenght = 71.5; 
tibiaLenght = 160; 

theta = [q1,q2,q3];
d = [0,0,0];
a = [shoulderLenght,femurLenght,tibiaLenght];
alpha = [pi/2,0,0];

T = eye(4);
for i = 1:3
        T = T * [cos(theta(i)),-sin(theta(i))*cos(alpha(i)),sin(theta(i))*sin(alpha(i)),a(i)*cos(theta(i));
                 sin(theta(i)),cos(theta(i))*cos(alpha(i)),-cos(theta(i))*sin(alpha(i)),a(i)*sin(theta(i));
                 0,sin(alpha(i)),cos(alpha(i)),d(i);
                 0,0,0,1];
end

J = jacobian([T(1,4),T(2,4),T(3,4)],theta);