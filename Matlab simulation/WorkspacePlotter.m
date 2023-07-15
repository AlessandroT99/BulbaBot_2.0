% Plotter of the spatial workspace of an antropomorphic manipulator

clear all, close all, clc;

%% Legs parameters
shoulderLenght = 50.5; %mm
femurLenght = 71.5; %mm
tibiaLenght = 160; %mm

MIN_SHOULDER_ANGLE = -90;
MAX_SHOULDER_ANGLE = 90;
MIN_FEMUR_ANGLE = -20;
MAX_FEMUR_ANGLE = 90;
MIN_TIBIA_ANGLE = -90;
MAX_TIBIA_ANGLE = 90;

%% Points evaluation for 3D graph
outputDim = 20; %Number of points of each line of each axis

q1 = linspace(MIN_SHOULDER_ANGLE,MAX_SHOULDER_ANGLE,2*outputDim)*pi/180;
q2 = linspace(MIN_FEMUR_ANGLE,MAX_FEMUR_ANGLE,outputDim)*pi/180;
q3 = linspace(MIN_TIBIA_ANGLE,MAX_TIBIA_ANGLE,outputDim)*pi/180;
theta = {q1,q2,q3};
d = [0,0,0];
a = [shoulderLenght,femurLenght,tibiaLenght];
alpha = [pi/2,0,0];

[x,y,z] = DHmatrixGenerator(theta,d,a,alpha,outputDim);


figure, plot3(x(:),y(:),z(:),'b.'), title("3D Workspace")
axis equal, grid on
xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]'), grid on

%% Points evaluation for XY 2D graphs
figure(2)
subplot(1,2,2), plot(x(:),y(:),'r.'), title("XY Workspace")
axis equal, grid on
xlabel('X [mm]'), ylabel('Y [mm]'), grid on

%% Points evaluation for XZ 2D graphs
outputDim = 50;
q2 = linspace(MIN_FEMUR_ANGLE,MAX_FEMUR_ANGLE,outputDim)*pi/180;
q3 = linspace(MIN_TIBIA_ANGLE,MAX_TIBIA_ANGLE,outputDim)*pi/180;
theta = {0,q2,q3};
[x,y,z] = DHmatrixGenerator(theta,d,a,alpha,outputDim);

figure(2) 
subplot(1,2,1), plot(x(:),z(:),'r.'), title("XZ Workspace")
axis equal
xlabel('X [mm]'), ylabel('Z [mm]'), grid on

