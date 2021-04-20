% Model Initialization
% ADT MPC Controller for Figure Eight Course
% Copyright 2020 The MathWorks, Inc.

%% add Images to the path
addpath(genpath('Images'));

%% define reference points
r = 75; % radius of circle
theta = 0:0.01:2*pi;
x1 = -r*cos(theta); % We wanted to create a skidpad track, hence we have used circle equations. 
%Alternatively, you can also use figure eight equation to generate the reference points.
y1 = r*sin(theta);
x2 = -2*r+r*cos(theta);
y2 = r*sin(theta);
xEight = [x1 x1 x2 x2];
yEight = [y1 y1 y2 y2];
% plot(xEight,yEight)
xRef = yEight;
yRef = xEight;

%% define vehicle parameters used in the models
L = 3; % bicycle length
ld = 4; % lookahead distance
X_o = xRef(1); % initial vehicle position
Y_o = yRef(1); % initial vehicle position 
psi_o = 0; % it's an important step to initialize yaw angle

%% calculating reference pose vectors for figure eight course

% Based on how far the vehicle travels, the pose is generated using 1-D
% lookup tables.

% calculate distance vector
rr = [yEight;xEight;xEight*0]';
distancematrix = squareform(pdist(rr));
distancesteps = zeros(length(rr)-1,1);
for i = 2:length(rr)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,2500); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp,'pchip');
yRef2 = interp1(distbp,yRef,gradbp,'pchip');
yRef2s = smooth(gradbp,yRef2);
xRef2s = smooth(gradbp,xRef2);

% calculate curvature vector
curvature = getCurvature(xRef2,yRef2);

%% Calculate A,B,C matrices used in MPC Model
% vehicle parameters
tau = 0.5;
Vx = 10;
m = 2000;
Iz = 4000;
Lf = 1.4;
Lr = 1.6;
Cf = 12e3;
Cr = 11e3;
% longitudinal model
A1 = [-1/tau 0; 1 0];
B1 = [1/tau; 0];
C1 = [ 0 1];
D1 = 0;
% lateral model
A2 = [-2*(Cf+Cr)/m/Vx -Vx-2*(Cf*Lf-Cr*Lr)/m/Vx;...
    -2*(Cf*Lf-Cr*Lr)/Iz/Vx -2*(Cf*Lf^2+Cr*Lr^2)/Iz/Vx];
B2 = 2*Cf*[1/m; Lf/Iz];
C2 = [1 0; 0 1];
D2 = [0;0];
% combined model
A = [A1 zeros(2,2); zeros(2,2) A2];
B = [B1 zeros(2,1); zeros(2,1) B2];
C = [C1 zeros(1,2); zeros(2,2) C2];
D = [D1 zeros(1,1); zeros(2,1) D2];


%% MPC Pedal Map
% additional vehicle parameters
rho = 1.21;
Cd = 0.3;
Af = 2;
tire_r = 0.309;
% bounds for 2-D lookup table
accel_vec = (-4:0.5:4)'; % acceleration is between -4 and 4 m/s^2
vel_vec = 0:2:20; % vehicle velocity is between 0 and 20 m/s
torque_map = zeros(length(accel_vec),length(vel_vec));
% calculate required torque
for i = 1:length(accel_vec)
    for j = 1:length(vel_vec)
        % Torque is based on sum of the forces times the wheel radius
        % F_tractive = F_i + F_resistive
        % F_resistive forces are drag and a constant tire loss force
        % The constant is one of the values used to calibrate the map
        % For more information on the forces, see Gillespie's "Fundamentals
        % of Vehicle Dynamics"
        torque_map(i,j) = tire_r*((m*accel_vec(i))+(0.5*rho*Cd*Af*vel_vec(j)^2)+160);
    end
end
% convert torque to pedal based on powertrain parameters
pedal_map = torque_map;
% positive torques are scaled based on powertrain's maximum wheel torque
max_prop_torque = 425*9.5;
pedal_map(pedal_map>0) = pedal_map(pedal_map>0)/max_prop_torque;
% calculate the conversion from torque to maximum pressure
pressure_conv = (0.2*7.5e6*pi*0.05*0.05*.177*2/4)*4*1.01; % 1.01 is a calibrated value
pedal_map(pedal_map<0) = pedal_map(pedal_map<0)/pressure_conv;

%% Curvature Function

function curvature = getCurvature(xRef,yRef)
% Calculate gradient by the gradient of the X and Y vectors
DX = gradient(xRef);
D2X = gradient(DX);
DY = gradient(yRef);
D2Y = gradient(DY);
curvature = (DX.*D2Y - DY.*D2X) ./(DX.^2+DY.^2).^(3/2);
end
