clc
clear all
close all
syms b1 theta2 theta3 b4
syms b1_dot theta2_dot theta3_dot b4_dot b1_ddot theta2_ddot theta3_ddot b4_ddot

%% Physical Data
 
theta = [ b1 ; theta2 ; theta3 ; b4 ];
theta_dot = [ b1_dot ; theta2_dot; theta3_dot ; b4_dot];
theta_ddot = [ b1_ddot ; theta2_ddot; theta3_ddot ; b4_ddot];

theta1=0;
theta4=0;
theta1_0=0;
theta4_0=0;
b1_0=0;
theta2_0 = 0;
theta3_0 = 0;
b4_0 = 0;

% Link0
mass_link0 = 0.8;
com_link0_ros = [0.0000000e+00 0.0000000e+00 0.0000000e+00]';
I0_ros = [0.44 , 0.0           , 0.0
            0.0         ,  0.003, 0.0
         0.0 ,  0.0          , 0.44];
% Link1
mass_link1 = 0.8;
com_link1_ros = [0.15 0.08 0.1]';
I1_ros = [2000 , 2.0           , 160
            2.0         ,  3000, 1.0
         160 ,  1.0          , 2000].*(10^(-6));
% Link2
mass_link2 = 1.2;
com_link2_ros = [0.35 0.1 0.25]';
I2_ros = [1600 , -0.25, -1200
     -0.25,  30000, 0.05
     -1200,  0.05, 30000].*(10^(-6));
% Link3
mass_link3 = 0.85;
com_link3_ros = [0.45 0.1 0.3]';
I3_ros = [6000, -0.3, 1300
      -0.3, 15000, -0.3
     1300, -0.3, 12000].*(10^(-6));
% Link4
mass_link4 = 0.15;
com_link4_ros = [0.65 0.02 0.4]';
I4_ros = [3000, 0.07, 0.04
         0.07,  3000,  0.1
         0.04,  0.1,  20].*(10^(-6));

%% D-H Form
% T matrices
T0 = [round(Qx(-pi/2)), [0;0;73.5]/1000; [0,0,0,1]] * [eye(3), [0;0;0.08]; [0,0,0,1]];
T = T0 * [round(Qx(pi/2)), [-0.2;0;0]; [0,0,0,1]];

% Link 1
com1_dh = T * [com_link1_ros; 1];
com1_dh = com1_dh(1:3);
Q = T(1:3, 1:3);
b = T(1:3, 4);
I1_dh = Q * (I1_ros + mass_link1 * ((b'*b) * eye(3) - b * b')) / Q;

% Link 2
T = T * [eye(3), [0.2;0.08;0.078]; [0,0,0,1]] * [eye(3), [-0.073;-0.083;-0.079]; [0,0,0,1]];
com2_dh = T * [com_link2_ros; 1];
com2_dh = com2_dh(1:3);
Q = T(1:3, 1:3);
I2_dh = Q * (I2_ros + mass_link2 * ((b'*b) * eye(3) - b * b')) / Q;

% Link 3
T = T * [eye(3), [0.47;0.09;0.2]; [0,0,0,1]] * [eye(3), [-0.4;-0.09;-0.2]; [0,0,0,1]];
com3_dh = T * [com_link3_ros; 1];
com3_dh = com3_dh(1:3);
b = T(1:3, 4);
Q = T(1:3, 1:3);
I3_dh = Q * (I3_ros + mass_link3 * ((b'*b) * eye(3) - b * b')) / Q;

% Link 4
T = T * [eye(3), [0.65;0.07;0.28]; [0,0,0,1]] * [eye(3), [-0.65;0;-0.28]; [0,0,0,1]];
com4_dh = T * [com_link4_ros; 1];
com4_dh = com4_dh(1:3);
b = T(1:3, 4);
Q = T(1:3, 1:3);
I4_dh = Q * (I4_ros + mass_link4 * ((b'*b) * eye(3) - b * b')) / Q;

%% First dot ( Write W and N vectors)
Q0 = Qx(-pi/2);
Q1 = [1,0,1; 0,0,1; 0,-1,0];
Q2 = [cos(theta2),-sin(theta2),0; sin(theta2),cos(theta2),0; 0,0,1];
Q3 = [cos(theta3),-sin(theta3),0; sin(theta3),cos(theta3),0; 0,0,1];
Q4 = [1,0,0; 0,1,0; 0,0,1];

zero = [0;0;0];
z = [0;0;1];
e1 = Q0*z;
e2 = [0;0;1];
e3 = e2;
e4 = e2;

av1 = [0;0;b1];
av2 = [0.3971*cos(theta2);0.3971*sin(theta2);0.121];
av3 = [0.1254*cos(theta3);0.1254*sin(theta3);0.08];
av4 = [0;0;b4];

r11 = Q0 * (com1_dh + [0;0;b1]);
r12 = Q0 * (av1 + Q1 * Qz(theta2 - theta2_0) * com2_dh);
r22 = Q0 * Q1 * Qz(theta2 - theta2_0) * com2_dh;
r13 = Q0 * (av1 + Q1 * av2 + Q1 * Q2 * Qz(theta3 - theta3_0) * com3_dh);
r23 = Q0 * (Q1 * av2 + Q1 * Q2 * Qz(theta3 - theta3_0) * com3_dh);
r33 = Q0 * (Q1 * Q2 * Qz(theta3 - theta3_0) * com3_dh);
r14 = Q0 * (av1 + Q1 * av2 + Q1 * Q2 * av3 + Q1 * Q2 * Q3 * (com4_dh + [0;0;b4]));
r24 = Q0 * (Q1 * av2 + Q1 * Q2 * av3 + Q1 * Q2 * Q3 * (com4_dh + [0;0;b4]));
r34 = Q0 * (Q1 * Q2 * av3 + Q1 * Q2 * Q3 * (com4_dh + [0;0;b4]));
r44 = Q0 * (Q1 * Q2 * Q3 * (com4_dh + [0;0;b4]));

% Ni in world frame
N1 = [e1 zero zero zero];
N2 = [e1 cross(e2, r22) zero zero];
N3 = [e1 cross(e2, r23) cross(e3, r33) zero];
N4 = [e1 cross(e2, r24) cross(e3, r34) e4];

% Wi in i th frame
W1 = [zero zero zero zero];
W2 = [zero z zero zero];
W3 = [zero Q2'*z z zero];
W4 = [zero Q3'*Q2'*z Q3'*z zero];

%% Final calculation ( Energy eq. - Use the paper )
M1 = mass_link1 * N1' * N1 + W1' * I1_dh * W1;
M2 = mass_link2 * N2' * N2 + W2' * Qz(theta2 - theta2_0) * I2_dh * Qz(theta2 - theta2_0)' * W2;
M3 = mass_link3 * N3' * N3 + W3' * Qz(theta3 - theta3_0) * I3_dh * Qz(theta3 - theta3_0)' * W3;
M4 = mass_link4 * N4' * N4 + W4' * I4_dh * W4;
M = M1 + M2 + M3 + M4;
T = 0.5 * theta_dot' * M * theta_dot;

%% Euler-Lagrange equations (jacobian etc.)
h1 = (Q0 * (com1_dh + [0;0;b1]))' * z;
h2 = (Q0 * (av1 + Q1 * Qz(theta2 - theta2_0) * com2_dh))' * z;
h3 = (Q0 * (av1 + Q1 * av2 + Q1 * Q2 * Qz(theta3 - theta3_0) * com3_dh))' * z;
h4 = (Q0 * (av1 + Q1 * av2 + Q1 * Q2 * av3 + Q1 * Q2 * Q3 * (com4_dh + [0;0;b4])))' * z;

g = 9.81;
v1 = mass_link1 * g * h1;
v2 = mass_link2 * g * h2;
v3 = mass_link3 * g * h3;
v4 = mass_link4 * g * h4;
V = v1 + v2 + v3 + v4;
M_dot = M_dot_generator(M, theta, theta_dot);

n = M_dot * theta_dot - jacobian(T, theta)' + jacobian(V, theta)';

tav = (M * theta_ddot + n);

%% Matlab Function
ForceandTorques = [b1; theta2; theta3; b4];
ForceandTorques = simplify(ForceandTorques);
matlabFunction(ForceandTorques, 'File', 'OpenMan_torquesandforce');
