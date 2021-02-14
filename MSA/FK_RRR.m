function [T, R1, R2, R3] = FK_RRR(Tbase, Ttool, q, theta, link_length)
%FK_RPP Summary of this function goes here
%% Extracting the angles and thetas

q1 = q(1);
q2 = q(2);
q3 = q(3);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
theta4 = theta(4);
theta5 = theta(5);
theta6 = theta(6);
theta7 = theta(7);
theta8 = theta(8);
theta9 = theta(9);
theta10 = theta(10);
theta11 = theta(11);
theta12 = theta(12);
theta13 = theta(13);
theta14 = theta(14);
theta15 = theta(15);



L1 = link_length(1);
L2 = link_length(2);
L3 = link_length(3);

%% Transformations

T1_2 = Tbase * Rz(q1) * Rz(theta1) * Tz(L1) * Tx(theta2) * Ty(theta3) * Tz(theta4) * ...
    Rx(theta5) * Ry(theta6) * Rz(theta7);

T2_3 = T1_2 * Ry(q2) * Ry(theta8) * Tx(L2)* Tx(theta9) * Ty(theta10) * Tz(theta11) * ...
    Rx(theta12) * Ry(theta13) * Rz(theta14);

R1 = T1_2(1:3,1:3);

R2 = T2_3(1:3,1:3);
T= T2_3 * Ry(q3) * Ry(theta15) * Tx(L3)* Ttool ; 

R3  = T(1:3,1:3);
end