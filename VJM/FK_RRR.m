function [T, R1, R2, R3] = FK_RRR(Tbase, Ttool, q, theta, link_length)
%FK_RPP Summary of this function goes here
%% Extracting the angles and thetas

q1 = q(1);
q2 = q(2);
q3 = q(3);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);





L1 = link_length(1);
L2 = link_length(2);
L3 = link_length(3);

%% Transformations

T1_2 = Tbase * Rz(q1) * Rz(theta1) * Tz(L1) ;

T2_3 = T1_2 * Ry(q2) * Ry(theta2) * Tx(L2) ;

R1 = T1_2(1:3,1:3);

R2 = T2_3(1:3,1:3);
T= T2_3 * Ry(q3) * Ry(theta3) * Tx(L3)* Ttool ; 

R3  = T(1:3,1:3);
end