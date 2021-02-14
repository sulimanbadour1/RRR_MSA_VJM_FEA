function [Jt] = Jt_RRR(Tbase, Ttool, q, theta, link_length)
%JQ_RPP Summary of this function goes here
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

%% Extract the inverse fk rotation

Trot = FK_RRR(Tbase, Ttool, q, theta, link_length);

Trot(1:3,4) = 0;

%% Transformations


Td = Tbase * Rz(q1) * Rzd(theta1) * Tz(L1)* Ry(q2) * ...
    Ry(theta2) * Tx(L2)* Ry(q3) * Ry(theta3) * Tx(L3)* Ttool / Trot;

J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';


Td = Tbase * Rz(q1) * Rz(theta1) * Tz(L1)* Ry(q2) * ...
    Ryd(theta2) * Tx(L2)* Ry(q3) * Ry(theta3) * Tx(L3)* Ttool / Trot;


J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

Td = Tbase * Rz(q1) * Rz(theta1) * Tz(L1)* Ry(q2) * ...
    Ry(theta2) * Tx(L2)* Ry(q3) * Ryd(theta3) * Tx(L3)* Ttool / Trot;

J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';


Jt = [J1 J2 J3];
end


