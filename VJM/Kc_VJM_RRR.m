function [Kc] = Kc_VJM_RRR(Jt)
%KC_VJM_RPP Summary of this function goes here
%   Detailed explanation goes here
d = 0.15; % Cylinder diameter
k0 = 1000000; % actuator stiffness
E = 7.0000e+10; % Young's modulus
G = 2.5500e+10; % shear modulus
link_length = [0.2 0.2 0.2]; %Link length

%for cylinder
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;

% Let's get the stiffness matrixes of the robot  
[k11, k12, k21, k22] = k_cylinder(E, G, d, link_length(1), S, Iy, Iz);

% K theta matrix
Kt = [k0 zeros(1,2)
     
     zeros(1,1) k0 zeros(1,1)
      
     zeros(1,2) k0];
 
% Numerical solution
% SM=inv([(Jt/Kt)*Jt' Jq;  Jq' zeros(3,3)]);
% Kc=SM(1:6,1:6);

Kc = inv((Jt/Kt)*Jt');
%Kc = pinv((Jt/Kt)*Jt');
end

