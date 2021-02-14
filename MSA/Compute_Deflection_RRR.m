function [deltaT, Kc] = Compute_Deflection_RRR(p_global, Force)
%COMPUTE_DEFLECTION_RPP Summary of this function goes here

%% Constant stuff
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

%% Transformation of the base & tool
Tbase = eye(4);

Ttool = eye(4);

%% Extract the global position

x = p_global(1);
y = p_global(2);
z = p_global(3);

theta = zeros(1,16);
%% Get the inverse kinematics and forward kinematics

q = IK_RRR( p_global, link_length);

[T, R1, R2 , R3] = FK_RRR(Tbase, Ttool, q, theta, link_length);

%% Transforming the q from local frame to the global frame

Q1 = [R1, zeros(3,3);
     zeros(3,3), R1];

Q2 = [R2, zeros(3,3);
     zeros(3,3), R2];
 
Q3 = [R3, zeros(3,3);
     zeros(3,3), R3];

%% Global stiffness matrices for the 1st link 

K11_Link1 = Q1*k11*Q1';
K12_Link1 = Q1*k12*Q1';
K21_Link1 = Q1*k21'*Q1';
K22_Link1 = Q1*k22*Q1';


%% Global stiffness matrices for the 2nd link 

K11_Link2 = Q2*k11*Q2';
K12_Link2 = Q2*k12*Q2';
K21_Link2 = Q2*k21'*Q2';
K22_Link2 = Q2*k22*Q2';

%% Global stiffness matrices for the 2nd link 

K11_Link3 = Q3*k11*Q3';
K12_Link3 = Q3*k12*Q3';
K21_Link3 = Q3*k21'*Q3';
K22_Link3 = Q3*k22*Q3';
%% Calculate Lamdas
[lambdaR1, lambdaE1] = lambda(6);
[lambdaR2, lambdaE2] = lambda(5);
[lambdaR3, lambdaE3] = lambda(5);

Full_Matrix = [ Eq_maker('rigid_base', 1, 6);
    Eq_maker('elastic_joint', 1, 6, lambdaR1, lambdaE1, k0);
 %   Eq_maker('flexible_link', 2, 6, K11_Link1, K12_Link1, K21_Link1, K22_Link1);
    Eq_maker('rigid_link', 2, 6, [0 0 1]);
    Eq_maker('elastic_joint', 3, 6, lambdaR2, lambdaE2, k0);
  %  Eq_maker('flexible_link', 4, 6, K11_Link2, K12_Link2, K21_Link2, K22_Link2);
    Eq_maker('rigid_link', 4, 6, [1 0 0]);
    Eq_maker('elastic_joint', 5, 6, lambdaR3, lambdaE3,k0);
   % Eq_maker('rigid_link', 6, 6 ,[1 0 0]);
    Eq_maker('flexible_link', 6, 6, K11_Link3, K12_Link3, K21_Link3, K22_Link3);
    Eq_maker('external_force', 6, 6)];

A = Full_Matrix(1:78, 1:78);

B = Full_Matrix(1:78, 79:84);

C = Full_Matrix(79:84, 1:78);

D = Full_Matrix(79:84, 79:84);

Kc =  D - C*(A\B);

%% deflection computation
deltaT = Kc \ Force; 

end
