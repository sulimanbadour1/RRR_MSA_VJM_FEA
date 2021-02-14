function [Kc] = Kc_def_RRR(p_global, theta)

%KC_DEF_RPP Summary of this function goes here
link_length = [0.2 0.2 0.2]; %Link length
%% Transformation of base and tool
Tbase = eye(4);
Ttool = eye(4);

%% Get the inverse kinematics
theta = zeros(1,3);
q = IK_RRR(p_global, link_length);

Jt = Jt_RRR(Tbase, Ttool, q, theta, link_length);

Kc = Kc_VJM_RRR(Jt);

end

