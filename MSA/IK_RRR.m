function [q] = IK_RRR(p_global, link_length)
%IK_RPP Summary of this function goes here

%% Extracting the link lengths and local points

L1 = link_length(1);
L2 = link_length(2);
L3 = link_length(3);
x = p_global(1);
y = p_global(2);
z = p_global(3);

m= 1;%+1 for elbow up or -1 for elbow down;

for i = 1:length(x)

    b =  L1-z(i) ; 
    a = sqrt(y^2+x^2);
    q1 = atan2(y,x);
    q2 = atan2(b,a) - m * acos((a^2 + b^2 + L2^2 - L3^2)/(2*L2*sqrt(a^2 + b^2)));
    q3 = acos( ( a^2 + b^2 - L2^2 - L3^2)/ (2 * L2 * L3) );

end
q=[q1 q2 q3];
% 
% 
% 
% % For getting q2 and q3
% a = sqrt( x^2 + y^2); 
% p = L1 - z;
% 
% %% extract q1
% q1 = atan2(y,x);
% 
% %% extract q2 $ q3
% q2 = atan2(b,a) - m * acos((a^2 + b^2 + L2^2 - L3^2)/(2*L2*sqrt(a^2 + b^2)));
% q3 = acos( ( X^2 + Y^2 - L2^2 - L3^2)/ (2 * L2 * L3) );
% s =L3 * sin(q3);
% h=(L2 + L3 * cos(q3));
% q2 = -atan2( s,h ) + q1;
% 
% q = [q1 q2 q3];


% q1(:,i) = atan2(py(i),px(i));
% q2(:,i) = atan2(b,a) - m * acos((a^2 + b^2 + L2^2 - L3^2)/(2*L2*sqrt(a^2 + b^2)));
% q3(:,i) = acos( ( a^2 + b^2 - L2^2 - L3^2)/ (2 * L2 * L3) );
% 
% end
%  q=[q1; q2 ;q3]';
end
