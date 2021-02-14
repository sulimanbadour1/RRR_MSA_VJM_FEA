function [lamdaR, lamdaP] = lambda(q)
%LAMBDA_R Summary of this function goes here
%   returns lambdas r
    I = eye(6);
    lamdaP = I(q,:);
    lamdaR = [I(1:q-1,:); I(q+1: end, :)];
            
end

