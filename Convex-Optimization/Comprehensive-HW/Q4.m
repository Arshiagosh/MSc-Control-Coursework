clc
clear
close all
% Parameters
A = [2 -1; -1 3];
B = [1 0; 0 2];
C = [1 0; 0 1];
gamma = 1.5;
delta = 5;


cvx_begin sdp
    variable X(2,2) symmetric;  
    minimize(trace(A'*X));     
    subject to
        trace(B'*X*B) >= gamma; 
        trace(C*X) <= delta;    
        X >= 0;                
cvx_end

disp('Optimal X:');
