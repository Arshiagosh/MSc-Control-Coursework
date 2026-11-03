clear all; close all; clc;

A = [-3, 0, 4, 0;
      4, -5, 2, 6;
     -3, 7, -2, 4;
      0, 3, 5, 7];
  
B1 = [0, -1;
     -2, 5;
      0, 4;
      7, -3];
  
B2 = [6; -3; 7; 2];

C1 = [7, -6, 4, -1;
     -3, 0, 5, 0];
 
C2 = [1, 0, 0, 0];

D11 = zeros(2,2);
D12 = [0; 1];
D21 = [1, 0];
D22 = 0;

X = sdpvar(4,4, 'symmetric'); 
Y = sdpvar(1,4);               
gamma = sdpvar(1);            

M11 = A*X + X*A' + B2*Y + Y'*B2';

M12 = B1;

M13 = (C1*X + D12*Y)';

M21 = B1';

M22 = -gamma^2 * eye(2);

M23 = D11';

M31 = C1*X + D12*Y;

M32 = D11;

M33 = -eye(2);

LMI = [M11, M12, M13;
       M21, M22, M23;
       M31, M32, M33];

Constraints = [LMI < 0, X > 0, gamma > 0];

options = sdpsettings('solver', 'lmilab', 'verbose', 1);

sol = optimize(Constraints, gamma, options);

if sol.problem == 0
    gamma_opt = value(gamma);
    X_opt = value(X);
    Y_opt = value(Y);
    
    K_opt = Y_opt / X_opt;
    
    disp(['مقدار بهینه γ: ', num2str(gamma_opt)]);
    disp('ماتریس بهره کنترل‌کننده (K):');
    disp(K_opt);
    disp('ماتریس X:');
    disp(X_opt);
    
    A_cl = A + B2*K_opt;
    eig_cl = eig(A_cl);
    disp('مقادیر ویژه سیستم حلقه بسته:');
    disp(eig_cl);
    
    if all(real(eig_cl) < 0)
        disp('سیستم حلقه بسته پایدار است.');
    else
        disp('سیستم حلقه بسته ناپایدار است!');
    end
    
else
    disp('========== خطا در حل مسئله ==========');
    disp(['پیام خطا: ' sol.info]);
end