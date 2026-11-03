A = [-1, 0, 0, 1;
      0, -1, 4, -3;
      1, -3, -1, -3;
      0, 4, 2, -1];
  
B = [0, 1;
     0, 0;
     -1, 0;
     0, 0];
  
C = [-1, 0, 1, 0;
      0, 1, 0, 1];
  
D = [0, 1;
     0, 0];

gamma = sdpvar(1);   
X = sdpvar(4,4);  
Y = sdpvar(1,4);    

M11 = A*X + X*A' + B(:,1)*Y + Y'*B(:,1)';
M12 = B(:,2);
M13 = (C*X + D(:,1)*Y)';

M21 = B(:,2)';
M22 = -gamma^2 * eye(1);
M23 = D(:,2)';

M31 = C*X + D(:,1)*Y;
M32 = D(:,2);
M33 = -eye(2);

LMI = [M11, M12, M13;
       M21, M22, M23;
       M31, M32, M33];

Constraints = [LMI < 0, X > 0, gamma > 0];

options = sdpsettings('solver', 'lmilab', 'verbose', 1);
optimize(Constraints, gamma, options);

disp(value(X));
disp(value(Y));
disp(value(Y*pinv(X));