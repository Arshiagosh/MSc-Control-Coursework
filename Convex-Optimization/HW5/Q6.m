alpha = sdpvar; 
Y = sdpvar(3, 3, 'symmetric');
L = sdpvar(1, 3, 'full');
A = [0 1 alpha; 0 0 1; 0 0 0];
B = [0; 0; 1];
Q = eye(3);
R = 1;
Constraints = [Y >= 0.001*eye(3)]; 
LMI = [A*Y + B*L + Y*A' + L'*B' + Q, L'; L, -inv(R)] <= -0.001*eye(4);
Constraints = [Constraints, LMI, -0.1 <= alpha <= 0.1];
Objective = trace(Y);
optimize(Constraints, Objective);
Y_opt = value(Y);
L_opt = value(L);
K_opt = L_opt / Y_opt