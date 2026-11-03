A = [0.7776, -0.0045; 26.6185, 1.8555];
B = [-0.0004; 0.2907];
I = eye(2);

% Decision variables
x = sdpvar(2,1);
u = sdpvar(1,1);

% Binary variables for piecewise region selection
d = binvar(3,1); % d1: region1, d2: region2, d3: region3

% Enforce that only one region is active
constraints = sum(d) == 1;

% Region constraints
M = 1e3; % Big-M constant

% Region 1: x2 <= -2
constraints = [constraints, x(2) <= -2 + M*(1 - d(1))];

% Region 2: -2 <= x2 <= 2
constraints = [constraints, x(2) >= -2 - M*(1 - d(2)), x(2) <= 2 + M*(1 - d(2))];

% Region 3: x2 >= 2
constraints = [constraints, x(2) >= 2 - M*(1 - d(3))];

% Steady-state equation
steady_state_eq = (I - A)*x - B*u == 0;
constraints = [constraints, steady_state_eq];

% System constraints
constraints = [constraints;
    abs(u) <= 15;
    abs(x(1)) <= 0.5;
    abs(x(2)) <= 5];

% Piecewise cost
cost1 = 10*x(2)^2 + 40*x(2) + 40;
cost2 = 0;
cost3 = 10*x(2)^2 - 40*x(2) + 40;

cost = x(1) + d(1)*cost1 + d(2)*cost2 + d(3)*cost3;

% Solve
options = sdpsettings('solver', 'bnb', 'verbose', 0); % 'bnb' is YALMIP's branch and bound solver
optimize(constraints, cost, options);

% Extract results
x_opt = value(x);
u_opt = value(u);
cost_opt = value(cost);

fprintf('Optimal Set Point:\n');
fprintf('x1 = %.4f mol/L\n', x_opt(1));
fprintf('x2 = %.4f K\n', x_opt(2));
fprintf('u = %.4f K\n', u_opt);
fprintf('Minimum Cost = %.4f\n', cost_opt);
