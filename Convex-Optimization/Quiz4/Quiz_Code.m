% Define variables
x = sdpvar(2,1);      % Decision variable x ∈ R^2
t = sdpvar(1);        % Auxiliary variable for L2 norm
u = sdpvar(2,1);      % Auxiliary variable for L1 norm (|x| ≤ u)

% Given data
A = [1 1; 2 1; 3 2];
b = [2; 3; 4];
gamma = 0.5;

% Constraints
Constraints = [];
Constraints = [Constraints, norm(A*x - b, 2) <= t];
Constraints = [Constraints, -u <= x <= u];
Constraints = [Constraints, u >= 0];

% Objective
Objective = t + gamma * sum(u);

% Solve
options = sdpsettings('solver', '', 'verbose', 1);
optimize(Constraints, Objective, options);

x_opt = value(x);
f_opt = value(Objective);

disp(['Optimal x = ', mat2str(x_opt,4)]);
disp(['Optimal cost = ', num2str(f_opt,4)]);

% Create a grid of x1 and x2
[x1_grid, x2_grid] = meshgrid(-1:0.05:3, -1:0.05:3);
Z = zeros(size(x1_grid));

% Evaluate cost function over grid
for i = 1:size(x1_grid, 1)
    for j = 1:size(x1_grid, 2)
        x_sample = [x1_grid(i,j); x2_grid(i,j)];
        term1 = norm(A*x_sample - b, 2);
        term2 = gamma * norm(x_sample, 1);
        Z(i,j) = term1 + term2;
    end
end

% Plot contour
figure;
contourf(x1_grid, x2_grid, Z, 50, 'LineColor','none');
hold on;
plot(x_opt(1), x_opt(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
colorbar;
xlabel('x_1');
ylabel('x_2');
title('Cost Function Contours: f(x) = ||Ax - b||_2 + \gamma||x||_1');
legend('Cost contours', 'Optimal point');
grid on;
