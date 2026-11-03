clear all; close all; clc;

%% 1) Load the data
load('data.mat');
X = z.x(:);
Y = z.y(:);

%% 2) Build the design matrix M
% Each row: [x_i^2, x_i*y_i, y_i^2, x_i, y_i, 1]
n = length(X);
M = [X.^2, X.*Y, Y.^2, X, Y, ones(n,1)];

%% 3) Split M into M' and f, then solve in simple LS
Mprime = M(:,1:5);   
f      = M(:,6);

alpha = -(Mprime'*Mprime)\(Mprime'*f);

A = alpha(1); 
B = alpha(2); 
C = alpha(3); 
D = alpha(4); 
E = alpha(5); 
F_ = 1;         % fixed

%% 4) Print the results
fprintf('Estimated conic parameters:\n');
fprintf('A = %.6f\n', A);
fprintf('B = %.6f\n', B);
fprintf('C = %.6f\n', C);
fprintf('D = %.6f\n', D);
fprintf('E = %.6f\n', E);
fprintf('F = %.6f\n', F_);

%% 5) Plot the data and the fitted conic
figure; 
plot(X, Y, 'bo');
hold on;
title('Data + Fitted Conic (Simple LS)');
axis equal;

% Make a grid for contour
xMin = min(X) - 1;  xMax = max(X) + 1;
yMin = min(Y) - 1;  yMax = max(Y) + 1;
[xgrid, ygrid] = meshgrid(linspace(xMin, xMax, 200), linspace(yMin, yMax, 200));
Z = A*xgrid.^2 + B*xgrid.*ygrid + C*ygrid.^2 + D*xgrid + E*ygrid + F_;

% Draw the zero contour => the conic
contour(xgrid, ygrid, Z, [0 0], 'r', 'LineWidth', 2);
legend('Data points','Fitted conic');
