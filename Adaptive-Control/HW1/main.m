%% Adaptive Control Systems - Homework 1
% Parameter Estimation using Least Squares Estimation (LSE)
% Student: Arshia Goshtasbi
% This script implements parameter estimation for models of different orders

%% Initialize and Clear
clear all; close all; clc;

%% Input Data
% Seven input values as specified in Figure 1-2
u = [0; 1; 2; 3; 4; 5; 6];

% Corresponding output measurements
y = [1; 2; 2.2; 3.5; 4.6; 6.1; 7.8];
n = length(y);

%% Model 1: Zero-th Order Model
% Model: y = b0
% This represents a constant output (mean of y)

fprintf('\n========================================\n');
fprintf('MODEL 1: Zero-th Order (y = b0)\n');
fprintf('========================================\n');

% Analytical solution: b0 = mean(y)
b0_model1 = mean(y);
y_pred_model1 = b0_model1 * ones(size(y));

% Calculate Mean Squared Error
MSE_model1 = 1/2 * sum((y - y_pred_model1).^2);

fprintf('Estimated Parameter: b0 = %.4f\n', b0_model1);
fprintf('Mean Squared Error: %.4f\n', MSE_model1);
fprintf('Book Reference Value: 3.85\n');

% Plot Results
figure('Name', 'Model 1: Zero-th Order', 'NumberTitle', 'off');
plot(u, y, 'bo-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Actual Output');
hold on;
plot(u, y_pred_model1, 'r--', 'LineWidth', 2, 'DisplayName', 'Model Prediction');
xlabel('Input (u)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output (y)', 'FontSize', 12, 'FontWeight', 'bold');
title('Model 1: Zero-th Order Approximation', 'FontSize', 14);
legend('Location', 'best');
grid on;
hold off;

%% Model 2: First Order Model
% Model: y(i) = b0 + b1*u(i)

fprintf('\n========================================\n');
fprintf('MODEL 2: First Order (y = b0 + b1*u)\n');
fprintf('========================================\n');

% Design matrix
Phi_model2 = [ones(n, 1), u];

% LSE Solution: theta = (Phi'*Phi)^(-1) * Phi'*y
theta_model2 = pinv(Phi_model2' * Phi_model2) * (Phi_model2' * y);
b0_model2 = theta_model2(1);
b1_model2 = theta_model2(2);

% Prediction
y_pred_model2 = Phi_model2 * theta_model2;

% Calculate Mean Squared Error
MSE_model2 = 1/2 * sum((y - y_pred_model2).^2);

fprintf('Estimated Parameters:\n');
fprintf('  b0 = %.4f\n', b0_model2);
fprintf('  b1 = %.4f\n', b1_model2);
fprintf('Mean Squared Error: %.4f\n', MSE_model2);

% Plot Results
figure('Name', 'Model 2: First Order', 'NumberTitle', 'off');
plot(u, y, 'bo-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Actual Output');
hold on;
plot(u, y_pred_model2, 'r--', 'LineWidth', 2, 'DisplayName', 'Model Prediction');
xlabel('Input (u)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output (y)', 'FontSize', 12, 'FontWeight', 'bold');
title('Model 2: First Order Linear Fit', 'FontSize', 14);
legend('Location', 'best');
grid on;
hold off;

%% Model 3: Second Order Model
% Model: y(i) = b0 + b1*u(i) + b2*u(i)^2

fprintf('\n========================================\n');
fprintf('MODEL 3: Second Order (y = b0 + b1*u + b2*u²)\n');
fprintf('========================================\n');

% Design matrix
Phi_model3 = [ones(n, 1), u, u.^2];

% LSE Solution
theta_model3 = pinv(Phi_model3' * Phi_model3) * (Phi_model3' * y);
b0_model3 = theta_model3(1);
b1_model3 = theta_model3(2);
b2_model3 = theta_model3(3);

% Prediction
y_pred_model3 = Phi_model3 * theta_model3;

% Calculate Mean Squared Error
MSE_model3 = 1/2 * sum((y - y_pred_model3).^2);

fprintf('Estimated Parameters:\n');
fprintf('  b0 = %.4f\n', b0_model3);
fprintf('  b1 = %.4f\n', b1_model3);
fprintf('  b2 = %.4f\n', b2_model3);
fprintf('Mean Squared Error: %.4f\n', MSE_model3);

% Plot Results
figure('Name', 'Model 3: Second Order', 'NumberTitle', 'off');
plot(u, y, 'bo-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Actual Output');
hold on;
plot(u, y_pred_model3, 'r--', 'LineWidth', 2, 'DisplayName', 'Model Prediction');
xlabel('Input (u)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output (y)', 'FontSize', 12, 'FontWeight', 'bold');
title('Model 3: Second Order Polynomial Fit', 'FontSize', 14);
legend('Location', 'best');
grid on;
hold off;

%% Model 4: Third Order Model
% Model: y(i) = b0 + b1*u(i) + b2*u(i)^2 + b3*u(i)^3

fprintf('\n========================================\n');
fprintf('MODEL 4: Third Order (y = b0 + b1*u + b2*u² + b3*u³)\n');
fprintf('========================================\n');

% Design matrix
Phi_model4 = [ones(n, 1), u, u.^2, u.^3];

% LSE Solution
theta_model4 = pinv(Phi_model4' * Phi_model4) * (Phi_model4' * y);
b0_model4 = theta_model4(1);
b1_model4 = theta_model4(2);
b2_model4 = theta_model4(3);
b3_model4 = theta_model4(4);

% Prediction
y_pred_model4 = Phi_model4 * theta_model4;

% Calculate Mean Squared Error
MSE_model4 = 1/2 * sum((y - y_pred_model4).^2);

fprintf('Estimated Parameters:\n');
fprintf('  b0 = %.4f\n', b0_model4);
fprintf('  b1 = %.4f\n', b1_model4);
fprintf('  b2 = %.4f\n', b2_model4);
fprintf('  b3 = %.4f\n', b3_model4);
fprintf('Mean Squared Error: %.4f\n', MSE_model4);

% Plot Results
figure('Name', 'Model 4: Third Order', 'NumberTitle', 'off');
plot(u, y, 'bo-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Actual Output');
hold on;
plot(u, y_pred_model4, 'r--', 'LineWidth', 2, 'DisplayName', 'Model Prediction');
xlabel('Input (u)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output (y)', 'FontSize', 12, 'FontWeight', 'bold');
title('Model 4: Third Order Polynomial Fit', 'FontSize', 14);
legend('Location', 'best');
grid on;
hold off;

%% Comparison of All Models
fprintf('\n========================================\n');
fprintf('MODEL COMPARISON\n');
fprintf('========================================\n');
fprintf('Model Order  |  MSE\n');
fprintf('-------------|----------\n');
fprintf('0th Order    |  %.4f\n', MSE_model1);
fprintf('1st Order    |  %.4f\n', MSE_model2);
fprintf('2nd Order    |  %.4f\n', MSE_model3);
fprintf('3rd Order    |  %.4f\n', MSE_model4);

% Comprehensive Comparison Plot
figure('Name', 'Model Comparison', 'NumberTitle', 'off');
plot(u, y, 'ko', 'LineWidth', 2.5, 'MarkerSize', 10, ...
     'MarkerFaceColor', 'k', 'DisplayName', 'Actual Data');
hold on;
plot(u, y_pred_model1, 's--', 'LineWidth', 1.5, 'MarkerSize', 6, ...
     'DisplayName', 'Model 1 (Order 0)');
plot(u, y_pred_model2, 'd--', 'LineWidth', 1.5, 'MarkerSize', 6, ...
     'DisplayName', 'Model 2 (Order 1)');
plot(u, y_pred_model3, '^--', 'LineWidth', 1.5, 'MarkerSize', 6, ...
     'DisplayName', 'Model 3 (Order 2)');
plot(u, y_pred_model4, 'v--', 'LineWidth', 1.5, 'MarkerSize', 6, ...
     'DisplayName', 'Model 4 (Order 3)');
xlabel('Input (u)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output (y)', 'FontSize', 12, 'FontWeight', 'bold');
title('Comparison of All Model Orders', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
hold off;
