% Arshia Goshtasbi
% Optimal Control HW8

clear;
clc;
close all;

% --- Scenario 1: N = 2 (as defined in the primary example) ---
fprintf('--- Running Scenario 1: N=2, dx=0.01, du=0.01 ---\n');
N1 = 2;
dx1 = 0.5; 
du1 = 0.5;
x0_1 = 1.5; 

[X_traj1, U_traj1, J_total1] = solve_optimal_control_dp(N1, dx1, du1, x0_1);

fprintf('Total Optimal Cost (J) for N=2: %.4f\n', J_total1);
disp('Optimal State Trajectory x(k):');
disp(X_traj1);
disp('Optimal Control Sequence u(k):');
disp(U_traj1);
plot_results(N1, X_traj1, U_traj1, 'Scenario 1: N=2, dx=0.01, du=0.01');


% --- Scenario 2: N = 20, dx=0.1, du=0.1 (Coarse Grid) ---
fprintf('\n--- Running Scenario 2: N=20, dx=0.1, du=0.1 ---\n');
N2 = 20;
dx2 = 0.1;
du2 = 0.1;
x0_2 = 1.5; 

[X_traj2, U_traj2, J_total2] = solve_optimal_control_dp(N2, dx2, du2, x0_2);

fprintf('Total Optimal Cost (J) for N=20 (coarse grid): %.4f\n', J_total2);

plot_results(N2, X_traj2, U_traj2, 'Scenario 2: N=20, dx=0.1, du=0.1');


% --- Scenario 3: N = 20, dx=0.01, du=0.01 (Fine Grid) ---
fprintf('\n--- Running Scenario 3: N=20, dx=0.01, du=0.01 ---\n');
N3 = 20;
dx3 = 0.01;
du3 = 0.01;
x0_3 = 1.5;
[X_traj3, U_traj3, J_total3] = solve_optimal_control_dp(N3, dx3, du3, x0_3);
fprintf('Total Optimal Cost (J) for N=20 (fine grid): %.4f\n', J_total3);
plot_results(N3, X_traj3, U_traj3, 'Scenario 3: N=20, dx=0.01, du=0.01');

% --- Scenario 4: N = 200, dx=0.1, du=0.1 (Coarse Grid) ---
fprintf('\n--- Running Scenario 4: N=200, dx=0.1, du=0.1 ---\n');
N4 = 200;
dx4 = 0.1;
du4 = 0.1;
x0_4 = 1.5;
[X_traj4, U_traj4, J_total4] = solve_optimal_control_dp(N4, dx4, du4, x0_4);
fprintf('Total Optimal Cost (J) for N=200 (coarse grid): %.4f\n', J_total4);
plot_results(N4, X_traj4, U_traj4, 'Scenario 4: N=200, dx=0.1, du=0.1');

% --- Scenario 5: N = 200, dx=0.01, du=0.01 (Fine Grid) ---
fprintf('\n--- Running Scenario 5: N=200, dx=0.01, du=0.01 ---\n');
N5 = 200;
dx5 = 0.01;
du5 = 0.01;
x0_5 = 1.5;
[X_traj5, U_traj5, J_total5] = solve_optimal_control_dp(N5, dx5, du5, x0_5);
fprintf('Total Optimal Cost (J) for N=200 (fine grid): %.4f\n', J_total5);
plot_results(N5, X_traj5, U_traj5, 'Scenario 5: N=200, dx=0.01, du=0.01');



%% DYNAMIC PROGRAMMING SOLVER FUNCTION
function [X_traj, U_traj, J_total] = solve_optimal_control_dp(N, dx, du, x0)
    x_min = 0.0; x_max = 1.5;
    u_min = -1.0; u_max = 1.0;
    
    x_grid = x_min:dx:x_max;
    u_grid = u_min:du:u_max;
    
    nx = length(x_grid); 
    nu = length(u_grid); 

    J = inf(nx, N + 1);
    U_opt = NaN(nx, N);

    J(:, N + 1) = x_grid.^2;

    for k = N-1:-1:0
        for i = 1:nx
            x_k = x_grid(i);
            min_cost_for_state_i = inf;
            best_u_for_state_i = NaN;
            
            for j = 1:nu
                u_k = u_grid(j);
                
                x_next = x_k + u_k;
                
                if x_next >= x_min && x_next <= x_max
                    stage_cost = 2 * u_k^2;

                    cost_to_go = interp1(x_grid, J(:, k + 2), x_next);
                    
                    current_total_cost = stage_cost + cost_to_go;
                    
                    if current_total_cost < min_cost_for_state_i
                        min_cost_for_state_i = current_total_cost;
                        best_u_for_state_i = u_k;
                    end
                end
            end
            
            J(i, k + 1) = min_cost_for_state_i;
            U_opt(i, k + 1) = best_u_for_state_i;
        end
        fprintf('DP Backward Pass: Completed Stage k=%d\n', k);
    end

    if x0 < x_min || x0 > x_max
        error('Initial state x0 is outside the allowed bounds.');
    end
    
    X_traj = zeros(1, N + 1);
    U_traj = zeros(1, N);
    X_traj(1) = x0;
    
    for k = 0:N-1

        U_traj(k + 1) = interp1(x_grid, U_opt(:, k + 1), X_traj(k + 1), 'nearest');
        
        X_traj(k + 2) = X_traj(k + 1) + U_traj(k + 1);
    end
    
    J_total = interp1(x_grid, J(:, 1), x0);
end

%% PLOTTING HELPER FUNCTION
function plot_results(N, X_traj, U_traj, title_str)
    figure('Name', title_str);
    
    time_k_state = 0:N;
    time_k_control = 0:N-1;
    
    subplot(2, 1, 1);
    stairs(time_k_state, X_traj , 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
    hold on;
    yline(1.5, 'r--', 'LineWidth', 1.5);
    yline(0.0, 'r--', 'LineWidth', 1.5);
    title(['Optimal State Trajectory x(k) for ', title_str]);
    xlabel('Time step k');
    ylabel('State x(k)');
    grid on;
    legend('x(k)', 'Constraints', 'Location', 'best');
    ylim([-0.1, 1.6]);

    subplot(2, 1, 2);
    stairs(time_k_control, U_traj, 'LineWidth', 1.5, 'MarkerFaceColor', 'g');
    hold on;
    yline(1.0, 'r--', 'LineWidth', 1.5);
    yline(-1.0, 'r--', 'LineWidth', 1.5);
    title(['Optimal Control Sequence u(k) for ', title_str]);
    xlabel('Time step k');
    ylabel('Control u(k)');
    grid on;
    legend('u(k)', 'Constraints', 'Location', 'best');
    ylim([-1.1, 1.1]);
end
