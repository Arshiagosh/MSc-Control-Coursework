clear; clc; close all;

%% (1) Define the AIRC state-space model (A, B, C, D)
A = [
    0       0       1.1320   0       -1.0000
    0      -0.0538  -0.1712  0        0.0705
    0       0        0       1.0000   0
    0       0.0485   0      -0.8556  -1.0130
    0      -0.2909   0       1.0532  -0.6859
];

B = [
    0       0       0
   -0.120   1.0000  0
    0       0       0
    4.4190  0      -1.6650
    1.5750  0      -0.0732
];

C = [
    1  0  0  0  0
    0  1  0  0  0
    0  0  1  0  0
];

D = zeros(3,3);

% Create the state-space model for G(s)
G = ss(A, B, C, D);

%% (2) High-Frequency Compensation: Compute K_h
% Target frequency for ALIGN
w_design = 3;  % Design frequency (rad/s)
[Gfreq, ~] = freqresp(G, w_design);  % Frequency response at w = 3 rad/s
Gfreq = Gfreq(:,:,1);                % Extract slice at s = j3

% Perform Singular Value Decomposition (SVD) for ALIGN
[U, S, V] = svd(Gfreq);

% Construct K_h using the ALIGN algorithm
K_h = -real(U * V');

disp('High-Frequency Compensation (K_h) using ALIGN:');
disp(K_h);

% Create a static gain block for K_h
Kh_ss = ss(K_h);

% Display the transfer function of K_h
disp('Transfer Function of High-Frequency Compensation (K_h):');
disp(K_h);

% Open-loop system after applying K_h
Gol_h = series(Kh_ss, G);

% Plot characteristic loci (Nyquist Plot) of G*K_h
figure('Name', 'Nyquist Plot of G*K_h');
nyquist(Gol_h);
grid on;
title('Nyquist Plot of G*K_h');

%% (3) Medium-Frequency Compensation: Design K_m with Lead-Lag
% Design a lead-lag compensator for ~40 degrees phase lead
a = 0.2175;   % Lead-lag parameter
T = 1.4295;   % Time constant for peak phase lead
s = tf('s');
K_lead = a * (1 + s*T) / (1 + s*a*T);  % Standard lead-lag form

% Extend lead-lag to all channels (diagonal matrix)
K_m = blkdiag(K_lead, K_lead, K_lead);

% Display the transfer function of K_m
disp('Transfer Function of Medium-Frequency Compensation (K_m):');
disp(K_lead);

% Combine K_h and K_m
Gol_m = series(K_m, Gol_h);

disp('Medium-Frequency Compensation (K_m): Lead-Lag designed.');

% Plot characteristic loci (Nyquist Plot) of G*K_h*K_m
figure('Name', 'Nyquist Plot of G*K_h*K_m');
nyquist(Gol_m);
grid on;
title('Nyquist Plot of G*K_h*K_m');

%% (4) Low-Frequency Compensation: Add Integral Action (K_L)
% Add integral action for zero steady-state error
Ti = 2;  % Time constant for integrator
K_int = (1 + s*Ti) / (s*Ti);  % Integral compensator
K_L = blkdiag(K_int, K_int, K_int);

% Display the transfer function of K_L
disp('Transfer Function of Low-Frequency Compensation (K_L):');
disp(K_int);

% Combine all compensators: K_total = K_h * K_m * K_L
K_total = series(K_L, Gol_m);

disp('Low-Frequency Compensation (K_L): Integral action added.');

% Plot characteristic loci (Nyquist Plot) of the final open-loop system
figure('Name', 'Nyquist Plot of Final Open-Loop');
nyquist(K_total);
grid on;
title('Nyquist Plot of G*K_h*K_m*K_L');

%% (5) Closed-Loop Performance
% Close the loop with negative unity feedback
CL = feedback(K_total, eye(3));

% Step response of the closed-loop system
figure('Name', 'Closed-Loop Step Responses');
step(CL, 5);  % Simulate for 5 seconds
grid on;
title('Closed-Loop Step Responses');

% Singular value Bode plot of the closed-loop system
figure('Name', 'Singular Value Bode Plot of Closed-Loop System');
sigma(CL, {1e-2, 1e2});
grid on;
title('Singular Value Bode Plot of Closed-Loop System');

disp('Design process complete. Check the figures for performance:');
disp('- Verify bandwidth, stability, and time-domain response.');
