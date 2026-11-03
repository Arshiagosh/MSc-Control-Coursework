clear; clc; close all;

%% (1) Define the TGEN state-space model
A = [-18.4456 4.2263 -2.2830 0.2260 0.4220 -0.0951;
     -4.0977 -6.0706 3.6825 -0.6966 -1.2246 0.2873;
     1.4449 1.4336 -2.6477 0.6092 0.8979 -0.2300;
     -0.0093 0.2302 -0.5002 -0.1764 -6.3132 0.1350;
     -0.0464 -0.3489 0.7238 6.3117 -0.6886 0.3645;
     -0.0602 -0.2361 0.2300 0.0915 -0.3214 -0.2087];

B = [-0.2748 3.1463;
     -0.0501 -9.3737;
     -0.1550 7.4296;
     0.0716 -4.9176;
     -0.0814 -10.2648;
     0.0244 13.7943];

C = [0.5971 -0.7697 4.8850 4.8608 -9.8177 -8.8610;
     3.1013 9.3422 -5.6000 -0.7490 2.9974 10.5719];

D = zeros(2);

% Create the state-space model for G(s)
G = ss(A, B, C, D);

%% (2) High-Frequency Compensation: Compute K_h
% Target frequency for ALIGN at desired bandwidth
w_design = 0.5; % Design frequency (rad/s) for target bandwidth
[Gfreq, ~] = freqresp(G, w_design);
Gfreq = Gfreq(:,:,1);

% Perform SVD for ALIGN
[U, S, V] = svd(Gfreq);

% Construct K_h using the ALIGN algorithm
K_h = -real(U * V');
disp('High-Frequency Compensation (K_h) using ALIGN:');
disp(K_h);

% Create static gain block for K_h
Kh_ss = ss(K_h);

% Open-loop system after applying K_h
Gol_h = series(Kh_ss, G);

% Plot characteristic loci of G*K_h
figure('Name', 'Characteristic Loci of G*K_h');
w = logspace(-2, 2, 1000);
nyquist(Gol_h);
grid on;
title('Characteristic Loci of G*K_h');

%% (3) Medium-Frequency Compensation: Design K_m with Lead-Lag
% Design lead-lag for phase margin improvement
s = tf('s');
a = 0.1; % Lead-lag parameter
T = 2; % Time constant
K_lead = (1 + s*T)/(1 + s*a*T); % Lead compensator

% Apply to both channels
K_m = blkdiag(K_lead, K_lead);

% Display transfer function
disp('Medium-Frequency Compensation (K_m):');
disp(K_lead);

% Combine K_h and K_m
Gol_m = series(K_m, Gol_h);

% Plot characteristic loci
figure('Name', 'Characteristic Loci of G*K_h*K_m');
nyquist(Gol_m);
grid on;
title('Characteristic Loci of G*K_h*K_m');

%% (4) Low-Frequency Compensation: Add Integral Action (K_L)
% Add integral action for zero steady-state error
Ti = 2; % Integral time constant
K_int = (s + 1/Ti)/s; % PI controller
K_L = blkdiag(K_int, K_int);

% Display transfer function
disp('Low-Frequency Compensation (K_L):');
disp(K_int);

% Combine all compensators
K_total = series(K_L, Gol_m);

% Plot final characteristic loci
figure('Name', 'Final Characteristic Loci');
nyquist(K_total);
grid on;
title('Characteristic Loci of Complete System');

%% (5) Closed-Loop Performance Analysis
% Form closed-loop system
CL = feedback(K_total, eye(2));

% Sensitivity and complementary sensitivity
S = feedback(eye(2), K_total);
T = feedback(K_total, eye(2));

% Step response
figure('Name', 'Closed-Loop Step Response');
step(CL, 10);
grid on;
title('Closed-Loop Step Response');

% Frequency response analysis
figure('Name', 'Frequency Response Analysis');

% Singular values of closed-loop system
subplot(2,2,1);
sigma(CL, w);
grid on;
title('Singular Values of Closed-Loop System');

% Sensitivity function
subplot(2,2,2);
sigma(S, w);
grid on;
title('Sensitivity Function');

% Complementary sensitivity
subplot(2,2,3);
sigma(T, w);
grid on;
title('Complementary Sensitivity');

% Performance Analysis
[mag_S, ~] = bode(S, w);
[mag_T, ~] = bode(T, w);

% Check specifications
low_freq_sens = max(max(squeeze(mag_S(:,:,w < 0.05))));
high_freq_noise = max(max(squeeze(mag_T(:,:,w > 50))));

fprintf('\nPerformance Analysis:\n');
fprintf('====================\n');
fprintf('Maximum sensitivity below 0.05 rad/s: %.4f\n', low_freq_sens);
fprintf('Maximum noise amplification above 50 rad/s: %.4f\n', high_freq_noise);

% Calculate and display stability margins from characteristic loci
eigvals = zeros(2, length(w));
for i = 1:length(w)
    L_w = freqresp(K_total, w(i));
    L_w = reshape(L_w, [2 2]);
    eigvals(:,i) = eig(L_w);
end

% Find minimum distance to critical point (-1,0)
min_dist = min(min(abs(eigvals + 1)));
gain_margin = 20*log10(1 + 1/min_dist);

fprintf('\nStability Analysis:\n');
fprintf('==================\n');
fprintf('Minimum distance to critical point: %.4f\n', min_dist);
fprintf('Guaranteed gain margin: %.2f dB\n', gain_margin);

% Plot characteristic value magnitudes
figure('Name', 'Characteristic Value Magnitudes');
semilogx(w, 20*log10(abs(eigvals(1,:))), 'b', w, 20*log10(abs(eigvals(2,:))), 'r--');
grid on;
title('Characteristic Value Magnitudes');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');
legend('\lambda_1', '\lambda_2');