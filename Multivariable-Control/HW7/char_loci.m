% Define a 2x2 MIMO transfer function system
s = tf('s');
G = [ 1/(s+1), -1/(s+2); 1/(s+1) 1/(s+4) ];
% Extract numerator and denominator matrices
[num, den] = tfdata(G, 'v');
step = 10000;
wi = -12;
wf = 12;
% Frequency range for Nyquist
freq1 = complex(0, -logspace(wf, wi, step));
freq2 = complex(0, logspace(wi, wf, step));
freq = [freq1 freq2];
n = length(freq1);
% Compute symbolic representation
syms s_sym;
G_symbolic = cell(size(num));
for i = 1:size(num, 1)
for j = 1:size(num, 2)
G_symbolic{i, j} = poly2sym(num{i, j}, s_sym) / poly2sym(den{i, j}, s_sym);
end
end
% Simplify symbolic representation
G_simplified = cell(size(G_symbolic));
for i = 1:size(G_symbolic, 1)
for j = 1:size(G_symbolic, 2)
G_simplified{i, j} = simplify(G_symbolic{i, j});
end
end
disp('G in simplified symbolic form:')
disp(G_simplified);
% Convert the symbolic cell array to a symbolic matrix
G_sym_matrix = cell2sym(G_simplified);
% Compute eigenvalues of G(s)
e = eig(G_sym_matrix);
% Compute Nyquist points for each eigenvalue
nyq1 = zeros(n, length(e));
nyq2 = zeros(n, length(e));
for k = 1:length(e)
nyq1(:, k) = double(subs(e(k), freq1));
nyq2(:, k) = double(subs(e(k), freq2));
end
% Plot Nyquist diagram
figure; hold on;
colors = ['b', 'r', 'g', 'm', 'y', 'k']; % Define colors for eigenvalues
for k = 1:length(e)
co = colors(mod(k - 1, length(colors)) + 1); % Cycle through colors
plot(real(nyq1(:, k)), imag(nyq1(:, k)), co);
plot(real(nyq2(:, k)), imag(nyq2(:, k)), co);
% Add frequency markers
mid_idx1 = fix(n / 2.5);
mid_idx2 = fix(n / 4);
text(real(nyq2(mid_idx1, k)), imag(nyq2(mid_idx1, k)), ...
['\rightarrow\omega=', int2str(abs(imag(freq2(mid_idx1))))], 'FontSize', 10, 'fontweight', 'b');
text(real(nyq2(mid_idx2, k)), imag(nyq2(mid_idx2, k)), ...
['\rightarrow\omega=', int2str(abs(imag(freq2(mid_idx2))))], 'FontSize', 10, 'fontweight', 'b');
end
% Plot critical point
plot(-1, 0, 'm*');
xlabel('Real');
ylabel('Imaginary');
title('Nyquist Diagram of the MIMO System');
grid on;
hold off;
