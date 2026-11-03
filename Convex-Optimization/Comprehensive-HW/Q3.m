function uav_path_planning_qp()
% UAV Path Optimization via Successive Convex QP
% Arshia-ready: clean, modular, easy to tweak.

%% -------------------- Parameters --------------------
T   = 20;          % horizon (positions defined t=0..T)
dt  = 1.0;         % time step
alpha  = 1.0;      % smoothing weight
lambdas = [0, 1, 10];  % risk penalty weights to compare

x0 = [0; 0];
xT = [10; 10];

% Box workspace
xmin = -1; xmax = 12;
ymin = -1; ymax = 12;

% Limits (box constraints as conservative approx of l2)
vmax = 1.0;       % |vx|<=vmax, |vy|<=vmax
amax = 2.0;       % |ax|<=amax, |ay|<=amax

% Circular obstacles
C = [5 5; 2 7; 8 3];        % centers (m x 2)
r = [1.0; 0.8; 1.2];        % radii
mCirc = size(C,1);

% Linear danger half-spaces: a_j^T x + b_j <= 0 is SAFE (<=0)
% Example from prompt: 10(x-6)+8(y-6) <= -15  -> 10x + 8y - (10*6+8*6) <= -15
A_lin = [10 8;  % region 1
         1  -2]; % region 2 (example orientation; change as needed)
b_lin = [ -15 + 10*6 + 8*6;   % shift from example
          0 + 1*9 + (-2)*2];  % example: plane through (9,2) with margin 0
mLin = size(A_lin,1);

% Iteration config
maxIter = 15;
tol     = 1e-3;
trust_margin = 0.00; % can set small positive if you want extra clearance
damping = 0.7;       % convex combo with previous path to stabilize

rng(1); % reproducibility for any random init parts

%% -------------------- Initial straight-line guess --------------------
x_guess = zeros(2, T+1);
v_guess = zeros(2, T+1);
u_guess = zeros(2, T);
for t = 0:T
    x_guess(:,t+1) = (1 - t/T)*x0 + (t/T)*xT;
end
v_guess(:,1:T) = diff(x_guess,1,2)/dt;
v_guess(:,T+1) = v_guess(:,T);

%% -------------------- Solve for different lambdas --------------------
figure('Color','w'); hold on; axis equal; grid on;
xlabel('x'); ylabel('y'); xlim([xmin-0.5, xmax+0.5]); ylim([ymin-0.5, ymax+0.5]);
title('UAV Paths via Successive Convex QP');

% Draw workspace box
plot([xmin xmax xmax xmin xmin],[ymin ymin ymax ymax ymin],'k--','LineWidth',1);

% Draw circular obstacles
theta = linspace(0,2*pi,200);
for i=1:mCirc
    plot(C(i,1)+r(i)*cos(theta), C(i,2)+r(i)*sin(theta),'r','LineWidth',1.5);
end

% Draw linear danger boundaries (a^T x + b = 0) and unsafe side (>0)
xx = linspace(xmin, xmax, 200);
for j=1:mLin
    a = A_lin(j,:).'; b = b_lin(j);
    % a1 x + a2 y + b = 0 -> y = -(a1/a2) x - b/a2  (if a2 ~= 0)
    if abs(a(2))>1e-8
        yy = -(a(1)/a(2))*xx - b/a(2);
        plot(xx, yy, 'm-', 'LineWidth', 1.2);
    end
end

colors = lines(numel(lambdas));
legends = cell(1,numel(lambdas));

for k=1:numel(lambdas)
    lambda = lambdas(k);
    [x_opt, v_opt, u_opt] = solve_one(lambda, x_guess, v_guess, u_guess, ...
        x0, xT, T, dt, alpha, trust_margin, damping, ...
        C, r, A_lin, b_lin, vmax, amax, xmin, xmax, ymin, ymax, ...
        maxIter, tol);
    
    plot(x_opt(1,:), x_opt(2,:), '-', 'LineWidth', 2, 'Color', colors(k,:));
    scatter(x_opt(1,1), x_opt(2,1), 60, colors(k,:), 'filled');  % start
    scatter(x_opt(1,end), x_opt(2,end), 60, colors(k,:), 'd', 'filled'); % goal
    legends{k} = sprintf('\\lambda = %g', lambda);
end
legend(legends, 'Location', 'best');

end % main


function [x_path, v_path, u_path] = solve_one(lambda, x_init, v_init, u_init, ...
    x0, xT, T, dt, alpha, trust_margin, damping, ...
    C, r, A_lin, b_lin, vmax, amax, xmin, xmax, ymin, ymax, ...
    maxIter, tol)

mCirc = size(C,1);
mLin  = size(A_lin,1);

% Decision vector layout (z):
% For convenience, we stack by time:
% z = [x(1:T-1) ; y(1:T-1) ; vx(0:T) ; vy(0:T) ; ax(0:T-1) ; ay(0:T-1) ; sCirc(:) ; sLin(:)]
% We don't include x(0) and x(T) directly (they’re fixed by eq constraints).
% Positions at interior times are decision variables; endpoints fixed.
nXint = T-1;    % interior positions: t=1..T-1
nV    = T+1;    % velocities t=0..T
nU    = T;      % accelerations t=0..T-1
nSCirc = mCirc * (T+1); % one slack per obstacle per t
nSLin  = mLin  * (T+1); % one slack per linear zone per t

% Index helpers
idx = struct();
ofs = 0;
idx.x = ofs + (1:nXint); ofs = ofs + nXint;
idx.y = ofs + (1:nXint); ofs = ofs + nXint;
idx.vx = ofs + (1:nV);   ofs = ofs + nV;
idx.vy = ofs + (1:nV);   ofs = ofs + nV;
idx.ax = ofs + (1:nU);   ofs = ofs + nU;
idx.ay = ofs + (1:nU);   ofs = ofs + nU;
idx.sCirc = ofs + (1:nSCirc); ofs = ofs + nSCirc;
idx.sLin  = ofs + (1:nSLin);  ofs = ofs + nSLin;
nZ = ofs;

% Initial z guess from (x_init, v_init, u_init)
z = zeros(nZ,1);
% Fill interior positions
for t=1:T-1
    z(idx.x(t)) = x_init(1,t+1);
    z(idx.y(t)) = x_init(2,t+1);
end
% velocities
for t=0:T
    z(idx.vx(t+1)) = v_init(1,t+1);
    z(idx.vy(t+1)) = v_init(2,t+1);
end
% accelerations
for t=0:T-1
    z(idx.ax(t+1)) = u_init(1,t+1);
    z(idx.ay(t+1)) = u_init(2,t+1);
end
% slacks init = 0
% z(idx.sCirc) = 0; z(idx.sLin) = 0;

% Pre-build equality constraints for dynamics and position continuity
[Aeq, beq] = build_equalities(idx, T, dt, x0, xT);

% Bounds for box constraints
[lb, ub] = build_bounds(idx, T, xmin, xmax, ymin, ymax, vmax, amax);

prev_cost = inf;

for it = 1:maxIter
    % Extract current full position trajectory X(t), Y(t) from z (including endpoints)
    X = zeros(1,T+1); Y = zeros(1,T+1);
    X(1) = x0(1); Y(1) = x0(2);
    for t=1:T-1
        X(t+1) = z(idx.x(t));
        Y(t+1) = z(idx.y(t));
    end
    X(T+1) = xT(1); Y(T+1) = xT(2);

    % ---- Linearized obstacle and linear zone constraints (Aineq z <= bineq) ---
    [Aineq, bineq, Qpen, qpen] = build_ineq_and_penalty(idx, X, Y, C, r, ...
        A_lin, b_lin, trust_margin, lambda, T);

    % ---- Quadratic objective H, f ----
    H = build_H(idx, T, alpha) + Qpen;
    f = qpen; % linear term from penalties (zero here; penalties are on slack^2 only)

    % Solve QP
    % min 0.5 z'H z + f'z  s.t.  Aeq z = beq,  Aineq z <= bineq,  lb<=z<=ub
    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
    [z_new, cost, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, lb, ub, z, opts);
    if exitflag <= 0
        warning('quadprog did not converge (it=%d, exitflag=%d). Using previous iterate.', it, exitflag);
        z_new = z; cost = prev_cost;
    end

    % Damping for stability
    z = damping*z + (1-damping)*z_new;

    % Check convergence
    rel_impr = (prev_cost - cost)/max(1,abs(prev_cost));
    if rel_impr < tol && rel_impr >= 0
        % converged
        break;
    end
    prev_cost = cost;
end

% Unstack solution to (x_path, v_path, u_path)
x_path = zeros(2,T+1);
v_path = zeros(2,T+1);
u_path = zeros(2,T);

x_path(:,1) = x0; x_path(:,end) = xT;
for t=1:T-1
    x_path(1,t+1) = z(idx.x(t));
    x_path(2,t+1) = z(idx.y(t));
end
for t=0:T
    v_path(1,t+1) = z(idx.vx(t+1));
    v_path(2,t+1) = z(idx.vy(t+1));
end
for t=0:T-1
    u_path(1,t+1) = z(idx.ax(t+1));
    u_path(2,t+1) = z(idx.ay(t+1));
end

end % solve_one


function [Aeq, beq] = build_equalities(idx, T, dt, x0, xT)
% Dynamics:
% x_{t+1} = x_t + vx_t*dt
% y_{t+1} = y_t + vy_t*dt
% v_{t+1} = v_t + a_t*dt
% We eliminate x_0,x_T from decision by fixing them; interior x_t are variables.

nZ = max(structfun(@max, idx));
Aeq = []; beq = [];

% Position continuity: for t = 0..T-1, both x and y
% We build equalities for interior time steps that tie x(t+1)-x(t)-vx_t*dt = 0
% For t=0: x(1) known = x0, but x(2) is decision, etc.
for t=0:T-1
    % x dim
    row = zeros(1,nZ);
    % x(t+1)
    if t+1 >=1 && t+1 <= T-1
        row(idx.x(t+1)) = 1;
    elseif t+1 == 0
        % never happens
    elseif t+1 == T
        % x_T fixed -> move to beq
    end
    % - x(t)
    if t>=1 && t<=T-1
        row(idx.x(t)) = row(idx.x(t)) - 1;
    elseif t==0
        % subtract x0 into beq
    end
    % - vx_t*dt
    row(idx.vx(t+1)) = row(idx.vx(t+1)) - dt;
    b = 0;
    % Handle constants (x0 and xT) moved to RHS
    if t==0
        b = b - x0(1);
    end
    if t+1 == T
        b = b + xT(1);
    end
    Aeq = [Aeq; row]; beq = [beq; b];

    % y dim
    row = zeros(1,nZ);
    if t+1 >=1 && t+1 <= T-1
        row(idx.y(t+1)) = 1;
    end
    if t>=1 && t<=T-1
        row(idx.y(t)) = row(idx.y(t)) - 1;
    elseif t==0
        % subtract y0 into beq
    end
    row(idx.vy(t+1)) = row(idx.vy(t+1)) - dt;
    b = 0;
    if t==0
        b = b - x0(2);
    end
    if t+1 == T
        b = b + xT(2);
    end
    Aeq = [Aeq; row]; beq = [beq; b];
end

% Velocity update: for t=0..T-1, vx_{t+1} - vx_t - ax_t*dt = 0
for t=0:T-1
    row = zeros(1,nZ);
    row(idx.vx(t+1+1)) = 1;      % vx_{t+1}
    row(idx.vx(t+1))   = row(idx.vx(t+1)) - 1; % -vx_t
    row(idx.ax(t+1))   = row(idx.ax(t+1)) - dt; % -ax_t*dt
    Aeq = [Aeq; row]; beq = [beq; 0];

    row = zeros(1,nZ);
    row(idx.vy(t+1+1)) = 1;
    row(idx.vy(t+1))   = row(idx.vy(t+1)) - 1;
    row(idx.ay(t+1))   = row(idx.ay(t+1)) - dt;
    Aeq = [Aeq; row]; beq = [beq; 0];
end

end % build_equalities


function [lb, ub] = build_bounds(idx, T, xmin, xmax, ymin, ymax, vmax, amax)
nZ = max(structfun(@max, idx));
lb = -inf(nZ,1); ub = inf(nZ,1);

% Interior positions
for t=1:T-1
    lb(idx.x(t)) = xmin; ub(idx.x(t)) = xmax;
    lb(idx.y(t)) = ymin; ub(idx.y(t)) = ymax;
end

% Velocities box bounds
for t=0:T
    lb(idx.vx(t+1)) = -vmax; ub(idx.vx(t+1)) =  vmax;
    lb(idx.vy(t+1)) = -vmax; ub(idx.vy(t+1)) =  vmax;
end

% Accelerations box bounds
for t=0:T-1
    lb(idx.ax(t+1)) = -amax; ub(idx.ax(t+1)) =  amax;
    lb(idx.ay(t+1)) = -amax; ub(idx.ay(t+1)) =  amax;
end

% Slacks must be >= 0
lb(idx.sCirc) = 0;  % no upper bound
lb(idx.sLin)  = 0;

end % build_bounds


function H = build_H(idx, T, alpha)
% Build the quadratic cost: sum ||u_t||^2 + alpha*sum ||x_{t+1}-x_t||^2
% u part: ax^2 + ay^2
% smoothing part couples x(t+1) and x(t), similarly for y

nZ = max(structfun(@max, idx));
H = sparse(nZ,nZ);

% Control effort
for t=0:T-1
    H(idx.ax(t+1), idx.ax(t+1)) = H(idx.ax(t+1), idx.ax(t+1)) + 2; % derivative 0.5*z'H z -> set 2 for ||u||^2
    H(idx.ay(t+1), idx.ay(t+1)) = H(idx.ay(t+1), idx.ay(t+1)) + 2;
end

% Smoothing: alpha * sum ||x_{t+1}-x_t||^2 for t=0..T-1
% Expand for x-dim:
% For interior variables, this yields a tridiagonal pattern. Endpoints are constants handled in beq; we account only variable parts.
% x(1..T-1) variables -> quadratic form with coefficients:
%   For t=1..T-2: alpha*( (x_{t+1}-x_t)^2 + (x_{t+2}-x_{t+1})^2 ) -> diagonals and off-diagonals
% Implement directly:
for t=0:T-1
    % involves x(t) and x(t+1). Cases with endpoints require care.
    if t>=1 && t<=T-2
        % contributes: +alpha to x(t)^2, +alpha to x(t+1)^2, -2alpha x(t)x(t+1)
        H(idx.x(t), idx.x(t)) = H(idx.x(t), idx.x(t)) + 2*alpha;
        H(idx.x(t+1), idx.x(t+1)) = H(idx.x(t+1), idx.x(t+1)) + 2*alpha;
        H(idx.x(t), idx.x(t+1)) = H(idx.x(t), idx.x(t+1)) - 2*alpha;
        H(idx.x(t+1), idx.x(t)) = H(idx.x(t+1), idx.x(t)) - 2*alpha;
    elseif t==0
        % only x(1) variable with x0 constant -> +alpha*x(1)^2 -2alpha*x0*x(1) (linear term omitted here; we keep only quadratic)
        H(idx.x(1), idx.x(1)) = H(idx.x(1), idx.x(1)) + 2*alpha;
    elseif t==T-1
        % x(T-1) variable with xT constant -> +alpha*x(T-1)^2 -2alpha*xT*x(T-1)
        H(idx.x(T-1), idx.x(T-1)) = H(idx.x(T-1), idx.x(T-1)) + 2*alpha;
    end
end

% Same for y-dim
for t=0:T-1
    if t>=1 && t<=T-2
        H(idx.y(t), idx.y(t)) = H(idx.y(t), idx.y(t)) + 2*alpha;
        H(idx.y(t+1), idx.y(t+1)) = H(idx.y(t+1), idx.y(t+1)) + 2*alpha;
        H(idx.y(t), idx.y(t+1)) = H(idx.y(t), idx.y(t+1)) - 2*alpha;
        H(idx.y(t+1), idx.y(t)) = H(idx.y(t+1), idx.y(t)) - 2*alpha;
    elseif t==0
        H(idx.y(1), idx.y(1)) = H(idx.y(1), idx.y(1)) + 2*alpha;
    elseif t==T-1
        H(idx.y(T-1), idx.y(T-1)) = H(idx.y(T-1), idx.y(T-1)) + 2*alpha;
    end
end

end % build_H


function [Aineq, bineq, Qpen, qpen] = build_ineq_and_penalty(idx, X, Y, C, r, A_lin, b_lin, trust_margin, lambda, T)
% Linearized obstacle constraints and linear zone constraints with nonnegative slacks.
% We collect them as:
%   n_{i,t}^T [x_t] >= n_{i,t}^T c_i + r_i + trust_margin - s_{i,t}   (circ)
%   a_j^T [x_t] + b_j <= s_{j,t}                                       (lin)
% and penalize sum s^2 in objective: lambda * sum s^2

nZ = max(structfun(@max, idx));
Aineq = []; bineq = [];

mCirc = size(C,1);
mLin  = size(A_lin,1);

% Penalty purely quadratic on slacks
Qpen = sparse(nZ,nZ);
qpen = zeros(nZ,1);

% Circular obstacles
for t=0:T
    xt = X(t+1); yt = Y(t+1);
    for i=1:mCirc
        ci = C(i,:).';
        diff = [xt; yt] - ci;
        dist = norm(diff);
        if dist < 1e-6
            n = [1;0]; % arbitrary
        else
            n = diff/dist; % outward normal
        end
        rhs = n.'*ci + r(i) + trust_margin;

        % Build inequality: n^T x_t >= rhs - s_{i,t}
        % -> -n^T x_t + s_{i,t} <= -rhs
        row = zeros(1,nZ);
        % place x_t variables (interior t)
        if t>=1 && t<=T-1
            row(idx.x(t)) = row(idx.x(t)) - n(1);
            row(idx.y(t)) = row(idx.y(t)) - n(2);
        else
            % t=0 or t=T -> x fixed: move to RHS
        end
        % slack index mapping
        sIdx = idx.sCirc( (t)*mCirc + i );
        row(sIdx) = row(sIdx) + 1;

        b = -rhs;
        % add constants from fixed endpoints
        if t==0
            b = b + n.'*[X(1); Y(1)];
        elseif t==T
            b = b + n.'*[X(end); Y(end)];
        end

        Aineq = [Aineq; row]; bineq = [bineq; b];

        % penalty lambda * s^2
        Qpen(sIdx,sIdx) = Qpen(sIdx,sIdx) + 2*lambda;
    end
end

% Linear danger half-spaces: a_j^T x + b_j <= s_{j,t}
for t=0:T
    for j=1:mLin
        a = A_lin(j,:).'; b0 = b_lin(j);

        row = zeros(1,nZ);
        if t>=1 && t<=T-1
            row(idx.x(t)) = row(idx.x(t)) + a(1);
            row(idx.y(t)) = row(idx.y(t)) + a(2);
        end
        sIdx = idx.sLin( (t)*mLin + j );
        row(sIdx) = row(sIdx) - 1;  % a^T x + b - s <= 0  -> row*z <= -b

        b = -b0;
        if t==0
            b = b - a.'*[X(1); Y(1)];
        elseif t==T
            b = b - a.'*[X(end); Y(end)];
        end

        Aineq = [Aineq; row]; bineq = [bineq; b];

        % penalty lambda * s^2
        Qpen(sIdx,sIdx) = Qpen(sIdx,sIdx) + 2*lambda;
    end
end

end % build_ineq_and_penalty
