timeSpan = [15 0];
initialValues = [0 0 0];
[t, sol] = ode45(@dynamicRiccati, timeSpan, initialValues);
t = flipud(t);
sol = flipud(sol);
K11 = timeseries(sol(:,1), t);
K12 = timeseries(sol(:,2), t);
K22 = timeseries(sol(:,3), t);
K = struct('K11', K11, 'K12', K12, 'K22', K22);
disp('Riccati gains generated and loaded into workspace.');
function dK = dynamicRiccati(~, kVec)
    dK = zeros(3,1);
    dK(1) = 2 * (kVec(2)^2 - 2 * kVec(2) - 1);                         % dK11/dt
    dK(2) = 2 * kVec(2) * kVec(3) - kVec(1) + kVec(2) - 2 * kVec(3);   % dK12/dt
    dK(3) = 2 * (kVec(3)^2) - 2 * kVec(2) + 2 * kVec(3) - 1;           % dK22/dt
end
