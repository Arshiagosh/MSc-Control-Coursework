timeSpan = [15 0];
BoundryValues = [0 0 0 0 0];
[t, sol] = ode45(@dynamicRiccati, timeSpan, BoundryValues);
t = flipud(t);
sol = flipud(sol);
K11 = timeseries(sol(:,1), t);
K12 = timeseries(sol(:,2), t);
K22 = timeseries(sol(:,3), t);
S1 = timeseries(sol(:,4), t);
S2 = timeseries(sol(:,5), t);
K = struct('K11', K11, 'K12', K12, 'K22', K22, 'S1', S1, 'S2', S2);
disp('Riccati gains generated and loaded into workspace.');
function dK = dynamicRiccati(t, kVec)
    dK = zeros(5,1);
    dK(1) = 20 * kVec(2).^2 - 4*kVec(2) - 2;                         % dK11/dt
    dK(2) = 20 * kVec(2) * kVec(3) - kVec(1) + kVec(2) - 2*kVec(3);  % dK12/dt
    dK(3) = 20 * kVec(3).^2 - 2*kVec(2) + 2*kVec(3);                 % dK22/dt
    dK(4) = 2*(10*kVec(2)-1)*kVec(5) + 0.4*t;                        % dS1/dt
    dK(5) = -kVec(4)+(20*kVec(3)+1)*kVec(5);                         % dS2/dt
end

