% plot(data{:,1}, data{:,2})

for i = 1:size(matrix,1)
    matrix(i,2) = matrix(i,2) - 21.65;
end
x = matrix(:,1);
y = matrix(:,2);

% T = 243.5;
% To = 21.79;
% tau = To*0.5;
% k = cft_result.k;
% c = 21.65;

% Kp = T/(k*(tau+To));
% Ti = min(T,4*(tau+To));

% s = tf('s');
% pade = (1-(s*To)/2)/(1+(s*To)/2);

% Go = (k*pade)/(s*T+1)
sys = tfest(matrix,1);
Gz = c2d(sys, 0.001);
pidTuner(Gz, 'PID')