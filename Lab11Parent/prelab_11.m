clear, clc, close all;

% Given values for K and tau
K = 1.538;
tau = .0615;

% Transfer function G(s) = K / (tau*s + 1)
num = K;
den = [tau 1];
G = tf(num, den);

% Step input magnitude
d = 1; % assuming a step input of 1 Volt

% Part b) Step response for a step input of magnitude d
syms unitD;
figure;
fplot(heaviside(unitD), [-1, 4 * tau]);

% Plot the step response - velocity vs. time
figure;
step(G * d);
title('Velocity vs. Time for a Step Input of Magnitude d');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');

% Steady-state speed verification, Omega_ss = K/d
Omega_ss = K / d;
disp(['The steady-state speed is: ', num2str(Omega_ss), ' rad/s'])

% Part c) Plot ln(1-Omega(t)/Omega_ss) vs time for t < 4*tau
small_number = 1e-10;
t = small_number:0.01:4*tau; % time vector from just above 0 to 4*tau
Omega_t = step(G * d, t);
ln_term = log(1 - Omega_t/Omega_ss);

figure;
plot(t, ln_term);
title('Plot of ln(1-Omega(t)/Omega_ss) vs. Time for t < 4*tau');
xlabel('Time (s)');
ylabel('ln(1-Omega(t)/Omega_ss)');
grid on;

% Verifying the slope -1/tau
slope = -1/tau;
disp(['The slope should be approximately: ', num2str(slope)]);
