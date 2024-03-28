clear; clc; close all;

stepData = load("StepInputData.mat");
stepData = stepData.data;
d = 5;

stepTime = stepData(:, 1);
stepRad = stepData(:, 2);
stepCommand = stepData(:, 3);

steadyStateStep = 6.66 / d;

tauIndex = 78;
tau = stepTime(tauIndex);


logTerm = log(1 - stepRad/(steadyStateStep*d));
tauTime = find(stepTime < 1.25 & stepTime > 1);

figure();
plot(stepTime(tauTime), logTerm(tauTime));
xlabel('Time (s)'); % Label the x-axis
ylabel('Log Angular Velocity'); % Label the y-axis
title('Log Plot of Non Driving'); % Title the plot


slope = -18.04;
tau_from_sope = - 1 / slope;

G = tf([steadyStateStep], [tau_from_sope, 1]);
[y, t] = step(G * d);
figure();
plot(t, y, 'DisplayName', 'Theoretical Step Response');
hold on;
plot(stepTime - 1, stepRad, 'ro', 'DisplayName', 'Step Response Data');
plot(stepTime(tauIndex) - 1, stepRad(tauIndex), 'ko', 'DisplayName', 'Estimated Tau');
hold off;
xlabel('Time (s)'); % Label the x-axis
ylabel('Rotational Velocity (rad/s)'); % Label the y-axis
title('Step Response'); % Title the plot
legend(); % Add a legend



%%

drivingStepData = load("DrivingStepInputData.mat");
drivingStepData = drivingStepData.data;

driveTime = drivingStepData(:, 1);
driveRad = drivingStepData(:, 2);
driveCommand = drivingStepData(:, 3);

steady_state = 6.37 / d;


logTerm = log(1 - driveRad/(steady_state*d));
tauTime = find(driveTime < 1.2 & driveTime > 1);

figure();
plot(stepTime(tauTime), logTerm(tauTime));
xlabel('Time (s)'); % Label the x-axis
ylabel('Log Angular Velocity'); % Label the y-axis
title('Log Plot of Driving'); % Title the plot

slope = -28.66;
tau = - 1 / slope;

G = tf(steady_state, [tau, 1]);

[y, t] = step(G * d);
figure();
plot(t, y, 'DisplayName', 'Theoretical Step Response');
hold on;
plot(driveTime - 1, driveRad, 'ro', 'DisplayName', 'Step Response Data');
hold off;
xlabel('Time (s)'); % Label the x-axis
ylabel('Rotational Velocity (rad/s)'); % Label the y-axis
title('Step Response'); % Title the plot
legend(); % Add a legend



