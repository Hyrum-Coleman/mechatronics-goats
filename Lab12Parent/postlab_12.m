clear, clc, close all;


%% Question 1

G_plant = tf(1.4, [.07, 1, 0]);

experimental_underdamped_p = load("UnderDampedPrelabP.mat");
kp_underdamped = 12.27;
step_size_underdamped = .8;
G_underdamped = feedback(kp_underdamped * G_plant, 1);
[y_theo_underdamped, t_theo_underdamped] = step(G_underdamped * step_size_underdamped);

figure;
plot(t_theo_underdamped, y_theo_underdamped, 'b');
hold on
plot(experimental_underdamped_p.time,experimental_underdamped_p.position, 'ro');
line(xlim, [step_size_underdamped, step_size_underdamped], 'Color', 'k', 'LineStyle', ':');
hold off
title('Underdamped P Controller Step Response');
xlabel('Time (s)');
ylabel('Output');
legend('Theoretical', 'Experimental');
grid on;

experimental_critdamped_p = load("CritDampedPrelabP.mat");
kp_critdamped = 2.55;
step_size_critdamped = 1;
G_critdamped = feedback(kp_critdamped * G_plant, 1);
[y_theo_critdamped, t_theo_critdamped] = step(G_critdamped * step_size_critdamped);


figure;
plot(t_theo_critdamped, y_theo_critdamped, 'b');
hold on
plot(experimental_critdamped_p.time, experimental_critdamped_p.position, 'ro');
line(xlim, [step_size_critdamped, step_size_critdamped], 'Color', 'k', 'LineStyle', ':');
hold off
title('Critically Damped P Controller Step Response');
xlabel('Time (s)');
ylabel('Output');
legend('Theoretical', 'Experimental');
grid on;

%% Question 2

experimental_pd = load("PDControlPrelab.mat");
kp_pd = 18.66;
kd_pd = .619;
step_size_pd = .5;

C_pd = pid(kp_pd, 0, kd_pd);

G_closed_pd = feedback(C_pd * G_plant, 1);

% Get theoretical step response
[y_theo_pd, t_theo_pd] = step(G_closed_pd * step_size_pd);

% Plot theoretical and experimental PD step responses
figure;
plot(t_theo_pd, y_theo_pd, 'b');
hold on
plot(experimental_pd.time, experimental_pd.position, 'ro');
line(xlim, [step_size_pd, step_size_pd], 'Color', 'k', 'LineStyle', ':');
title('PD Controller Step Response');
hold off
xlabel('Time (s)');
ylabel('Output');
legend('Theoretical', 'Experimental');
grid on;

%% Question 3

experimental_pid = load("PIDStepResponse.mat");
kp_pid = 40;
ki_pid = 350;
kd_pid = .5;
step_size_pid = .25;

C_pid = pid(kp_pid, ki_pid, kd_pid);

G_closed_pid = feedback(C_pid * G_plant, 1);

[y_theo_pid, t_theo_pid] = step(G_closed_pid * step_size_pid);

% Plot theoretical and experimental PID step responses
figure;
plot(t_theo_pid, y_theo_pid, 'b');
hold on;
plot(experimental_pid.time, experimental_pid.position, 'ro');
line(xlim, [step_size_pid, step_size_pid], 'Color', 'k', 'LineStyle', ':');
hold off;
title('PID Controller Step Response');
xlabel('Time (s)');
ylabel('Output');
legend('Theoretical', 'Experimental');
grid on;