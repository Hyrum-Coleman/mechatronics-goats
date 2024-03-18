clear, clc, close all

data = load('Lab10_motordata.mat');

speed_rads = data.Speed;
torque_Nm = data.Torque;
current_A = data.Current;
voltage_V = data.Volt;

% Convert speed from rad/s to RPM
speed_rpm = (speed_rads * 60) / (2 * pi);

% Calculate the power output in watts
power_output = torque_Nm .* speed_rads;

% Calculate the power input in watts
power_input = current_A .* voltage_V;

% Calculate efficiency
efficiency = (power_output ./ power_input) * 100;

% Generate Torque vs Speed Plot
figure;
plot(speed_rpm, torque_Nm, 'b-');
xlabel('Speed (RPM)');
ylabel('Torque (N-m)');
title('Torque vs Speed');
grid on;

coeffs = polyfit(speed_rpm, torque_Nm, 1);
stall_torque = coeffs(2); % Stall torque is the y-intercept
no_load_speed = -stall_torque / coeffs(1); % No-load speed, solve for x when y=0

% Add stall torque and no-load speed to the plot
hold on;
plot(0, stall_torque, 'ro', 'DisplayName', 'Stall Torque'); % Stall Torque point
plot(no_load_speed, 0, 'ro', 'DisplayName', 'No-Load Speed'); % No-load speed point
legend show;
hold off;
% Generate Efficiency vs Speed Plot
figure;
plot(speed_rpm, efficiency, 'r-');
xlabel('Speed (RPM)');
ylabel('Efficiency (%)');
title('Efficiency vs Speed');
grid on;
