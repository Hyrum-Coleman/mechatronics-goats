
clear;                  % Clear Workspace
close all;              % Close all open figures
clc;                    % Clear command window
delete(instrfindall);   % Delete any remaining instruments

% Initialize
COM = '/dev/cu.usbmodem101';                  % Set this to match Arduino Com port
BaudRate = 9600;               % Set this to match Arduino baud rate
s1 = serialport(COM,BaudRate);  % Open serial port
pause(13)  % Give the Arduino time to complete the experiment

% Read data
i = 1;
while s1.NumBytesAvailable > 0    
% Receive data from Arduino one row at a time
    data(i,:) = str2num(readline(s1));
    i = i + 1;
end

% Select variables of interest  
t = data(:,1);
V = data(:,2);
M = data(:,3);

% Plot results
figure
subplot(2,1,1);
plot(t,V,'r.')
xlabel("time (sec)")
ylabel("Velocity (rad/sec)")
title("Open Loop Response")

subplot(2,1,2);
plot(t,M,'b.')
xlabel("time (sec)")
ylabel("Motor Command")

% Close the port
clear s1