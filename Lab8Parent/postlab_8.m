clear, clc, close all;

%% Question 1

distance = [0.9
2
3
4
5
6
7
8
9
10];

voltage = [4.941
3.101
2.743
2.62
2.569
2.54
2.523
2.513
2.507
2.503];

plot(distance, voltage);

%% Question 2

angle = [0
30
60
90
120
150
180
210
240
270
300
330
360];

voltage = [4.943
4.755
3.919
2.46
0.652
0.353
0.2784
0.75
0.8737
2.132
3.908
4.74
4.92];

figure();
plot(angle, voltage, "DisplayName", "Voltage Data");
legend()
figure();
plot(sind(angle), voltage, "DisplayName", "Sin Data")
hold on;
plot(cosd(angle), voltage, "DisplayName", "CosData")
hold off;
legend();

%% Question 3

% Read the CSV file for yellow block
yellow_block_data = readtable('yellow_block.csv');
% Calculate mean and standard deviation for the yellow block
yellow_block_mean = mean(yellow_block_data{:,:});
yellow_block_std = std(yellow_block_data{:,:});

% Output the results for yellow block
fprintf('Yellow Block - Mean BGR Values: Blue: %f, Green: %f, Red: %f\n', yellow_block_mean);
fprintf('Yellow Block - Standard Deviation BGR Values: Blue: %f, Green: %f, Red: %f\n', yellow_block_std);

% Read the CSV file for red block
red_block_data = readtable('red_block.csv');
% Calculate mean and standard deviation for the red block
red_block_mean = mean(red_block_data{:,:});
red_block_std = std(red_block_data{:,:});

% Output the results for red block
fprintf('Red Block - Mean BGR Values: Blue: %f, Green: %f, Red: %f\n', red_block_mean);
fprintf('Red Block - Standard Deviation BGR Values: Blue: %f, Green: %f, Red: %f\n', red_block_std);

% Read the CSV file for blue block
blue_block_data = readtable('blue_block.csv');
% Calculate mean and standard deviation for the blue block
blue_block_mean = mean(blue_block_data{:,:});
blue_block_std = std(blue_block_data{:,:});

% Output the results for blue block
fprintf('Blue Block - Mean BGR Values: Blue: %f, Green: %f, Red: %f\n', blue_block_mean);
fprintf('Blue Block - Standard Deviation BGR Values: Blue: %f, Green: %f, Red: %f\n', blue_block_std);
