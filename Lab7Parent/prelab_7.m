clc, clear, close all;

data = load("Lab7_prelab4_noisydata.mat");
data = data.data;


Y_1 = IIR_WA(data, .75);

Y_2 = IIR_WA(data, .9);

F_1 = FIR_MA(data, 8);

F_2 = FIR_MA(data, 25);

plot_stuff(Y_1, data, 'IIR')
plot_stuff(Y_2, data, 'IIR')
plot_stuff(F_1, data, 'FIR')
plot_stuff(F_2, data, 'FIR')

function plot_stuff(data, originalData, filter_type)
    figure;
    
    grid on;

    plot(originalData(:, 1), originalData(:, 2), 'DisplayName', 'Original Data');
    hold on;
    plot(data(:, 1), data(:, 2), 'DisplayName', 'Filtered Data');
    hold off;
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    title([filter_type ' Digital Filter']);
    legend('Location', 'best');
end