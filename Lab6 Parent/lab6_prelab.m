clear, clc, close all;

% Given values
R1 = 10e3;       % Resistance R1 in ohms
R2 = 33e3;      % Resistance R2 in ohms
C = 470e-9;     % Capacitance in farads

% Cutoff frequency in rad/s
fc = 1 / (2 * pi * R2 * C);

% Frequency range for plotting
f = logspace(0, log10(2e3), 200); % Frequency range from 0 Hz to 2 KHz

% Calculate the gain for each frequency
gain = (R2 / R1) ./ sqrt(1 + (f ./ fc).^2);
gain_db = 20 * log10(gain);
phase_deg = 180 - atand(f ./ fc);

R = 164.4e3;
C_2nd = 1e-6;
Rg = 6.8e3;
k = 1.588;

% Lab data entry
lab_f = [1, 5, fc, 20, 50, 100, 1000];
lab_mag_db_1st_order = [10.18405045, 9.201296894, 7.293388084, 3.27089843, -3.473800537, -8.817478359, -28.23731027];
lab_phase_1st_order = [174.2, 153.5, 132.5, 117.8, 103.9, 97.5, 94.5];

lab_mag_db_2nd_order = [3.960429289, 3.257796049, 0.983430702, -6.301870076, -20.7839275, -30.46628257, -48.34008982];
lab_phase_2nd_order = [-8.163, -40.82, -80, -125.5, -157.9, -168, -174.3];

plot_filter_response(f, gain_db, phase_deg, fc, "Active Low Pass", lab_f, lab_mag_db_1st_order, lab_phase_1st_order, lab_mag_db_2nd_order, lab_phase_2nd_order);

function plot_filter_response(f, magnitude_dB, phase_deg, f_c, filter_type, lab_f, lab_mag, lab_phase, mag_2nd, phase_2nd)
    % Create a new figure
    figure;
    
    % Plot the magnitude response
    subplot(2, 1, 1);
    semilogx(f, magnitude_dB, 'DisplayName', 'Theoretical Response');
    hold on;
    plot(lab_f, lab_mag, 'o', 'Color', "#D95319", 'MarkerSize', 8, 'DisplayName', 'Lab Data 1st Order');
    plot(lab_f, mag_2nd, 'o', 'Color', "#8249DA", 'MarkerSize', 8, 'DisplayName', 'Lab Data 2nd Order');
    hold off;

    
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title([filter_type ' Pass Filter Magnitude Response']);
    legend('Location', 'best');
    hold on;
    plot([f_c f_c], ylim, 'r--', 'DisplayName', 'Cutoff Frequency'); % Cut-off frequency line
    hold off;
    
    % Plot the phase response
    subplot(2, 1, 2);
    semilogx(f, phase_deg, 'DisplayName', 'Theoretical Response');
    hold on;
    plot(lab_f, lab_phase, 'o', 'Color', "#D95319", 'MarkerSize', 8, 'DisplayName', 'Lab Data');
    plot(lab_f, phase_2nd, 'o', 'Color', "#8249DA", 'MarkerSize', 8, 'DisplayName', 'Lab Data 2nd Order');
    hold off;

    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Phase (degrees)');
    title([filter_type ' Pass Filter Phase Response']);
    legend('Location', 'best');
    hold on;
    plot([f_c f_c], ylim, 'r--', 'DisplayName', 'Cutoff Frequency'); % Cut-off frequency line
    hold off;
    
    % Add a main title to the figure
    sgtitle(['Frequency Response of ' filter_type ' Pass Filter']);
end