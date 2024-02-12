clc, clear, close all

% Parameters
R = 4.67e3;   % resistance in ohms
C = 0.1e-6;  % capacitance in farads
f_c = 1 / (2 * pi * R * C); % cutoff frequency
f = logspace(log10(10), log10(2e3), 200); % Frequency range from 10 Hz to 2 KHz

% Call the function for low pass filter
[magnitude_low_dB, phase_low_deg] = filter_response('low', R, C, f);

% Call the function for high pass filter
[magnitude_high_dB, phase_high_deg] = filter_response('high', R, C, f);

lab_f = [10 20 40 150 340 500 700 1000 2000];

low_pass_mag_data = [0.0333 0.0746 -0.0504 -0.7311 -2.6498 -4.3211 -6.1353 -8.2970 -12.4524];
low_pass_phase_data = [-1.6230, -3.2310, -6.4150, -22.5700, -42.6100, -52.8600, -60.7200, -67.3000, -74.5100];


high_pass_mag_data = [-20.0083, -17.8640, -15.8642, -7.5616, -3.1459, -1.8027, -1.0561, -0.5736, -0.1818];

high_pass_phase_data = [78, 75.6000, 77.7200, 65.7500, 46.7200, 36.6800, 28.8800, 22.2800, 14.9600];

% Plot the low pass filter response
plot_filter_response(f, magnitude_low_dB, phase_low_deg, f_c, 'Low', lab_f, low_pass_mag_data, low_pass_phase_data);

% Plot the high pass filter response
plot_filter_response(f, magnitude_high_dB, phase_high_deg, f_c, 'High', lab_f, high_pass_mag_data, high_pass_phase_data);


function [magnitude_dB, phase_deg] = filter_response(filter_type, R, C, f)
    % Calculate cutoff frequency
    f_c = 1 / (2 * pi * R * C);
    
    % Initialize output variables
    magnitude_dB = zeros(size(f));
    phase_deg = zeros(size(f));
    
    % Calculate response based on filter type
    switch filter_type
        case 'low'
            H = 1 ./ sqrt(1 + (f ./ f_c).^2);
            phase_rad = -atan(f ./ f_c);
        case 'high'
            H = (f ./ f_c) ./ sqrt(1 + (f ./ f_c).^2);
            phase_rad = atan(f_c ./ f);
        otherwise
            error('Invalid filter type. Use ''low'' or ''high''.');
    end
    
    % Convert magnitude to dB and phase to degrees
    magnitude_dB = 20 .* log10(H);
    phase_deg = phase_rad .* (180/pi);
end

function plot_filter_response(f, magnitude_dB, phase_deg, f_c, filter_type, lab_f, lab_mag, lab_phase)
    % Create a new figure
    figure;
    
    % Plot the magnitude response
    subplot(2, 1, 1);
    semilogx(f, magnitude_dB, 'DisplayName', 'Theoretical Response');
    hold on;
    plot(lab_f, lab_mag, 'o', 'Color', "#D95319", 'MarkerSize', 8, 'DisplayName', 'Lab Data');
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