% Parameters
R = 4.7e3;   % resistance in ohms
C = 0.1e-6;  % capacitance in farads
f_c = 1 / (2 * pi * R * C); % cutoff frequency
f = logspace(log10(1), log10(1e6), 200); % Frequency range from 1 Hz to 1 MHz

% Call the function for low pass filter
[magnitude_low_dB, phase_low_deg] = filter_response('low', R, C, f);

% Call the function for high pass filter
[magnitude_high_dB, phase_high_deg] = filter_response('high', R, C, f);

% Plot the low pass filter response
plot_filter_response(f, magnitude_low_dB, phase_low_deg, f_c, 'Low');

% Plot the high pass filter response
plot_filter_response(f, magnitude_high_dB, phase_high_deg, f_c, 'High');


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

function plot_filter_response(f, magnitude_dB, phase_deg, f_c, filter_type)
    % Create a new figure
    figure;
    
    % Plot the magnitude response
    subplot(2, 1, 1);
    semilogx(f, magnitude_dB);
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title([filter_type ' Pass Filter Magnitude Response']);
    hold on;
    plot([f_c f_c], ylim, 'r--'); % Cut-off frequency line
    hold off;
    
    % Plot the phase response
    subplot(2, 1, 2);
    semilogx(f, phase_deg);
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Phase (degrees)');
    title([filter_type ' Pass Filter Phase Response']);
    hold on;
    plot([f_c f_c], ylim, 'r--'); % Cut-off frequency line
    hold off;
    
    % Add a main title to the figure
    sgtitle(['Frequency Response of ' filter_type ' Pass Filter']);
end