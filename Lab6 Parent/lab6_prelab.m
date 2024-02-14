% Given values
R1 = 7e3;       % Resistance R1 in ohms
R2 = 22e3;      % Resistance R2 in ohms
C = 760e-9;     % Capacitance in farads

% Cutoff frequency in rad/s
fc = 1 / (2 * pi * R2 * C);

% Frequency range for plotting
f = 0:100; % Hz

% Calculate the gain for each frequency
gain_ = (R2 / R1) ./ sqrt(1 + (f ./ fc).^2);
gain_db = 20 * log10(gain);

% Display the gain at specific frequencies
zeroHzGain = sprintf('Gain at f=0 Hz: %f dB\n', gain_db(1));
tenHzGain = sprintf('Gain at f=10 Hz: %f dB\n', gain_db(f == 10));
hundredHzGain = sprintf('Gain at f=100 Hz: %f dB\n', gain_db(f == 100));

% Plot the gain using data points
figure;
plot(f, gain_db, 'bo', 'DisplayName', 'Frequency Response'); % 'bo' specifies blue circle markers
hold on;
plot([fc fc], ylim, 'r--', 'DisplayName', 'Cutoff Frequency');
text(70, 1, zeroHzGain);
text(70, -1, tenHzGain);
text(70, -3, hundredHzGain);
hold off;
title('Filter Gain vs Frequency');
xlabel('Frequency (Hz)');
ylabel('Gain |V_o/V_in| (dB)');
legend('Location', 'best');
grid on;

