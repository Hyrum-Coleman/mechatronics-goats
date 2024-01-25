clear, clc

% Load Raw Data
load('Plot_1_REV3.mat', 'rawData');
rawData = rawData(:, 1); % Use only the first column

% Create a figure
figure;

% Plot the original signal
subplot(3, 1, 1); % Subplot 1
plot(rawData);
title('Original Signal');
xlabel('Time');
ylabel('Amplitude');

% Perform FFT
mag = fft(rawData);
freq = 0:999;

% Plot the full spectrum
subplot(3, 1, 2); % Subplot 2
[m, ~, f, ~] = fft_rev2(rawData, 1000);

plot(f, m);
title('Full Spectrum');
xlabel('Frequency (Hz)');
ylabel('Magnitude');

% Filter out harmonics above the 5th
freq_5_copy = freq;
mag_5_copy = mag;
freq_5_copy(freq > 100) = 0;
mag_5_copy(freq > 100) = 0;

% Filter out harmonics above the 10th
freq_10_copy = freq;
mag_10_copy = mag;
freq_10_copy(freq > 200) = 0;
mag_10_copy(freq > 200) = 0;

% Plot the filtered spectrum
subplot(3, 1, 3); % Subplot 3
plot(f(1:100), m(1:100));
title('Spectrum up to the 5th Harmonic');
xlabel('Frequency (Hz)');
ylabel('Magnitude');

% Reconstruct the signal with limited harmonics
reconstructedSignal_5 = ifft(mag_5_copy);
reconstructedSignal_10 = ifft(mag_10_copy);
reconstructedSignal_full = ifft(mag);

% Add the first reconstructed signal plot to the same figure
figure;
subplot(3, 1, 1); % Subplot 1
plot(real(reconstructedSignal_5)); hold on;
title('Reconstructed Signal with up to 5 Harmonics');
xlabel('Time');
ylabel('Amplitude');

% Add the second reconstructed signal plot to the same figure
%subplot(3, 1, 2); % Subplot 1
plot(real(reconstructedSignal_10));
title('Reconstructed Signal with up to 5 Harmonics');
xlabel('Time');
ylabel('Amplitude');

% Add the third reconstructed signal plot to the same figure
%subplot(3, 1, 3); % Subplot 1
plot(real(reconstructedSignal_full));
title('Reconstructed Signal with up to 5 Harmonics');
xlabel('Time');
ylabel('Amplitude'); hold off;
