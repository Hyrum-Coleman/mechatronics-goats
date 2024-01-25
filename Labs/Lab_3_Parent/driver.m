clear, clc, close all

load('Lab3_prelab_6.mat'); 

sample_rate = 500;
[Mag, phase, freq] = fft_to_spec(data, sample_rate);

figure;
plot(freq, Mag);
xlabel('Frequency (Hz)');
ylabel('Magnitude (Volts)');
title('Magnitude Spectrum');
grid on;

saveas(gcf, 'FFT_Magnitude_Plot.png');
