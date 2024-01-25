clear; clc; close all;
load("square_data.mat");

[Mag, phase, freq, G] = fft_sample(rawData(:, 1), 1000);

%% Plotting Pt. 1
tlo = tiledlayout(2,1);

nexttile(1);
plot(timeData, rawData(:,1));
title("Raw Signal");
xlabel("Time [s]"); ylabel("Signal [V]");

nexttile(2);
plot(freq, Mag);
title("Spectral Analysis");
xlabel("Frequency [Hz]"); ylabel("Mag [V]");

%% Plotting Pt. 2 -- The Inverse
fiveIndex = find(freq > 90, 1, "first");
H = G;
H(fiveIndex:end-fiveIndex) = 0;
inverseData5 = ifft(H);

tenIndex = find(freq > 210, 1, "first");
J = G;
J(tenIndex:end-tenIndex) = 0;
inverseData10 = ifft(J);

inverseDataAll = ifft(G);

figure();
plot(timeData, real(inverseData5));
hold on;
plot(timeData, real(inverseData10));
plot(timeData, inverseDataAll, "k");

title("IFFT Truncation Experiment");
xlabel("Time [s]"); ylabel("Reconstructed Signal [V]");
legend("5 Harmonics", "10 Harmonics", "All Harmonics", "Location", "eastoutside");


