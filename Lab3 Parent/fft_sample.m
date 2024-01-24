function [ Mag, phase, freq ] = fft_sample(data, sample_rate )
G = fft(data);

spectralResolution = sample_rate/length(data);

f_Nyquist = sample_rate / 2;

freq = 0:spectralResolution:f_Nyquist;

Mag = abs(G(1:length(freq))) / length(data);
Mag(2:end-1) = 2*Mag(2:end-1);

Power = Max.^2;

phase = angle(G) * 180 / pi;

end