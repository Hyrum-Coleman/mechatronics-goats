function[mag, phase, freq, power] = fft_rev2(data, Fs)
    G = fft(data);
    res = Fs/length(data);
    Fn = Fs/2;
    freq = 0:res:Fn;
    mag = abs(G(1:length(freq)))/length(data);
    mag(2:end-1) = 2*mag(2:end-1);
    power = mag.^2;
    phase = angle(G)*180/pi;
end
    