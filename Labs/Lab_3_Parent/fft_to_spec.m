function [Mag, phase, freq] = fft_to_spec(data, sample_rate)
    % Number of data points
    N = length(data);

    % Perform the FFT
    fft_data = fft(data);

    % Double-sided spectrum
    P2 = abs(fft_data/N);

    % Single-sided spectrum
    P1 = P2(1:N/2+1);
    P1(2:end-1) = 2*P1(2:end-1);

    % Calculate the magnitude (volts)
    Mag = P1;

    % Calculate the phase (degrees)
    phase = angle(fft_data) * 180/pi;

    % Frequency domain (Hz)
    freq = sample_rate*(0:(N/2))/N;

    % Adjust the phase array size to match the magnitude array
    phase = phase(1:length(freq));
end
