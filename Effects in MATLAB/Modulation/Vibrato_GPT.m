function y = Vibrato_GPT(x, fs, depth, rate)
% y = Vibrato(x, fs, depth, rate)
% x - Input signal
% fs - Sampling frequency
% depth - Depth of the vibrato (max delay in seconds)
% rate - LFO rate (frequency of the vibrato in Hz)

% Parameters
N = length(x);
y = zeros(size(x));

% Convert the vibrato depth from seconds to samples
max_delay = depth * fs;  % Maximum delay in samples

% Create an LFO (Low-Frequency Oscillator)
lfo = sin(2 * pi * rate * (1:N) / fs);

% Apply vibrato
for n = 1:N
    % Calculate the varying delay based on LFO
    delay = round((1 + lfo(n)) * max_delay / 2);
    
    % Ensure delay does not exceed signal boundaries
    if n - delay > 0
        y(n) = x(n - delay);
    else
        y(n) = x(n);  % No delay at the start
    end
end
end
