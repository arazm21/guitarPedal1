clear; clc; close;
% Apply soft clipping using arcTan distortion Algorithm.
% with Adjustable with two parameters: Gain and Alpha.

% Define parameters
fs = 44100; % Sample rate
f = 440; % Frequency of the sine wave
A = 1; % Amplitude of the sine wave
dur = 2; % Duration of signal in seconds
[x, t] = singen(fs,A,0,f,dur);
%%
[x, fs] = audioread('slide_electric_clean_guitar-104333.mp3');
%%

y = zeros( 5, length(x) );

y(1,:) = arcTanClipping(x, 1);
y(2,:) = arcTanClipping(x, 2);
y(3,:) = arcTanClipping(x, 4);
y(4,:) = arcTanClipping(x, 8);
y(5,:) = arcTanClipping(x, 64);
%%
plot(t, x, 'k-.', 'LineWidth', 1.1); hold on;
plot(t, y, 'LineWidth', 0.7); yline(0, 'r--');

% Adjust size and position of legend
lgd = legend('Original','alpha = 1', 'alpha = 2', 'alpha = 4', 'alpha = 8', 'alpha = 16');
lgd.FontSize = 12; % Set font size

xlabel('Time'); ylabel('Amplitude');
title('arcTan Soft Clipping with different Alphas');
grid on; grid minor;xlim([0 2*1/440]);
%%
