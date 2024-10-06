%% sine input
clear; clc; close;
depth = 0.7; rate = 0.3; 

f0 = 440; DC = 0; fs = 96e3; 
[x, t] = singen(fs, 1, DC, f0, 4);
y = Tremolo(x,fs,depth, rate);
subplot(2,1,1); plot(t,x); grid on; grid minor; xlim([0 2e2/f0]);
subplot(2,1,2); plot(t,y); grid on; grid minor; xlim([0 2e2/f0]);
sound(y,fs)
%% audio input

clear; clc; close;
[x, fs] = audioread('eb_blues_riff-92879.mp3');
x = x(:,1);
x = x';
delay_time = length(x)/fs;
depth = 0.6; rate = 0.2; 
y = Tremolo(x, fs, depth, rate);
sound(x,fs); pause(delay_time+1); sound(y,fs);

%%
clc; close all;
subplot(2,1,1);
spectrogram(x, 100, [], [], fs, 'yaxis'); % 'yaxis' shows frequency in Hz
colorbar; % Adds a colorbar for intensity

subplot(2,1,2);
spectrogram(y, 100, [], [], fs, 'yaxis'); % 'yaxis' shows frequency in Hz
colorbar; % Adds a colorbar for intensity