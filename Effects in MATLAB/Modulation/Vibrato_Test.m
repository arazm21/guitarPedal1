clc; close; clear;

fs = 44.1e3; f = 440;
[x, t] = singen(fs,1,0,f,5);

%%
[x, fs] = audioread('slide_electric_clean_guitar-104333.mp3');
x = x(:,1);

%%
depth = 1; rate = 0.1; avg_delay = 0.8;
y = Vibrato(x, fs, depth, rate, avg_delay);

sound(y,fs); 

%%
spectrogram(y, 100, [], [], fs, 'yaxis'); % 'yaxis' shows frequency in Hz
colorbar; % Adds a colorbar for intensity


