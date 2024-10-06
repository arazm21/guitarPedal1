clc; clear; close; clear sound;

[x, fs] = audioread('acoustic-guitar.mp3');
x = x(:,1);

pre_delay = 0.2; depth = .6; rate = .5; g = .7;
y = Chorus_i(x, fs, pre_delay, depth, rate, g);

%y = Chorus(x, fs, 0.7, 0.8, 0.2, 0.2);

%sound(y,fs)
