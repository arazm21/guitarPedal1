clear; clc; close;

[x, fs] = audioread('eb_blues_riff-92879.mp3');
x = x(:,1);
level = 1; depth = 1; rate = 0.8;
y = Flanger(x, fs, level, depth, rate);
sound(y,fs);
%%
G = [0.334785284060598;1];
SOS = [1,2,1,1,0.162874799277944,0.176266336964449];
[b,a] = sos2tf(SOS,G);
y1 = filter(b,a,y);
sound(y1,fs)
%%
[X w] = DTFT(x,1024);
Y = DTFT(y,1024);


subplot(2,1,1);
plot(w/2/pi*fs, abs(X), '--g','LineWidth', 1); 
hold on; plot(w/2/pi*fs, abs(Y), '-.');
xlim([0 2/pi*fs/10]); legend('X','Y'); grid on;

subplot(2,1,2);
plot(w/2/pi*fs, angle(X), '--g','LineWidth', 1); hold on; plot(w/2/pi*fs, angle(Y), '-.');
xlim([0 3e4/fs]); legend('X','Y'); grid on; 

