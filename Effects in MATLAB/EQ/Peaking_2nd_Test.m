clear; clc; close;

fs = 96e3;
f1 = 100; f2 = 1000;
[x, t] = singen(fs, 1, 0, f1, 1);
x = x + singen(fs, 1, 0, f2, 1);
%% params
fc = 100; f_bw = 20; g = 0.1;
y = Peaking_2nd(x, fs, fc, f_bw, g);
%% plot
plot(t,x,t,y); legend('IN','OUT');
grid on; grid minor; xlim([10/f1 13/f1]);