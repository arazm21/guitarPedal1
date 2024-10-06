clc; clear; close;

fs = 96e3; f0 = 5e3;
[x, t] = singen(fs, 1, 0, f0, 0.1);
%% params
fc = 1e3; V0 = 2; 
y = High_Shelf_2nd(x,fs,V0,fc);
%% plot
plot(t,x,t,y); legend('IN','OUT');
grid on; grid minor; xlim([20/f0 25/f0]);