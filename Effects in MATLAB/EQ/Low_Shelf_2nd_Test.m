clc; clear; close;

fs = 96e3;
[x, t] = singen(fs, 1, 0, 220, 0.1);
%% params
fc = 500; V0 = 1/5; 
y = Low_Shelf_2nd(x,fs,V0,fc);
%% plot
plot(t,x,t,y); legend('IN','OUT');
grid on; grid minor; xlim([100/fs 1e3/fs]);