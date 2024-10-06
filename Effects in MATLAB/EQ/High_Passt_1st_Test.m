clear; clc; close;

f0 = 20; DC = 2;
fs = 96e3; fc1 = 0.01*f0; fc2 = 0.1*f0; fc3 = 2*f0;
[x, t] = singen(fs, 1, DC, 0.4, 10);
y1 = High_Pass_1st(x, fs, fc1);
y2 = High_Pass_1st(x, fs, fc2);
y3 = High_Pass_1st(x, fs, fc3);
%%
close;
plot(t,x,t,y1,t,y2,t,y3,'b'); hold on; yline(0);
grid on; grid minor; legend('x','y1','y2','y3');
xlim([0, 1e2/f0]); hold on; yline(DC);