clear; clc; close;


fs = 96e3; fc = 1e3;
[x, t] = singen(fs, 1, 0, 440, 0.001);
y = Low_Pass_2nd_Butter(x, fs, fc);
%%
subplot(2,1,1); stem(x); grid on; grid minor; 
subplot(2,1,2); stem(y); grid on; grid minor;