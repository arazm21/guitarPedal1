
%% Sinus input
clear; clc; close;
fs = 96e3;
[x, t] = singen(fs, 0.7, 0, 440, 2);
%% Audio input
clear; clc; close;
[x,fs] = audioread('eb_blues_riff-92879.mp3');

%% variable Ammount
y1 = Tanh_Distortion(x,1);
y2 = Tanh_Distortion(x,2);
y3 = Tanh_Distortion(x,4);
y4 = Tanh_Distortion(x,8);
%%
doPlots(t,x,y1,y2,y3,y4); xlim([0 500/fs]);
legend('x', 'Ammount=1', 'Ammount=2', 'Ammount=4', 'Ammount=8');
%%



%%
function doPlots(t,x,y1,y2,y3,y4)
plot(t,x,'r-.','LineWidth', 1.5); hold on;
plot(t,y1, 'LineWidth', 0.8); hold on;
plot(t,y2, 'LineWidth', 0.8); hold on;
plot(t,y3, 'LineWidth', 0.8); hold on;
plot(t,y4, 'LineWidth', 0.8); 
grid on; grid minor;
end

