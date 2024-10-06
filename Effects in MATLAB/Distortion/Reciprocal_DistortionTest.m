
%% Sinus input
clear; clc; close;
fs = 96e3;
[x, t] = singen(fs, 0.7, 0, 440, 2);
%% Audio input
clear; clc; close;
[x,fs] = audioread('eb_blues_riff-92879.mp3');

%% variable a
y1a = Reciprocal_Distortion(x,1,1);
y2a = Reciprocal_Distortion(x,1,4);
y3a = Reciprocal_Distortion(x,1,8);
y4a = Reciprocal_Distortion(x,1,30);
%%
doPlots(t,x,y1a,y2a,y3a,y4a); xlim([0 500/fs]);
legend('x', 'a=1', 'a=4', 'a=8', 'a=30');

%% variable A
y1A = Reciprocal_Distortion(x,1,1);
y2A = Reciprocal_Distortion(x,2,1);
y3A = Reciprocal_Distortion(x,3,1);
y4A = Reciprocal_Distortion(x,4,1);
%%
doPlots(t,x,y1A,y2A,y3A,y4A); xlim([0 500/fs]);
legend('x', 'a=1', 'a=4', 'a=8', 'a=30');
%%
plotSpectrum(y4a);


%%
function doPlots(t,x,y1,y2,y3,y4)
plot(t,x,'r-.','LineWidth', 1.5); hold on;
plot(t,y1, 'LineWidth', 0.8); hold on;
plot(t,y2, 'LineWidth', 0.8); hold on;
plot(t,y3, 'LineWidth', 0.8); hold on;
plot(t,y4, 'LineWidth', 0.8); 
grid on; grid minor;
end

function plotSpectrum(x)
[H, w] = DTFT(x,1024);
subplot(1,1,1); plot(w/pi, abs(H)); xlim([0 1])
grid on; grid minor;
end


