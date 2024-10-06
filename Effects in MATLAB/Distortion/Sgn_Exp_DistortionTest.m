
%% Sinus input
clear; clc; close;
fs = 96e3;
[x, t] = singen(fs, 0.7, 0, 440, 2);
%% Audio input
clear; clc; close;
[x,fs] = audioread('eb_blues_riff-92879.mp3');

%% variable Ammount
y1e = Sgn_Exp_Distortion(x,2);
y2e = Sgn_Exp_Distortion(x,5);
y3e = Sgn_Exp_Distortion(x,10);
y4e = Sgn_Exp_Distortion(x,100);
%%
doPlots(t,x,y1e,y2e,y3e,y4e); xlim([0 500/fs]);
legend('x', 'ex=2', 'ex=5', 'ex=10', 'ex=100');
%%
plotSpectrum(y4e);

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


