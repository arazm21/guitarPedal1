%% Sinus input
clear; clc; close;
fs = 96e3;
[x, t] = singen(fs, 0.35, 0, 440, 2);
%% Audio input
%clear; clc; close;
%[x,fs] = audioread('eb_blues_riff-92879.mp3');

%% variable Ammount
y1a = Exp_Distortion(x,1,1);
y2a = Exp_Distortion(x,4,1);
y3a = Exp_Distortion(x,16,1);
y4a = Exp_Distortion(x,64,1);
%%
subplot(2,1,1);
doPlots(t,x,y1a,y2a,y3a,y4a); xlim([0 500/fs]);
legend('Original', 'gain=1', 'gain=4', 'gain=16', 'gain=32');
%% variable Symmetry
y1s = Exp_Distortion(x,2,1);
y2s = Exp_Distortion(x,2,4);
y3s = Exp_Distortion(x,2,16);
y4s = Exp_Distortion(x,2,32);
%%
subplot(2,1,2);
doPlots(t,x,y1s,y2s,y3s,y4s); xlim([0 500/fs]);
legend('Original input', 's=1', 's=4', 's=16', 's=32');
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

%% DTFTs
clc;
[fft_x, w] = DTFT(x,0);
fft_y1a = DTFT(y1a, 0);
fft_y2a = DTFT(y1a, 0);
fft_y3a = DTFT(y1a, 0);
fft_y4a = DTFT(y1a, 0);

plot(w/pi, abs(fft_x), 'r--'); hold on;
plot(w/pi, abs(fft_y1a)); hold on;
plot(w/pi, abs(fft_y2a)); hold on;
plot(w/pi, abs(fft_y3a)); hold on;
plot(w/pi, abs(fft_y4a)); hold on;
xlim([0 1])

%% spectrum
clc; close all;
pspectrum(x); hold on
pspectrum(y1a); hold on
pspectrum(y4a); hold on
xline(0.01, "--",linewidth = 1.2);
legend("input", "gain=1", "gain=16", "origianl frequency");