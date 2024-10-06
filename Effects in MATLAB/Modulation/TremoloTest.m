% Define parameters
clc;close;clear
f_LFO = 8; % Frequency of LFO - [0.1 - 25] Hz.
fs=44100; A = 0.4;  f=440; DC=0; dur=5;
depth = 0.2; % Depth of effect - [0 - 1];
[x, t] = singen(fs,A,DC,f,dur);
LFO = sawtooth(2*pi*f_LFO*t,0.5);

y = x.*( (1-depth) + depth*LFO );
%% 4-th order low pass filter
y_filtered = fourth_order_low_pass(y);

%%
subplot(3,1,1); plot(t,x); xlim([0 2/f_LFO]); yline(0); ylim([-1.2*A 1.2*A])
subplot(3,1,2); plot(t,LFO, 'r'); xlim([0 2/f_LFO]); yline(0);
subplot(3,1,3); plot(t,y, 'g'); xlim([0 2/f_LFO]); yline(0); ylim([-1.2*A 1.2*A])
sound(1.7*y,fs)







%%
function y_filtered = fourth_order_low_pass(y)
% fc = 10k;
G = [0.3099; 0.2232; 1.0000];
SOS = [ 1.0000    2.0000    1.0000    1.0000   -0.2111    0.4507;
        1.0000    2.0000    1.0000    1.0000   -0.1521    0.0449];
[b, a] = sos2tf(SOS, G);
y_filtered = filter(b,a,y);

% plots
[Y w] = DTFT(y,2^10);
subplot(2,1,1); plot(w/pi, abs(Y)); xlim([0 1]); ylim([0 2]);

[Y_filtered w] = DTFT(y_filtered,2^10);
subplot(2,1,2); plot(w/pi, abs(Y_filtered)); xlim([0 1]); ylim([0 2]);
end