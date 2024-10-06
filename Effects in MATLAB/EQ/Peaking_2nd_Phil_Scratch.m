%% filter parameters
clear; clc; close;

% Sampling frequency
fs = 100e6;
T = 1/fs;

% Cutoff frequency
fc = 4e3;

% Bandwith (f)
f_bw = 100;

% Linear gain
g = 0.1;

% frequency range for plotting
N = 1e6;
f_range = linspace(0, fs/2, N);

%% Analog
clc; close;
wc = 2*pi*fc;
w_bw = 2*pi*f_bw;
Q = wc/w_bw;

%%s = tf('s');
% Ha = ( (s/wc)^2 + (g/Q)*(s/wc) + 1 ) / ( (s/wc)^2 + (1/Q)*(s/wc) + 1 );
% bode(Ha);

B = [1/wc^2, (g/Q)*1/wc, 1];
A = [1/wc^2, (1/Q)*1/wc, 1];
[MAG, PHASE] = bode(B, A, 2*pi*f_range);
MAG = squeeze(MAG);
PHASE = squeeze(PHASE);

subplot(2,1,1); semilogx(f_range, MAG, 'k', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude');
subplot(2,1,2); semilogx(f_range, PHASE, 'k', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude');

%% Digital
clc; close;

wc = (2/T)*tan(wc*T/2);
wc_T = wc*T;
wc_T2 = wc_T^2;

b0 = 4 + 2 * (g/Q) * wc_T + wc_T2;
b1 = 2 * wc_T2 - 8;
b2 = 4 - 2 * (g/Q) * wc_T + wc_T2;

a0 = 4 + 2 * 1/Q * wc_T + wc_T2;
a1 = 2 * wc_T2 - 8;
a2 = 4 - 2 * 1/Q * wc_T + wc_T2;

b = [b0 b1 b2];
a = [a0 a1 a2];

Hd = freqz(b, a, 2*pi*f_range/fs);
mag = abs(Hd);
phase = angle(Hd); phase = rad2deg(phase);

subplot(2,1,1); semilogx(f_range, mag, 'r--', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude');
subplot(2,1,2); semilogx(f_range, phase, 'r--', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude');

%% plot both
clc; close;
subplot(2,1,1);
semilogx(f_range, MAG, 'k', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude'); hold on;
semilogx(f_range, mag, 'r--', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude');
legend("Analog", "Digital");
title("Peaking Filter", Fontsize=20);

subplot(2,1,2);
semilogx(f_range, PHASE, 'k', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude'); hold on;
semilogx(f_range, phase, 'r--', 'LineWidth', 1.3); 
grid on; grid minor; xlim([0, max(f_range)]); xlabel('Frequency (f)'); ylabel('Magnitude');

legend("Analog", "Digital");




