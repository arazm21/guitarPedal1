%% Attempt 3 DAFX
clear; clc; close;
fs = 100e3;
fc = 400; Wc = 2*pi*fc/fs; G = 6;
f = 200;
[x,t] = singen(fs, 1, 0, f, 0.02);
y = lowshelving(x, Wc, G);
plot(t,x,t,y); grid on; grid minor; legend('input','outoput');
max(y)
%% raghac meshleba aq. zemot fs-is shecvlaze reagirebs cudad.
V0 = 10^(G/20); H0 = V0-1;
if G >= 0
    c = (tan(pi*Wc/2)-1) / (tan(pi*Wc/2)+1); % boost
else
    c = (tan(pi*Wc/2)-V0) / (tan(pi*Wc/2)+V0); % cut
end
b = [1+H0/2+c, c+H0/2*c+H0/2];
a = [1 c];
[Hd w] = freqz(b,a);
semilogx(w/2/pi*fs, db(abs(Hd)));

%% Attempt 2
clear; clc; close;
f0 = 200; W0 = 2*pi*f0;
B0 = 1; Q = 0.7071;;
s = tf('s');
Ha = 1 + W0^2/(s^2 + s*W0/Q + W0^2)
[mag,phase,wout] = bode(Ha, linspace(0,2*pi*5e3, 1024));
mag = squeeze(mag);
wout = squeeze(wout);
semilogx(wout/2/pi, mag); grid on, grid minor;
xlabel('Frequencies'); ylabel('Magnitude');
%% Attempt 1, 1st orders
clear; clc; close;
W0 = 2*pi*500;
B0 = 1; Q = 0.7071;

B = [1, 1 + W0*(B0+1)];
A = [1 , W0];
Ha = tf(B,A);
W = linspace(0,5e3*2*pi,20e3);
Haa = freqs(B,A,W);
semilogx(W/2/pi,db(abs(Haa))); xlabel('Frequency (f)');
%bode(Ha,{0, 2*pi*60e3})




%% From DAFX
function y = lowshelving (x, Wc, G)
% y = lowshelving (x, Wc, G)
% Author: M. Holters
% Applies a low-frequency shelving filter to the input signal x.
% Wc is the normalized cut-off frequency 0<Wc<1, i.e. 2*fc/fS
% G is the gain in dB
V0 = 10^(G/20); H0 = V0 - 1;
if G >= 0
    c = (tan(pi*Wc/2)-1) / (tan(pi*Wc/2)+1); % boost
else
    c = (tan(pi*Wc/2)-V0) / (tan(pi*Wc/2)+V0); % cut
end
xh = 0;
for n = 1:length(x)
    xh_new = x(n) - c*xh;
    ap_y = c * xh_new + xh;
    xh = xh_new;
    y(n) = 0.5 * H0 * (x(n) + ap_y) + x(n); % change to minus for HS
end
end