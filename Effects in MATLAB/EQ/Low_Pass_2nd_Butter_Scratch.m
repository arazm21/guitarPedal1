%% Initialize Parameters
clear; clc; close;
fs = 96e3; T = 1/fs;
f0 = 3e3;
w0 = f0*2*pi;
Q = 0.707;

%% Analog
B = 1;
A = [1/(w0^2), 1/(Q*w0), 1];
Ha = tf(B,A);
bode(Ha,{0, 2*pi*fs/2}); grid on; grid minor

%% Discrete
w0 = 2/T*tan(w0*T/2); 
c = w0*T;

b = c^2*[1 2 1];
a = [4 + 2*c/Q + c^2, -8 + 2*c^2 , 4 - 2*c/Q + c^2];

z = tf('z',T);
Hd = ( b(1) + b(2)*z^-1 + b(3)*z^-2 )/ ( a(1) + a(2)*z^-1 + a(3)*z^-2 );
bode(Hd, {0 , 2*pi*fs/2}); grid on; grid minor;

%% Plot both
bode(Ha, Hd, {0 , 2*pi*fs/2}); grid on; grid minor; legend('Analog','Discrete');

%% Different methods for plotting - Don't work ;(((
f = linspace(0,fs/2, 1024);
W = f*2*pi;
H_analog = freqs(B, A, W);
plot(W/2/pi, db(abs(H_analog))); grid on; grid minor;
%%
[H, w] = freqz(b,a, 1024);
hold on; plot(f,db(abs(H))); 

%%
[H, w] = freqz(b,a, 1e5);
f = w*fs/2/pi; 

plot(f,abs(H)); yline(0.5); yline(0.7071); grid on; grid minor;
xlim([0 6000])
