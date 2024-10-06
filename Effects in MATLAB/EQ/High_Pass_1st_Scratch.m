%% Initialize Parameters
clear; clc; close;
fs = 96e3;
T = 1/fs;
f0 = 2;
w0 = f0*2*pi;
f_range = linspace(0, 5, 8*1024);

%% Analog
clc;

B = [1 0];
A = [1, w0];

Ha = freqs(B,A, 2*pi*f_range);
plot(2*pi*f_range, db(abs(Ha)) ); grid on; grid minor;
hold on; yline(-3);

%Ha = tf(B,A);
%bode(Ha,{0, 2*pi*fs/2}); grid on; grid minor

%% Discrete
clc;
wc = 2/T*tan(w0*T/2);
b0 = 2; b1 = -2;
a0 = 2 + wc*T; a1 = -2 + wc*T;
b = [b0 b1];
a = [a0 a1];

z = tf('z',T);
Hd = ( b(1) + b(2)*z^-1 )/ ( a(1) + a(2)*z^-1 );
bode(Hd, {0 , 5*2*pi}); grid on; grid minor;
%% Discrete Plot no bode
ww = linspace(0, 0.0001*pi,1024);

Hdd = ( b(1) + b(2)*exp(-1j*ww) )./ ( a(1) + a(2)*exp(-1j*ww) );
plot(ww*fs/2/pi, db(abs(Hdd))); hold on; yline(-3);



%% Plot Both bode
close; clc;
Ha = tf(B,A);
bode(Hd,Ha, {0 ,5*2*pi}); grid on; grid minor;
legend('Analog','Discrete');



%% Plot Both fail
subplot(2,1,1); plot(f_range,db(abs(Ha)), f_range,db(abs(Hd))); 
grid on; grid minor;


