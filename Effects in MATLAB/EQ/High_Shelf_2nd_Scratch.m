clear; clc; close;

fs = 96e3;
[x, t] = singen(fs, 1, 0, 0.4e3, 0.1);
%% params
fc = 1e3;
K = tan(pi*fc/fs);
% G - db Gain. V0 - linear Gain/
G = -6; V0 = 2;
%% coeffs
if V0 >= 1
    den = 1+sqrt(2)*K+K*K;
    b0 = (V0+sqrt(2*V0)*K+K*K)/den;
    b1 = 2*(K*K-V0)/den;
    b2 = (V0-sqrt(2*V0)*K+K*K)/den;
    a1 = 2*(K*K-1)/den;
    a2 = (1-sqrt(2)*K+K*K)/den;
else
    den = 1+sqrt(2*V0)*K+V0*K*K;
    b0 = V0*(1+sqrt(2)*K+K*K)/den;
    b1 = 2*V0*(K*K-1)/den;
    b2 = V0*(1-sqrt(2)*K+K*K)/den;
    a1 = 2*(V0*K*K-1)/den;
    a2 = (1-sqrt(2*V0)*K+V0*K*K)/den;
end
%% filter
N = length(x);
y = zeros(1,N);

for i=3:N
    y(i) = b0*x(i) + b1*x(i-1) + b2*x(i-2) - a1*y(i-1) - a2*y(i-2); 
end
%% plot
plot(t,x,t,y); legend('IN','OUT');
grid on; grid minor; xlim([0 1e3/fs]);
%% filter analysis
f = linspace(0,fs/2,1e6);
b = [b0, b1, b2];
a = [1, a1, a2];
H = freqz(b,a,f,fs);
semilogx(f,(abs(H)));
grid on; grid minor; 
title('Magnitude Response (dB)')
xlabel('Frequency');
ylabel('Magnitude');
xlim([0, f(end)]);