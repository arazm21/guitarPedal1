clear; clc; close;

fs = 96e3;
[x, t] = singen(fs, 1, 0, 220, 0.1);
%% params
fc = fs/10;
K = tan(pi*fc/fs);
% G - db Gain. V0 - linear Gain/
G = -6; V0 = 10^(G/20);
%% coeffs
den = V0+sqrt(2*V0)*K+K^2;
b0 = V0*(1+sqrt(2)*K+K^2)/den;
b1 = 2*V0*(K^2-1)/den;
b2 = V0*(1-sqrt(2)*K+K^2)/den;
a1 = 2*(K^2-V0)/den;
a2 = (V0-sqrt(2*V0)*K+K^2)/den;
%% filter
N = length(x);
y = zeros(1,N);

for i=3:N
    y(i) = b0*x(i) + b1*x(i-1) + b2*x(i-2) - a1*y(i-1) - a2*y(i-2); 
end
%% plot
plot(t,x,t,y); legend('IN','OUT');
grid on; grid minor; xlim([0 1e3/fs]);
%% analysis
f = linspace(0,fs/2,1e6);
b = [b0, b1, b2];
a = [1, a1, a2];
H = freqz(b,a,f,fs);
plot(f,db(abs(H)));
grid on; grid minor; 
xlabel('Frequencies (f)');
ylabel('Magnitude abs(H)');
