function y = High_Shelf_2nd(x,fs,V0,fc)
% y = High_Shelf_2nd(x,fs,V0,fc)
% x - Input 
% y - Output
% fs - Sampling frequency
% V0 - Linear Gain
% fc - cutoff frequency

K = tan(pi*fc/fs);

if V0 >= 1 % Boost
    den = 1+sqrt(2)*K+K*K;
    b0 = (V0+sqrt(2*V0)*K+K*K)/den;
    b1 = 2*(K*K-V0)/den;
    b2 = (V0-sqrt(2*V0)*K+K*K)/den;
    a1 = 2*(K*K-1)/den;
    a2 = (1-sqrt(2)*K+K*K)/den;
else % Cut
    den = 1+sqrt(2*V0)*K+V0*K*K;
    b0 = V0*(1+sqrt(2)*K+K*K)/den;
    b1 = 2*V0*(K*K-1)/den;
    b2 = V0*(1-sqrt(2)*K+K*K)/den;
    a1 = 2*(V0*K*K-1)/den;
    a2 = (1-sqrt(2*V0)*K+V0*K*K)/den;
end

N = length(x);
y = zeros(1,N);

for i=3:N
    y(i) = b0*x(i) + b1*x(i-1) + b2*x(i-2) - a1*y(i-1) - a2*y(i-2); 
end

end

