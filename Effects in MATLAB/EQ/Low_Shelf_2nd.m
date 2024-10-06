function y = Low_Shelf_2nd(x,fs,V0,fc)
% y = Low_Shelf_2nd(x,fs,V0,fc)
% x - Input 
% y - Output
% fs - Sampling frequency
% V0 - Linear Gain
% fc - cutoff frequency

K = tan(pi*fc/fs);
if V0 >= 1 % BOOST
    den = 1+sqrt(2)*K+K^2;
    b0 = (1+sqrt(2*V0)*K+V0*K^2)/den;
    b1 = 2*(V0*K^2-1)/den;
    b2 = (1-sqrt(2*V0)*K+V0*K^2)/den;
    a1 = 2*(K^2-1)/den;
    a2 = (1-sqrt(2)*K+K^2)/den;
else % CUT
    den = V0+sqrt(2*V0)*K+K^2;
    b0 = V0*(1+sqrt(2)*K+K^2)/den;
    b1 = 2*V0*(K^2-1)/den;
    b2 = V0*(1-sqrt(2)*K+K^2)/den;
    a1 = 2*(K^2-V0)/den;
    a2 = (V0-sqrt(2*V0)*K+K^2)/den;
end

N = length(x);
y = zeros(1,N);
for i=3:N
    y(i) = b0*x(i) + b1*x(i-1) + b2*x(i-2) - a1*y(i-1) - a2*y(i-2); 
end

end

