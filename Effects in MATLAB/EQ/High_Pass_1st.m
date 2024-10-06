function y = High_Pass_1st(x, fs, fc)
% y = High_Pass_1st(x, fs, fc)
wc_T = 2*tan(pi*fc/fs);
b0 = 2; b1 = -2;
a0 = 2 + wc_T; a1 = -2 + wc_T;

N = length(x);
y = zeros(1,N);

for i = 2:length(x)
    y(i) = b0*x(i) + b1*x(i-1) - a1*y(i-1);
    y(i) = y(i)/a0;
end

end

