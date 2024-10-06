function y = Peaking_2nd(x, fs, fc, f_bw, g)
% y = Peaking_2nd(x, fs, fc, f_bw, g)
% x - Input signal
% fs - Sampling frequency
% fc - Center frequency
% f_bw - Bandwidth
% g - Linear gain

Q = fc/f_bw;
wc = 2*fs*tan(pi*fc/fs);
wc_T = wc/fs;
wc_T2 = wc_T^2;

b0 = 4 + 2 * (g/Q) * wc_T + wc_T2;
b1 = 2 * wc_T2 - 8;
b2 = 4 - 2 * (g/Q) * wc_T + wc_T2;

a0 = 4 + 2 * 1/Q * wc_T + wc_T2;
a1 = 2 * wc_T2 - 8;
a2 = 4 - 2 * 1/Q * wc_T + wc_T2;

N = length(x);
y = zeros(1,N);

for i=3:N
   y(i) = b0*x(i) + b1*x(i-1) + b2*x(i-2) - a1*y(i-1) - a2*y(i-2);
   y(i) = y(i)/a0;
end

end

