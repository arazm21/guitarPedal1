function y = Tremolo(x, fs, depth, rate)
% x = Tremolo(x, fs, d, r);
% x - Input signal
% fs - Sampling frequency
% depth - Depth [0-1]
% rate - rate [0-1]

f_lfo = 0.1 + rate*11.9; % Hz

N = length(x); 
y = zeros(1,N);

n = 0;
for i = 1:N
   if(fs <= f_lfo*n); n = 0; end
   y(i) = x(i)*cos(2*pi*f_lfo/fs*n);
   n = n+1;
end

y = (1-depth)*x + depth*y;
end

