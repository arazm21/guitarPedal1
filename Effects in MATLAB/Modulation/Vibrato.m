function y = Vibrato(x, fs, d, r, avg_del)
% y = Vibrato(x, fs, d, r)
% x - Input signal
% fs - Sampling frequency
% d - Depth [0.0 - 1.0] 
% r - Rate [0.0 - 1.0] 

% LFO Frequency [0.5 - 7] Hz 
f_lfo = 0.5 + 6.5*r;

% Average Delay [5 - 10] ms
D0 = round( 0.001 * fs * (5 + 5*avg_del) );

% Depth [0.1 - 5] ms
D = round( 0.001 * fs * (0.1 + 4.9*d) );

D_MAX = D0+D;
N = length(x);
y = x;

n = 0;
for i = D_MAX:N
    if f_lfo*n >= fs; n = 0; end 
    
    nd = D0 + D*sin(2*pi*f_lfo*n/fs);
    nd_trunc = floor(nd);
    nd_ciel = ceil(nd);
    frac = nd - nd_trunc;
    y(i) = (1-frac)*x(i-nd_ciel) + frac*x(i-nd_trunc);

    n = n+1;
end

end

