function y = Chorus_i(x, fs, pre_delay, depth, rate, g)
% y = Chorus_i(x, fs, pre_delay, depth, rate, g)
% x - Input signal
% fs - Sampling Frequency
% pre_delay - Pre-Delay
% depth - Depth of Effect
% rate - Rate of LFO Oscillation
% g - Ammount of Chorus Effect

% Pre-Delay [5 - 40] ms
d0 = 5 + 35*pre_delay;

% Depth [.1 - 3] ms
d = .1 + 2.9*depth;

% LFO Frequency [.01 - 4.0] Hz;
f = 0.01 + 4.99*rate; 

D0 = round(.001*d0*fs);
D = round(.001*d*fs);
D_max = D0+D;

N = length(x); 
y = zeros(1,N);
y = x;

n = 0;
for i = D_max+1:N
    if f*n >= fs; n = 0; end
    
    %D_curr = round(D0 + D*sin(2*pi*f*n/fs));
    %y(i) = (1-g)*x(i) + g*x(i -  D_curr);
    
    D_curr = D0 + D*sin(2*pi*f*n/fs);
    D_floor = floor(D_curr);
    D_ceil = ceil(D_curr);
    frac = D_ceil-D_curr;
    
    x_frac_del = (1-frac)*x(i-D_ceil) + frac*x(i-D_floor);
    y(i) = (1-g)*x(i) + g*x_frac_del;
    
    n = n+1;
end

%y = (1-g)*x + g*y;

end

