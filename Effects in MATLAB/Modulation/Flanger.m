function y = Flanger(x, fs, level, d, r)
% y = Flanger(x, fs, level, ad, d, r)
% x - Input Signal
% fs - Sampling Frequency (Hz)
% level - Effect Level (0.0 - 1.0)
% d - Depth  (0.0 - 1.0) (ms)
% r - Rate (0.0 - 1.0) (Hz)

if nargin ~= 5, error('Invalid number or arguments'), end

if level<0, level=0; elseif level>1, level = 1; end
if d<0, d=0; elseif d>1, d = 1; end 
if r<0, r=0; elseif r>1, r = 1; end

% Depth (0.0 - 4.0) ms
depth = 4*d;

% LFO Frequency (0.1-1.0) Hz;
f = 0.1 + 0.9*r; 

D = round(.001*depth*fs);
fn = f/fs;

y = x;
n=0;

for i = D:length(x)
    if(fs == fn*n); n = 0; end
    
    %curr_delay = round(-1 + D * (1+sin(2*pi*fn*n)) );
    %y(i) = x(i) + level*x(i-curr_delay); 
    
    
    curr_delay = -1 + D * (1+sin(2*pi*fn*n));
    f = curr_delay - floor(curr_delay);
    frac_delay = (1-f)*x(i-floor(curr_delay)) + f*x(i-ceil(curr_delay));
    
    y(i) = x(i) + level*frac_delay;
    n=n+1;
end

end

