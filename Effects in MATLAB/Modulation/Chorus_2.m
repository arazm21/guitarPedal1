function y = Chorus_2(x, fs, level, ad, d, r)

% y = Chorus_2(x, fs, level, ad, d, f)
% x - Input Signal
% fs - Sampling Frequency (Hz)
% level - Effect Level (0.0 - 1.0)
% ad - Average delay (0.0 - 1.0) (ms)
% d - Depth  (0.0 - 1.0) (ms)
% r - Rate (0.0 - 1.0) (Hz)

if nargin ~= 6, error('Invalid number or arguments'), end

if level<0, level=0; elseif level>1, level = 1; end
if ad<0, ad=0; elseif ad>1, ad = 1; end
if d<0, d=0; elseif d>1, d = 1; end 
if r<0, r=0; elseif r>1, r = 1; end


% Average Delay (5.0-30.0) ms
m = 5 + 25*ad;

% Depth (1.0 - 5.0) ms
depth = 1 + 4*d;

% LFO Frequency (0.1-5.0) Hz;
f = 0.1 + 4.9*r; 

M = round(.001*m*fs);
D = round(.001*depth*fs);
fn = f/fs;

y = x;
n=0;

for i = M+D:length(x)
    if(fs == fn*n); n = 0; end
    curr_delay = round(M + D*sin(2*pi*fn*n));
    y(i) = x(i) + level*x(i -  curr_delay);
    n=n+1;
end

end

