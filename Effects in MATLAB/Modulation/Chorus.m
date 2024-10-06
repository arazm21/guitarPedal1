function y = Chorus(x, fs, level, ad1, d1, r1, ad2, d2, r2)
% y = Chorus(x, fs, level, ad1, d1, f1, ad2, d2, f2)
% x - Input Signal
% fs - Sampling Frequency (Hz)
% level - Effect Level (0.0 - 1.0)
% ad1, ad2 - Average delay (0.0 - 1.0) (ms)
% d1, d2 - Depth  (0.0 - 1.0) (ms)
% r1, r2 - Rate (0.0 - 1.0) (Hz)

if (nargin ~= 6 && nargin ~= 9), error('Invalid number or arguments'), end
if level<0, level=0; elseif level>1, level = 1; end

if ad1<0, ad1=0; elseif ad1>1, ad1 = 1; end
if d1<0, d1=0; elseif d1>1, d1 = 1; end 
if r1<0, r1=0; elseif r1>1, r1 = 1; end

    
% Average Delay (10.0-40.0) ms
m1 = 10 + 30*ad1;

% Depth (1.0 - 10.0) ms
depth1 = 1 + 9*d1;

% LFO Frequency (0.01-4.0) Hz;
f1 = 0.01 + 3.99*r1; 

y = x;
n=0;

M1 = round(.001*m1*fs);
D1 = round(.001*depth1*fs);
fn1 = f1/fs;

for i = M1+D1:length(x)
    if(fs == fn1*n); n = 0; end
    curr_delay = round(M1 + D1*sin(2*pi*fn1*n));
    
    if nargin ~= 6 ,y(i) = x(i) + x(i -  curr_delay);
    else, y(i) = x(i) + 0.5*x(i -  curr_delay); end
    n=n+1;
end

if nargin == 8 
    if ad2<0, ad2=0; elseif ad2>1, ad2 = 1; end
    if d2<0, d2=0; elseif d2>1, d2 = 1; end 
    if r2<0, r2=0; elseif r2>1, r2 = 1; end

    
    m2 = 10 + 30*ad2;
    depth2 = 1 + 9*d2;
    f2 = 0.01 + 3.9*r2; 
    
    M2 = round(.001*m2*fs);
    D2 = round(.001*depth2*fs);
    fn2 = f2/fs;
    
    n = 0;
    for i = M2+D2:length(x)
        if(fs >= fn2*n); n = 0; end
        curr_delay = round(M2 + D2*sin(2*pi*fn2*n));
        y(i) = x(i) + 0.5*x(i -  curr_delay);
        n=n+1;
    end
        
end

y = (1-level)*x + level*y; 
end

