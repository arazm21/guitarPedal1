clc; clear; close;
%% 2. DELTA
x = zeros(1,5e3);
x(1) = 1;


%% 1. AUDIO
[x, fs] = audioread('acoustic-guitar.mp3');
x = x(:,1);

%% SET UP FILTER PARAMS
% wet and delay (0.0-1.0).
clc;
wet = 1; delay = 3;

CF_g = [0.805, 0.827, 0.783, 0.764];
CF_delays = delay*[36.04, 31.12, 40.44, 44.92]; % ms
CF_N = round( 0.001*fs*CF_delays );

AP_g = 0.7;
AP_delays = delay*[5 1.68 0.48];  % ms
AP_N = round( 0.001*fs*AP_delays );

Arrays_Size = sum(CF_N) + sum(AP_N) + max(CF_N)
Memory_KBytes = 0.001*4*Arrays_Size % fs = 44.1e3;
%% Calculate Filter Parameters
clc;
CF1a = CF_IIR(CF_N(1), CF_g(1));
CF2a = CF_IIR(CF_N(2), CF_g(2));
CF3a = CF_IIR(CF_N(3), CF_g(3));
CF4a = CF_IIR(CF_N(4), CF_g(4));

[AP1b, AP1a] = AP_comb(AP_N(1),AP_g);
[AP2b, AP2a] = AP_comb(AP_N(2),AP_g);
[AP3b, AP3a] = AP_comb(AP_N(3),AP_g);

%% Apply Filters
x_CF1 = filter(1,CF1a, x);
x_CF2 = filter(1,CF2a, x);
x_CF3 = filter(1,CF3a, x);
x_CF4 = filter(1,CF4a, x);
y = 0.25*(x_CF1+x_CF2+x_CF3+x_CF4);

y = filter(AP1b, AP1a, y);
y = filter(AP2b, AP2a, y);
y = filter(AP3b, AP3a, y);
y = (1-wet)*x + wet*y;
%%
%sound(x,fs)
