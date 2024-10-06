%% Audio
clear; clc; close;
[x, fs] = audioread('eb_blues_riff-92879.mp3');
x = x(:,1);
%% Sine
clear; clc; close;
fs = 96e3;
[x, t] = singen(fs, 1, 0, 440, 2);

%% Flanger
% BL=0.7, FF=0.7, FB=0.7, Depth = 0-2 ms, MOD = 0.1-1 Hz
clc;
BL=0.9; FF=0.5; FB=0.7; Depth = 1.5; f_LFO = 0.5;

M0 = round( .001*.5*Depth*fs );
L = length(x);
y = zeros(1,L);
y(1:2*M0) = x(1:2*M0);

n = 0;
for i=2*M0:L % -2 for non causal spline interpolation
    if (1/fs*f_LFO*n >= 1); n = 0; end
    %O = round(  M0*( 1 + sin(2*pi*f_LFO*n/fs) )  );
    %y(i) = BL*x(i) + (FB-BL*FF)*x(i-O) + FB*y(i-O);
    
    % linear interpolation
    O = M0*( 1 + sin(2*pi*f_LFO*n/fs) );
    f = O - floor(O);
    M = floor(O);
    x_frac_delay = (1-f)*x(i-M) + f*x(i-M-1);
    y_frac_delay = (1-f)*y(i-M) + f*y(i-M-1);
    
    % spline interpolation - non causal
    %O = M0*( 1 + sin(2*pi*f_LFO*n/fs) );
    %f = O - floor(O);
    %M = floor(O);
    %x_frac_delay = f^3/6*x(i-M-1) + ( (1+f)^3 - 4*f^3 )/6*x(i-M) + 1/6*( (2-f)^3 - 4*(1-f)^3 ) * x(i-M+1) + (1-f)^3/6*x(i-M+2);
    %y_frac_delay = f^3/6*y(i-M-1) + ( (1+f)^3 - 4*f^3 )/6*y(i-M) + 1/6*( (2-f)^3 - 4*(1-f)^3 ) * y(i-M+1) + (1-f)^3/6*y(i-M+2);
    
    
    y(i) = BL*x(i) + (FF-BL*FB)*x_frac_delay + FB*y_frac_delay;
    n = n+1;
end
sound(y,fs)
%%
[B,A] = sos2tf(SOS,G); 
y1 = filter(B,A,y);


%%
sound(x,fs);
sound(y,fs);



