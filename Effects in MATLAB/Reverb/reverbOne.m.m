%% part 0 - simple delay line
clear ;clc; close;
fs = 96e3; delay = 2; %(ms)
N = round(0.001*delay*fs)
h = [zeros(1,N), 1];
%%
[x t] = singen(fs, 1, 0, 440, 2);
y = conv(h,x); y=y(1:length(x));
plot(t,x,t,y); 
legend('input','output');
xlim([0 2e3/fs])

%% part 1 - Comb filters



%% 1.1 ffc - feed forward comb filter

%% 1.2 fbc - feed backwards comb filter


%% 1.3 above two combined
fvtool([1],[1 0 0 0 0 -0.8])


%% part 2 








