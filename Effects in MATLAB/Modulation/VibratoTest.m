clear; clc; close;

[x, fs] = audioread('slide_electric_clean_guitar-104333.mp3');
x = x(:,1);

%%
[x, t] = singen(44100,1,0,440,5);

%%
clear; clc; close;
fs = 44.1e3;

recorder = audiorecorder(fs,16,1);
disp('Start Recorded');
recordblocking(recorder,5);
disp('Stopped Recording');
x = getaudiodata(recorder);
%%
d0 = 7; % AVG_DELAY (ms)
d1 = 2; % DEPTH (ms)
D0 = round(0.001*d0*fs);
D1 = round(0.001*d1*fs);
LFO_f = 1; % Hz

MAX_DELAY = D0+D1;
fn = LFO_f/fs;
y = x;
n = 0;
for i = MAX_DELAY:length(x)
    if(fs == fn*n); n = 0; end
    curr_delay = round(D0 + D1*sin(2*pi*fn*n));
    y(i) = x(i) + x(i -  curr_delay);
    n=n+1;
end
%%
sound(y,fs)
