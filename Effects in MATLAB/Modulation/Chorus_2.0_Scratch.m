%% Audio
clear; clc; close;
[x, fs] = audioread('eb_blues_riff-92879.mp3');
x = x(:,1);
%% Sine
clear; clc; close;
fs = 96e3;
[x, t] = singen(fs, 1, 0, 440, 2);

%% Chorus
% BL=0.7, FF=1, FB=-0.7, Delay = 1-30 ms, Depth = 1-30 ms, MOD = LPN
clc;
BL=0.7; FF=0.7; FB=0.7; Delay = 25; Depth = 20; 

% Low Pass Noise -ish?
LPN_N = 1e5;
D1 = .001*Depth*fs;
LPN = round(.15*D1*randn(1,LPN_N));
LPN = LPN(find(LPN>0));
LPN_N = length(LPN);
LPN = sort(LPN,'descend');
LPN = [sort(LPN) LPN];
S = max(LPN);

%% Apply Effect
L = length(x);
y = zeros(1,L);

n = 1;
for i=S+1:L
    if (n > LPN_N); n = 1; end
    O = LPN(n);
    y(i) = BL*x(i) + (FF-BL*FB)*x(i-O) + FB*y(i-O);
    n = n+1;
end

sound(,fs)
