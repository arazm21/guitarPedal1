function [y, t] = singen(fs,A,DC,f, dur)
% [y,t] = singen(fs,A,DC,f, dur)
% generate sine wave
% fs - sampling frequency
% A - aplitude of sine wave
% DC - DC offset
% f - frequency of sine wave
% dur - duration in seconds

T=1/fs; 
t = 0:T:dur-T;
y = DC + A*sin(2*pi*f*t);
end

