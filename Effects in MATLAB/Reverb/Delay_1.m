clear; clc; close;

% audio input
[x, fs] = audioread('acoustic-guitar.mp3');
x = x(:,1);

% delta input
x = zeros(1,5e3);
x(1) = 1;

% params
g = 0.9;
ammount = 0.6;
delay = 300; % ms

N = round( 0.001*fs*delay );
[b, a] = AP_comb( N , 0.7 );
y = filter(b,a, x);

% CF give basically echo / bad reverb
%a = CF_IIR(N, g);
%y = filter(1,a,x);

%y = ammount*y + (1-ammount)*x;
%sound(y,fs);

Memory_KBytes = 0.001*4*N*2


