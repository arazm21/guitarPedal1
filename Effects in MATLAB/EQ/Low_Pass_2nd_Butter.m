function y = Low_Pass_2nd_Butter(x, fs, fc)
% y = Low_Pass_2nd_Butter(x, fs, fc)

wc_T = 2*tan(pi*fc/fs);
Q = 0.7071;
b = wc_T^2*[1 2 1];
a = [4 + 2*wc_T/Q + wc_T^2, -8 + 2*wc_T^2 , 4 - 2*wc_T/Q + wc_T^2];

N = length(x);
y = zeros(1,N);

for i = 3:N
    y(i) = b(1)*x(i) + b(2)*x(i-1) + b(3)*x(i-2) - a(2)*y(i-1) - a(3)*y(i-2);
    y(i) = y(i)/a(1);
end

end

