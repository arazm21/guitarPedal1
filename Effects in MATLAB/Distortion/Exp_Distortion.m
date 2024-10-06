function y = Exp_Distortion(x, a, s)
% y = Exp_Distortion(x, a, s)
% x - Input Signal
% a - Amount
% s - Symmetry Ratio

if nargin ~= 3
    error('Invalid Ammount of Arguments');
end

N = length(x);
y = zeros(1,N);

for i = 1:N
    if x(i)>0
        y(i) = 1 - exp(-s*a*x(i));
    elseif x(i)<0
        y(i) = -1 + exp(a*x(i));
    else
        y(i) = 0;
    end
    
end

end

