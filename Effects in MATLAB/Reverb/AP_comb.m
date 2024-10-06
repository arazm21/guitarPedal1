function [b,a] = AP_comb(N,g)
% function [b,a] = AP_comb(N,g)
b = [-g, zeros(1,N-1), 1];
a = [1, zeros(1,N-1), -g];
end

