function a = CF_IIR(N,g)
% function a = CF_IIR(N,g)
% H = 1/a;
a = [1, zeros(1,N-1), -g];
end

