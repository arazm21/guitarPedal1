function out = cubicDistortion(in, level, factorInd)
% out = cubicDistortion(in, level, factorInd);
% Adjust level for amount of distortion (0-1)
% Adjust factor for softer-harder clipping (1-5 integers)
factors = [3 5 7 9 11];
out = in - level*(1/factors(factorInd))*in.^factors(factorInd);

end

