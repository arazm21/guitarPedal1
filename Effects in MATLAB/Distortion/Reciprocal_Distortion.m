function y = Reciprocal_Distortion(x,A,a)
% y = Reciprocal_Distortion(x,A,a)
% A - Top/Bottom Rail
% a - Reciprical Factor. Large a - more Distortion
y = sign(x) .* (A - 1./(a*abs(x)+1));
end

