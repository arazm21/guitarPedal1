function out = arcTanClipping(in, a)
% out = arcTanClipping(in, a)
% a - Ammount (1.0 - inf)
out = zeros(1,length(in));
for i = 1:length(in)
    out(i) = (2/pi) * atan( in(i)*a );
end

end

