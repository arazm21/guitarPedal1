function y = Sgn_Exp_Distortion(x, a)
% y = Sgn_Exp_Distortion(x, ex, f)
% a - Ammount (2.0 - inf) default = 2 
y = sign(x) .* ( 1 - abs(1.5*x - sign(x)).^a  );
end

