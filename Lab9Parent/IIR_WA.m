% IIR_WA.m
function Y = IIR_WA(X, alpha)
Y = zeros(size(X));
Y(1) = X(1);
for k = 2:length(X)
    Y(k)= alpha* X(k)+ (1-alpha) * Y(k-1);
end

end