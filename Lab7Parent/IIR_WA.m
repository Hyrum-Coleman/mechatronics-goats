% IIR_WA.m
function Y = IIR_WA(X, alpha)
t = X(:, 1);
Y = zeros(size(X));
Y(:, 1) = t;
Y(1, 2) = X(1, 2);
for k = 2:length(t)
    Y(k, 2)= alpha* X(k, 2)+ (1-alpha) * Y(k-1, 2);
end

end