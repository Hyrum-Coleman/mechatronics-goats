% FIR_MA.m
function Y = FIR_MA(X, N)
t = X(:, 1);
Y = zeros(size(X));
Y(:, 1) = t;
Y(1:N-1)=X(1:N-1);
for k=N:length(t)
    Y(k, 2) = sum(X(k-(N-1):k, 2))/N;
end
end