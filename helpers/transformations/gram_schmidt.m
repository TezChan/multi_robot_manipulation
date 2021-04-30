function U = gram_schmidt(V)
[n, k] = size(V);
U = zeros(n, k);
U(:, 1) = V(:, 1) / norm(V(:,1));
for ii = 2:k
    U(:, ii) = V(:, ii);
    for jj = 1:ii - 1
        U(:, ii) = U(:, ii) - (U(:,jj)'*U(:,ii)) / (U(:,jj)'*U(:,jj)) * U(:, jj);
    end
    U(:, ii) = U(:, ii) / norm(U(:, ii));
end
end