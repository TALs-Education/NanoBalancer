%% Linear stability of the 100 Hz LQR vs. BNO055 measurement delay
% Augmented state z = [x(k); x(k-1); ...; x(k-d)], u(k) = -K*x(k-d)
load(fullfile(fileparts(mfilename('fullpath')), 'NanoBalancer_params.mat'), 'NB');
Ad = NB.Ad; Bd = NB.Bd; K = NB.K; n = 4;
for d = 0:8
    N = n*(d+1); F = zeros(N);
    F(1:n,1:n) = Ad;
    F(1:n, n*d+(1:n)) = F(1:n, n*d+(1:n)) - Bd*K;
    for j = 1:d, F(n*j+(1:n), n*(j-1)+(1:n)) = eye(n); end
    rho = max(abs(eig(F)));
    if rho < 1, st = 'stable'; else, st = 'UNSTABLE'; end
    fprintf('delay %d samples (%3.0f ms): spectral radius %.4f  %s\n', d, d*NB.Ts*1e3, rho, st);
end
