function [X, Wmu, Wcov] = UnscentedTransformSigmaPoints(mu, cov, n, kappa, alpha)
%Calculates the sigma points for the unscented transform given the mean,
%covariance, lambda coefficient (distance from mean), and number of
%dimensions. This function will calculate 2n+1 sigma points.
%INPUTS:
%       mu - vector containing mean values of random variables
%       cov - covariance matrix of random variables
%       n - number of mean values
%       kappa - scaling factor which influence how far the sigma points are
%           from the mean. Kappa is of similar magnitude to n.
%       alpha - scaling factor which influence how far the sigma points are
%           from the mean. Its value is between 0 < alpha <= 1
%OUTPUTS:
%       X - sigma point matrix, where each row is a new sigma point
%       Wmu - sigma mean weights
%       Wcov - sigma covariance weights
%Author:Jasprabhjit Mehami, 13446277

X = zeros(2*n,length(mu));

beta = 2; %Ideal value for gaussian random variable

%Intermediate value used to calculate sigma points and weights
lambda = (alpha^2)*(n+kappa)-n;

%The variation that will be applied to the mean to get all sigma points
a = (n+lambda).*cov;
a = sqrtm(a);

%Apply variation to calculate sigma points. Sigma points are +/- on either
%side of the mean
for i = 1:n
    X(i,:) = mu + a(i,:);
    X(n+i,:) = mu - a(i,:);
end

%The mean is always the first sigma point
X = [mu; X];

%Mean and covariance weights
w = 1/(2*(n+lambda));
Wmu = w.*ones(2*n,1);
Wcov = w.*ones(2*n,1);

%first sigma point has different mean and covariance weights
Wmu = [lambda/(n+lambda); Wmu];
Wcov = [Wmu(1) + (1 - alpha^2 + beta); Wcov];

end

