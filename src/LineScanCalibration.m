function [optPhi, res] = LineScanCalibration(worldCoord, v, phi0, optmAlgStruct)
% Carry out non-linear least-squares optimisation to determine the calibration for the linescan
% frame camera system
%INPUTS:
%       worldCoord - vector of feature points in world coordinates
%       v - vector of feature pixels
%       phi0 - initial guess of parameters from closed form solution
%       optmAlgStruct - passed in optimisation options structure
%OUTPUT:
%       optPhi - optimised camera parameters
%       resNorm - sum of residual squared
% 
% Author: Jasprabhjit Mehami, 13446277

%The optimisation function
f = @(H)LinescanPinholeOptReprojection(H, worldCoord, v);

%run trust-region first
if optmAlgStruct.algo == 2
    
    %Perform optimisation
    optPhi = lsqnonlin(f,phi0, optmAlgStruct.LBounds , optmAlgStruct.UBounds, optmAlgStruct.optOptionsTrust);
    
    %optimised parameters now become initial guess
    phi0 = optPhi; 
end

%levenberg-marquardt 
[optPhi,~, residuals] = lsqnonlin(f,phi0, [] , [], optmAlgStruct.optOptionsLeven);

res = norm(residuals)/length(residuals);

end